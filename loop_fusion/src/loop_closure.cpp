// Scan Context / virtual-relative ICP / iSAM2: Giseop Kim, FAST_LIO_SLAM SC-PGO.
#include "loop_fusion/registration.h"
#include "scancontext/Scancontext.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/point_tests.h>
#include <pcl/io/pcd_io.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <boost/filesystem.hpp>
#include <Eigen/Geometry>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <future>
#include <iomanip>
#include <map>
#include <mutex>
#include <set>
#include <thread>

namespace loop_fusion {
struct Keyframe {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  size_t id;
  ros::Time stamp;
  gtsam::Pose3 raw, optimized;
  Cloud::Ptr cloud;
  Eigen::Matrix3d level_from_body;
};
using Keyframes = std::vector<Keyframe, Eigen::aligned_allocator<Keyframe>>;
struct MapSnapshot { Keyframes keyframes; size_t revision; };
struct Task {
  nav_msgs::OdometryConstPtr odom;
  sensor_msgs::PointCloud2ConstPtr cloud;
  std::shared_ptr<std::promise<std::string>> save;
};

class LoopClosure {
 public:
  LoopClosure() : private_("~") {
    std::ostringstream snapshot;
    auto number = [&](const std::string& name, double value) {
      double result; private_.param(name, result, value);
      snapshot << name << ": " << std::setprecision(17) << result << '\n';
      return result;
    };
    auto integer = [&](const std::string& name, int value) {
      int result; private_.param(name, result, value);
      snapshot << name << ": " << result << '\n'; return result;
    };
    auto string = [&](const std::string& name, const std::string& value) {
      std::string result; private_.param(name, result, value);
      snapshot << name << ": " << std::quoted(result) << '\n'; return result;
    };
    frame_ = string("frame_id", "camera_init");
    body_frame_ = string("body_frame_id", "body");
    const auto odom_topic = string("odom_topic", "/Odometry");
    const auto cloud_topic = string("cloud_topic", "/cloud_registered_body");
    const auto highrate_topic = string("highrate_odom_topic", "/Odometry_highrate");
    corrected_body_frame_ = string("corrected_body_frame_id", "body_loop");
    translation_ = number("keyframe/translation", 0.5);
    rotation_ = number("keyframe/rotation_deg", 10.0) * M_PI / 180.0;
    cloud_voxel_ = number("keyframe/voxel", 0.4);
    min_points_ = integer("keyframe/min_points", 100);
    sync_buffer_ = integer("sync/buffer_size", 200);
    max_queue_ = integer("sync/max_pending_pairs", 3000);
    sc_.PC_NUM_RING = integer("sc/rings", 20);
    sc_.PC_NUM_SECTOR = integer("sc/sectors", 60);
    sc_.PC_MAX_RADIUS = number("sc/max_radius", 80.0);
    sc_.PC_UNIT_SECTORANGLE = 360.0 / sc_.PC_NUM_SECTOR;
    sc_.PC_UNIT_RINGGAP = sc_.PC_MAX_RADIUS / sc_.PC_NUM_RING;
    sc_.LIDAR_HEIGHT = number("sc/height_offset", 2.0);
    sc_.SC_DIST_THRES = number("sc/distance_threshold", 0.3);
    sc_.NUM_EXCLUDE_RECENT = integer("sc/exclude_recent", 30);
    sc_.NUM_CANDIDATES_FROM_TREE = integer("sc/candidates", 3);
    private_.param("sc/gravity_align", gravity_align_, false);
    snapshot << "sc/gravity_align: " << (gravity_align_ ? "true" : "false") << '\n';
    detection_period_ = 1.0 / number("sc/detection_frequency", 1.0);
    min_interval_ = number("sc/min_time_interval", 0.0);
    loop_interval_ = number("sc/accepted_loop_interval", 0.0);
    submap_neighbors_ = integer("submap/neighbors", 25);
    registration_.max_correspondence = number("icp/max_correspondence", 150.0);
    registration_.iterations = integer("icp/iterations", 100);
    registration_.max_fitness = number("icp/max_fitness", 0.3);
    const double prior_rotation = number("noise/prior_rotation_sigma", 1e-6);
    const double prior_translation = number("noise/prior_translation_sigma", 1e-6);
    const double odom_rotation = number("noise/odom_rotation_sigma", 0.001);
    const double odom_translation = number("noise/odom_translation_sigma", 0.01);
    const double loop_rotation = number("noise/loop_rotation_sigma", std::sqrt(0.5));
    const double loop_translation = number("noise/loop_translation_sigma", std::sqrt(0.5));
    const double cauchy = number("noise/cauchy", 1.0);
    map_voxel_ = number("output/map_voxel", 0.3);
    map_period_ = number("output/map_period", 10.0);
    const auto output_root = string("output/directory", "/tmp/fastlio_loop_results");
    if (translation_ <= 0 || rotation_ <= 0 || cloud_voxel_ <= 0 || map_voxel_ <= 0 ||
        sync_buffer_ < 1 || max_queue_ < 1 || min_points_ < 1 || sc_.PC_NUM_RING < 1 ||
        sc_.PC_NUM_SECTOR < 4 || sc_.PC_MAX_RADIUS <= 0 || sc_.NUM_EXCLUDE_RECENT < 1 ||
        sc_.NUM_CANDIDATES_FROM_TREE < 1 || min_interval_ < 0 || loop_interval_ < 0 ||
        submap_neighbors_ < 0 || !std::isfinite(detection_period_) || detection_period_ <= 0 || map_period_ <= 0 ||
        registration_.max_correspondence <= 0 || registration_.iterations < 1 ||
        registration_.max_fitness <= 0 ||
        prior_rotation <= 0 || prior_translation <= 0 || odom_rotation <= 0 ||
        odom_translation <= 0 || loop_rotation <= 0 || loop_translation <= 0 || cauchy <= 0)
      throw std::runtime_error("Invalid loop_fusion configuration");
    prior_noise_ = noise(prior_rotation, prior_translation);
    odom_noise_ = noise(odom_rotation, odom_translation);
    loop_noise_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(cauchy), noise(loop_rotation, loop_translation));
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    isam_.reset(new gtsam::ISAM2(params));
    const boost::filesystem::path root(output_root);
    boost::filesystem::create_directories(root);
    output_ = (root / boost::filesystem::unique_path("run-%%%%-%%%%-%%%%")).string();
    boost::filesystem::create_directory(output_);
    std::ofstream parameters(output_ + "/parameters.yaml");
    parameters << snapshot.str();
    loops_.open(output_ + "/loops.csv");
    if (!parameters || !loops_) throw std::runtime_error("Cannot write result directory");
    loops_ << "history_id,current_id,history_stamp,current_stamp,sc_distance,sc_yaw_rad,"
              "icp_converged,fitness_m2,rmse_m,accepted,reason\n";
    loops_ << std::setprecision(17);
    path_pub_ = node_.advertise<nav_msgs::Path>("/loop/path", 1, true);
    raw_path_pub_ = node_.advertise<nav_msgs::Path>("/loop/path_raw", 1, true);
    map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/loop/map", 1, true);
    raw_map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/loop/map_raw", 1, true);
    edge_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/loop/edges", 1, true);
    source_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/loop/source", 1, true);
    target_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/loop/target", 1, true);
    aligned_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/loop/aligned", 1, true);
    status_pub_ = node_.advertise<std_msgs::String>("/loop/status", 1, true);
    corrected_odom_pub_ = node_.advertise<nav_msgs::Odometry>("/loop/odometry", 100);
    corrected_highrate_pub_ = node_.advertise<nav_msgs::Odometry>("/loop/odometry_highrate", 1000);
    correction_pub_ = node_.advertise<geometry_msgs::PoseStamped>("/loop/correction", 1, true);
    highrate_sub_ = node_.subscribe(highrate_topic, 1000, &LoopClosure::highrateOdometry, this);
    odom_sub_ = node_.subscribe(odom_topic, sync_buffer_, &LoopClosure::odometry, this);
    cloud_sub_ = node_.subscribe(cloud_topic, sync_buffer_, &LoopClosure::cloud, this);
    save_service_ = private_.advertiseService("save", &LoopClosure::save, this);
    map_worker_ = std::thread(&LoopClosure::mapWork, this);
    worker_ = std::thread(&LoopClosure::work, this);
    ROS_INFO_STREAM("loop_fusion ready; results: " << output_);
  }
  ~LoopClosure() {
    { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; }
    condition_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

 private:
  static gtsam::SharedNoiseModel noise(double rotation, double translation) {
    gtsam::Vector6 sigmas;
    sigmas << rotation, rotation, rotation, translation, translation, translation;
    return gtsam::noiseModel::Diagonal::Sigmas(sigmas);  // rad then m, standard deviations
  }
  void odometry(const nav_msgs::OdometryConstPtr& msg) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      odometry_[msg->header.stamp.toNSec()] = msg;
      match();
    }
    publishCorrected(msg, corrected_odom_pub_);
  }
  void highrateOdometry(const nav_msgs::OdometryConstPtr& msg) {
    publishCorrected(msg, corrected_highrate_pub_);
  }
  void cloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    clouds_[msg->header.stamp.toNSec()] = msg;
    match();
  }
  void match() {
    for (auto it = clouds_.begin(); it != clouds_.end();) {
      auto odom = odometry_.find(it->first);
      if (odom == odometry_.end()) { ++it; continue; }
      if (tasks_.size() < static_cast<size_t>(max_queue_)) {
        tasks_.push_back({odom->second, it->second, nullptr}); ++paired_;
      } else { ++dropped_pairs_; ROS_WARN_THROTTLE(5, "loop_fusion input queue full; reduce bag rate"); }
      odometry_.erase(odom); it = clouds_.erase(it);
    }
    // Unmatched messages wait for their exact partner. Bounded caches eventually
    // discard oldest unpaired messages, never substituting a newer pose.
    while (odometry_.size() > static_cast<size_t>(sync_buffer_)) { odometry_.erase(odometry_.begin()); ++dropped_odom_; }
    while (clouds_.size() > static_cast<size_t>(sync_buffer_)) { clouds_.erase(clouds_.begin()); ++dropped_cloud_; }
    condition_.notify_one();
  }
  bool save(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    auto promise = std::make_shared<std::promise<std::string>>();
    auto future = promise->get_future();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) { response.success = false; response.message = "backend stopping"; return true; }
      tasks_.push_back({nullptr, nullptr, promise});
    }
    condition_.notify_one();
    // A queue barrier: all matched input queued BEFORE this call is processed
    // before export. Leave the bag and frontend idle before requesting final save.
    try { response.message = future.get(); response.success = true; }
    catch (const std::exception& e) { response.success = false; response.message = e.what(); }
    return true;
  }
  void work() {
    for (;;) {
      Task task;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [&] { return stopping_ || !tasks_.empty(); });
        if (tasks_.empty() && stopping_) break;
        task = std::move(tasks_.front()); tasks_.pop_front();
      }
      if (task.save) {
        try { exportResults(); task.save->set_value(output_); }
        catch (...) { task.save->set_exception(std::current_exception()); }
        continue;
      }
      try { process(task); }
      catch (const std::exception& e) {
        ROS_FATAL_STREAM("loop_fusion processing failed: " << e.what());
        { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true;
          for (auto& pending : tasks_) if (pending.save) pending.save->set_exception(std::current_exception());
          tasks_.clear(); }
        ros::shutdown(); break;
      }
    }
    stopMapWorker();
    try { if (!keyframes_.empty()) exportResults(); }
    catch (const std::exception& e) { ROS_ERROR_STREAM("Final loop export failed: " << e.what()); }
  }
  void process(const Task& task) {
    if (task.odom->header.frame_id != frame_ || task.cloud->header.frame_id != body_frame_ ||
        task.odom->child_frame_id != body_frame_) {
      ++invalid_inputs_; ROS_WARN_THROTTLE(5, "loop_fusion input frame mismatch"); return;
    }
    if (!last_stamp_.isZero() && task.odom->header.stamp <= last_stamp_) {
      ++invalid_inputs_; ROS_WARN_THROTTLE(5, "loop_fusion requires increasing scan timestamps"); return;
    }
    last_stamp_ = task.odom->header.stamp;
    const auto& p = task.odom->pose.pose;
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    Eigen::Vector3d t(p.position.x, p.position.y, p.position.z);
    if (!q.coeffs().allFinite() || q.norm() < 1e-6 || !t.allFinite()) { ++invalid_inputs_; return; }
    q.normalize();
    const gtsam::Pose3 raw(gtsam::Rot3(q.toRotationMatrix()), t);
    if (have_previous_raw_) {
      const auto delta = previous_raw_.between(raw);
      translation_accumulated_ += delta.translation().norm();
      rotation_accumulated_ += Eigen::AngleAxisd(delta.rotation().matrix()).angle();
    }
    previous_raw_ = raw; have_previous_raw_ = true;
    if (!keyframes_.empty() && translation_accumulated_ < translation_ &&
        rotation_accumulated_ < rotation_) return;
    Cloud::Ptr input(new Cloud);
    pcl::fromROSMsg(*task.cloud, *input);
    Cloud::Ptr finite(new Cloud);
    for (const auto& point : *input) if (pcl::isFinite(point)) finite->push_back(point);
    auto body = downsample(finite, cloud_voxel_);
    if (body->size() < static_cast<size_t>(min_points_)) { ++invalid_inputs_; return; }
    translation_accumulated_ = rotation_accumulated_ = 0;
    const size_t i = keyframes_.size();
    const gtsam::Pose3 predicted = i == 0 ? raw :
        keyframes_.back().optimized.compose(keyframes_.back().raw.between(raw));
    // Remove only world yaw to level the descriptor. Geometry retains body coords.
    Eigen::Matrix3d level = Eigen::Matrix3d::Identity();
    if (gravity_align_) level = Eigen::AngleAxisd(-raw.rotation().yaw(), Eigen::Vector3d::UnitZ()).toRotationMatrix() * raw.rotation().matrix();
    keyframes_.push_back({i, task.odom->header.stamp, raw, predicted, body, level});
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    values.insert(i, predicted);
    if (i == 0) factors.add(gtsam::PriorFactor<gtsam::Pose3>(0, raw, prior_noise_));
    else factors.add(gtsam::BetweenFactor<gtsam::Pose3>(i - 1, i, keyframes_[i - 1].raw.between(raw), odom_noise_));
    isam_->update(factors, values);
    keyframes_.back().optimized = isam_->calculateEstimate<gtsam::Pose3>(i);
    if (i == 0) updateCorrection();
    Eigen::Matrix4d leveling = Eigen::Matrix4d::Identity();
    leveling.topLeftCorner<3, 3>() = level;
    auto descriptor_cloud = transform(body, leveling);
    sc_.makeAndSaveScancontextAndKeys(*descriptor_cloud);
    const bool query_due = last_query_stamp_.isZero() ||
        (keyframes_[i].stamp - last_query_stamp_).toSec() >= detection_period_;
    std::pair<int, float> candidate{-1, 0.0f};
    if (query_due) {
      refreshPoses();
      if (!edges_.empty()) updateCorrection();
      size_t history_limit = 0;
      while (history_limit < i && (keyframes_[i].stamp - keyframes_[history_limit].stamp).toSec() >= min_interval_) ++history_limit;
      candidate = sc_.detectLoopClosureID(history_limit);
      last_query_stamp_ = keyframes_[i].stamp;
    }
    bool loop_added = false;
    RegistrationResult result;
    if (!query_due) result.reason = "query_interval";
    else if (candidate.first < 0) result.reason = sc_.last_candidate < 0 ? "no_eligible_match" : "sc_distance";
    else if ((keyframes_[i].stamp - last_loop_stamp_).toSec() < loop_interval_) result.reason = "loop_interval";
    else if (edges_.count({static_cast<size_t>(candidate.first), i})) result.reason = "duplicate";
    else {
      const size_t j = static_cast<size_t>(candidate.first);
      Cloud::Ptr target(new Cloud);
      const size_t begin = j > static_cast<size_t>(submap_neighbors_) ? j - submap_neighbors_ : 0;
      const size_t end = std::min(i, j + submap_neighbors_ + 1);
      const gtsam::Pose3 reference = keyframes_[j].optimized;
      for (size_t k = begin; k < end; ++k) {
        if (i - k < static_cast<size_t>(sc_.NUM_EXCLUDE_RECENT) ||
            (keyframes_[i].stamp - keyframes_[k].stamp).toSec() < min_interval_) continue;
        *target += *transform(keyframes_[k].cloud, reference.between(keyframes_[k].optimized).matrix());
      }
      target = downsample(target, cloud_voxel_);
      // Virtual relative registration: colocate body origins rather than feeding
      // accumulated odometry translation into the ICP search. Work directly in
      // history-body coordinates, so the result is the BetweenFactor(j, i) measurement.
      Eigen::Matrix4d initial = Eigen::Matrix4d::Identity();
      if (gravity_align_)
        initial.topLeftCorner<3, 3>() = keyframes_[j].level_from_body.transpose() * level;
      result = registerClouds(body, target, initial, registration_);
      if (result.accepted) {
        gtsam::NonlinearFactorGraph loop;
        loop.add(gtsam::BetweenFactor<gtsam::Pose3>(j, i, gtsam::Pose3(result.T_target_source), loop_noise_));
        isam_->update(loop, gtsam::Values());
        for (int n = 0; n < 3; ++n) isam_->update();
        refreshPoses(); updateCorrection();
        edges_.insert({j, i}); last_loop_stamp_ = keyframes_[i].stamp; loop_added = true;
        ROS_INFO_STREAM("Loop " << j << " -> " << i << ": SC=" << sc_.last_distance << ", fitness=" << result.fitness << " m^2, RMSE=" << result.rmse << " m");
      } else ROS_INFO_STREAM("Rejected loop " << j << " -> " << i << ": " << result.reason << ", fitness=" << result.fitness << " m^2, RMSE=" << result.rmse << " m");
      // All debug clouds share the current optimized world reference of j.
      const Eigen::Matrix4d world = keyframes_[j].optimized.matrix();
      publishCloud(source_pub_, transform(body, world * initial), keyframes_[i].stamp);
      publishCloud(target_pub_, transform(target, world), keyframes_[i].stamp);
      publishCloud(aligned_pub_, transform(body, world * result.T_target_source), keyframes_[i].stamp);
    }
    if (query_due) {
      const int j = sc_.last_candidate;
      loops_ << j << ',' << i << ',' << (j >= 0 ? keyframes_[j].stamp.toSec() : 0) << ',' << keyframes_[i].stamp.toSec() << ','
             << sc_.last_distance << ',' << candidate.second << ',' << result.converged << ','
             << result.fitness << ',' << result.rmse << ',' << result.accepted << ',' << result.reason << '\n';
      loops_.flush();
    }
    if (loop_added || (ros::WallTime::now() - last_path_).toSec() >= 1.0) {
      publishPaths(); publishEdges(); last_path_ = ros::WallTime::now();
    }
    if (loop_added || (ros::WallTime::now() - last_map_).toSec() >= map_period_) requestMap();
    publishStatus();
  }
  void updateCorrection() {
    const auto& anchor = keyframes_.back();
    const gtsam::Pose3 correction = anchor.optimized.compose(anchor.raw.inverse());
    {
      std::lock_guard<std::mutex> lock(correction_mutex_);
      correction_ = correction; correction_stamp_ = anchor.stamp;
    }
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = frame_; msg.header.stamp = anchor.stamp;
    const auto t = correction.translation(); const auto q = correction.rotation().toQuaternion();
    msg.pose.position.x = t.x(); msg.pose.position.y = t.y(); msg.pose.position.z = t.z();
    msg.pose.orientation.x = q.x(); msg.pose.orientation.y = q.y(); msg.pose.orientation.z = q.z(); msg.pose.orientation.w = q.w();
    correction_pub_.publish(msg);
  }
  void publishCorrected(const nav_msgs::OdometryConstPtr& input, const ros::Publisher& publisher) {
    if (input->header.frame_id != frame_ || input->child_frame_id != body_frame_) return;
    gtsam::Pose3 correction;
    {
      std::lock_guard<std::mutex> lock(correction_mutex_);
      if (input->header.stamp < correction_stamp_) return;
      correction = correction_;
    }
    const auto& p = input->pose.pose;
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    const Eigen::Vector3d t(p.position.x, p.position.y, p.position.z);
    if (!q.coeffs().allFinite() || q.norm() < 1e-6 || !t.allFinite()) return;
    const auto corrected = correction.compose(gtsam::Pose3(gtsam::Rot3(q.normalized().toRotationMatrix()), t));
    nav_msgs::Odometry output = *input;
    output.child_frame_id = corrected_body_frame_;
    const auto position = corrected.translation(); const auto rotation = corrected.rotation().toQuaternion();
    output.pose.pose.position.x = position.x(); output.pose.pose.position.y = position.y(); output.pose.pose.position.z = position.z();
    output.pose.pose.orientation.x = rotation.x(); output.pose.pose.orientation.y = rotation.y();
    output.pose.pose.orientation.z = rotation.z(); output.pose.pose.orientation.w = rotation.w();
    Eigen::Matrix<double, 6, 6> axes = Eigen::Matrix<double, 6, 6>::Zero();
    axes.topLeftCorner<3, 3>() = axes.bottomRightCorner<3, 3>() = correction.rotation().matrix();
    const Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance(input->pose.covariance.data());
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> output_covariance(output.pose.covariance.data());
    output_covariance = axes * covariance * axes.transpose();
    // Twist remains in the same body axes. Global correction uncertainty is not
    // available here; this covariance only transports the frontend covariance.
    publisher.publish(output);
  }
  void refreshPoses() {
    const auto estimate = isam_->calculateEstimate();
    bool history_changed = false;
    for (auto& keyframe : keyframes_) {
      const auto updated = estimate.at<gtsam::Pose3>(keyframe.id);
      if (keyframe.id + 1 < keyframes_.size() &&
          (updated.matrix() - keyframe.optimized.matrix()).norm() > 1e-8) history_changed = true;
      keyframe.optimized = updated;
    }
    if (history_changed) ++map_revision_;
  }
  geometry_msgs::PoseStamped pose(const Keyframe& keyframe, bool optimized) const {
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = frame_; msg.header.stamp = keyframe.stamp;
    const auto& T = optimized ? keyframe.optimized : keyframe.raw;
    const auto t = T.translation(); const auto q = T.rotation().toQuaternion();
    msg.pose.position.x = t.x(); msg.pose.position.y = t.y(); msg.pose.position.z = t.z();
    msg.pose.orientation.x = q.x(); msg.pose.orientation.y = q.y(); msg.pose.orientation.z = q.z(); msg.pose.orientation.w = q.w();
    return msg;
  }
  void publishPaths() {
    nav_msgs::Path optimized, raw;
    optimized.header.frame_id = raw.header.frame_id = frame_;
    optimized.header.stamp = raw.header.stamp = keyframes_.back().stamp;
    for (const auto& keyframe : keyframes_) {
      optimized.poses.push_back(pose(keyframe, true)); raw.poses.push_back(pose(keyframe, false));
    }
    path_pub_.publish(optimized); raw_path_pub_.publish(raw);
  }
  void publishEdges() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_; marker.header.stamp = keyframes_.back().stamp;
    marker.ns = "verified_loops"; marker.id = 0; marker.type = visualization_msgs::Marker::LINE_LIST;
    // An empty LINE_LIST with ADD causes an RViz error; clear the marker instead.
    marker.action = edges_.empty() ? visualization_msgs::Marker::DELETE : visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.12; marker.color.r = 1; marker.color.b = 1; marker.color.a = 1;
    for (const auto& edge : edges_) {
      marker.points.push_back(pose(keyframes_[edge.first], true).pose.position);
      marker.points.push_back(pose(keyframes_[edge.second], true).pose.position);
    }
    visualization_msgs::MarkerArray array; array.markers.push_back(marker); edge_pub_.publish(array);
  }
  void publishCloud(const ros::Publisher& publisher, const Cloud::ConstPtr& cloud, const ros::Time& stamp) {
    sensor_msgs::PointCloud2 msg; pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = frame_; msg.header.stamp = stamp; publisher.publish(msg);
  }
  std::pair<Cloud::Ptr, Cloud::Ptr> maps(const Keyframes& snapshot) const {
    VoxelMap raw(map_voxel_), optimized(map_voxel_);
    for (const auto& keyframe : snapshot) {
      raw.add(*keyframe.cloud, keyframe.raw.matrix());
      optimized.add(*keyframe.cloud, keyframe.optimized.matrix());
    }
    return {raw.cloud(), optimized.cloud()};
  }
  void publishMapClouds(const std::pair<Cloud::Ptr, Cloud::Ptr>& clouds,
      const ros::Time& stamp, size_t revision) {
    std::lock_guard<std::mutex> lock(map_publish_mutex_);
    if (stamp < published_map_stamp_ || revision < published_map_revision_) return;
    publishCloud(raw_map_pub_, clouds.first, stamp);
    publishCloud(map_pub_, clouds.second, stamp);
    published_map_stamp_ = stamp; published_map_revision_ = revision;
  }
  void requestMap() {
    refreshPoses();
    if (!edges_.empty()) updateCorrection();
    std::unique_ptr<MapSnapshot> snapshot(new MapSnapshot{keyframes_, map_revision_});
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map_pending_ = std::move(snapshot);  // only the newest display snapshot is needed
    }
    last_map_ = ros::WallTime::now(); map_condition_.notify_one();
  }
  void stopMapWorker() {
    { std::lock_guard<std::mutex> lock(map_mutex_); map_stopping_ = true; map_pending_.reset(); }
    map_condition_.notify_one();
    if (map_worker_.joinable()) map_worker_.join();
  }
  void mapWork() {
    VoxelMap raw(map_voxel_), optimized(map_voxel_);
    size_t raw_count = 0, optimized_count = 0, revision = 0;
    for (;;) {
      std::unique_ptr<MapSnapshot> snapshot;
      {
        std::unique_lock<std::mutex> lock(map_mutex_);
        map_condition_.wait(lock, [&] { return map_stopping_ || map_pending_; });
        if (map_stopping_) break;
        snapshot = std::move(map_pending_);
      }
      try {
        if (snapshot->revision != revision) {
          optimized.clear(); optimized_count = 0; revision = snapshot->revision;
        }
        for (; raw_count < snapshot->keyframes.size(); ++raw_count) {
          const auto& k = snapshot->keyframes[raw_count]; raw.add(*k.cloud, k.raw.matrix());
        }
        for (; optimized_count < snapshot->keyframes.size(); ++optimized_count) {
          const auto& k = snapshot->keyframes[optimized_count]; optimized.add(*k.cloud, k.optimized.matrix());
        }
        publishMapClouds({raw.cloud(), optimized.cloud()}, snapshot->keyframes.back().stamp, revision);
      } catch (const std::exception& e) { ROS_ERROR_STREAM("Loop map publication failed: " << e.what()); }
    }
  }
  void publishStatus() {
    std::ostringstream text;
    text << "{\"keyframes\":" << keyframes_.size() << ",\"accepted_loops\":" << edges_.size();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      text << ",\"paired\":" << paired_ << ",\"pending\":" << tasks_.size()
           << ",\"unmatched_odom\":" << odometry_.size() << ",\"unmatched_cloud\":" << clouds_.size()
           << ",\"dropped_odom\":" << dropped_odom_ << ",\"dropped_cloud\":" << dropped_cloud_
           << ",\"dropped_pairs\":" << dropped_pairs_;
    }
    text << ",\"invalid_inputs\":" << invalid_inputs_ << ",\"output\":" << std::quoted(output_) << '}';
    std_msgs::String msg; msg.data = text.str(); status_pub_.publish(msg);
    std::ofstream status(output_ + "/status.json"); status << text.str() << '\n';
  }
  void exportResults() {
    if (keyframes_.empty()) throw std::runtime_error("No keyframes to save");
    refreshPoses();
    if (!edges_.empty()) updateCorrection();
    std::ofstream times(output_ + "/keyframe_times.txt"), raw(output_ + "/trajectory_raw.tum"), optimized(output_ + "/trajectory_optimized.tum");
    times << std::fixed << std::setprecision(9);
    raw << std::fixed << std::setprecision(9); optimized << std::fixed << std::setprecision(9);
    for (const auto& keyframe : keyframes_) {
      times << keyframe.id << ' ' << keyframe.stamp.toSec() << '\n';
      for (bool use_optimized : {false, true}) {
        const auto p = pose(keyframe, use_optimized).pose;
        auto& out = use_optimized ? optimized : raw;
        out << keyframe.stamp.toSec() << ' ' << p.position.x << ' ' << p.position.y << ' ' << p.position.z << ' '
            << p.orientation.x << ' ' << p.orientation.y << ' ' << p.orientation.z << ' ' << p.orientation.w << '\n';
      }
    }
    const auto clouds = maps(keyframes_);
    if (!times || !raw || !optimized ||
        pcl::io::savePCDFileBinary(output_ + "/map_raw.pcd", *clouds.first) != 0 ||
        pcl::io::savePCDFileBinary(output_ + "/map_optimized.pcd", *clouds.second) != 0)
      throw std::runtime_error("Failed to save loop results");
    publishMapClouds(clouds, keyframes_.back().stamp, map_revision_);
    publishPaths(); publishEdges();
    publishStatus(); loops_.flush();
    ROS_INFO_STREAM("Saved " << keyframes_.size() << " keyframes and " << edges_.size() << " loops to " << output_);
  }

  ros::NodeHandle node_, private_;
  ros::Subscriber odom_sub_, cloud_sub_, highrate_sub_;
  ros::Publisher path_pub_, raw_path_pub_, map_pub_, raw_map_pub_, edge_pub_, source_pub_, target_pub_, aligned_pub_, status_pub_;
  ros::Publisher corrected_odom_pub_, corrected_highrate_pub_, correction_pub_;
  ros::ServiceServer save_service_;
  std::mutex mutex_; std::condition_variable condition_; std::thread worker_;
  bool stopping_ = false, gravity_align_ = false;
  std::mutex map_mutex_, map_publish_mutex_, correction_mutex_;
  std::condition_variable map_condition_; std::thread map_worker_;
  std::unique_ptr<MapSnapshot> map_pending_;
  bool map_stopping_ = false, have_previous_raw_ = false;
  size_t map_revision_ = 0, published_map_revision_ = 0;
  gtsam::Pose3 previous_raw_, correction_;
  ros::Time correction_stamp_, published_map_stamp_, last_query_stamp_;
  double translation_accumulated_ = 0, rotation_accumulated_ = 0;
  std::map<uint64_t, nav_msgs::OdometryConstPtr> odometry_;
  std::map<uint64_t, sensor_msgs::PointCloud2ConstPtr> clouds_;
  std::deque<Task> tasks_;
  Keyframes keyframes_;
  std::set<std::pair<size_t, size_t>> edges_;
  SCManager sc_; RegistrationConfig registration_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::SharedNoiseModel prior_noise_, odom_noise_, loop_noise_;
  std::ofstream loops_;
  std::string frame_, body_frame_, corrected_body_frame_, output_;
  ros::Time last_stamp_, last_loop_stamp_; ros::WallTime last_map_, last_path_;
  double translation_, rotation_, cloud_voxel_, min_interval_, loop_interval_, detection_period_, map_voxel_, map_period_;
  int min_points_, sync_buffer_, max_queue_, submap_neighbors_;
  size_t paired_ = 0, dropped_odom_ = 0, dropped_cloud_ = 0, dropped_pairs_ = 0, invalid_inputs_ = 0;
};
}  // namespace loop_fusion

int main(int argc, char** argv) {
  ros::init(argc, argv, "fastlio_loop");
  try {
    loop_fusion::LoopClosure loop;
    ros::AsyncSpinner spinner(2); spinner.start(); ros::waitForShutdown(); spinner.stop();
  } catch (const std::exception& e) { ROS_FATAL_STREAM(e.what()); return 1; }
  return 0;
}

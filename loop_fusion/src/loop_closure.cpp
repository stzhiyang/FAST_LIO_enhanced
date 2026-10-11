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
#include <csignal>
#include <ctime>
#include <deque>
#include <fstream>
#include <future>
#include <iomanip>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <thread>

namespace loop_fusion {
namespace {
volatile std::sig_atomic_t exit_requested = 0;
void signalHandler(int) { exit_requested = 1; }
}  // namespace

struct Keyframe {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  size_t id;
  ros::Time stamp;
  gtsam::Pose3 raw, optimized;
  Cloud::Ptr cloud;
  Eigen::Matrix3d level_from_body;
};
using Keyframes = std::vector<Keyframe, Eigen::aligned_allocator<Keyframe>>;
struct MapSnapshot {
  Keyframes keyframes;
  size_t revision;
  bool export_files = false;
  std::shared_ptr<std::promise<std::string>> save;
};
struct Task {
  nav_msgs::OdometryConstPtr odom;
  sensor_msgs::PointCloud2ConstPtr cloud;
  std::shared_ptr<std::promise<std::string>> save;
};

class LoopClosure {
 public:
  LoopClosure() : private_("~") {
    auto number = [&](const std::string& name, double value) {
      double result; private_.param(name, result, value);
      return result;
    };
    auto string = [&](const std::string& name, const std::string& value) {
      std::string result; private_.param(name, result, value);
      return result;
    };
    frame_ = "camera_init";
    body_frame_ = "body";
    const std::string odom_topic = "/Odometry";
    const std::string cloud_topic = "/cloud_registered_body";
    const std::string highrate_topic = "/Odometry_highrate";
    corrected_body_frame_ = "body_loop";
    // Keep the SC-PGO tuning surface; other settings retain their working values.
    translation_ = number("keyframe/translation", 0.5);
    rotation_ = number("keyframe/rotation_deg", 10.0) * M_PI / 180.0;
    cloud_voxel_ = 0.4;
    min_points_ = 100;
    sync_buffer_ = 200;
    max_queue_ = 3000;
    sc_.PC_NUM_RING = 20;
    sc_.PC_NUM_SECTOR = 60;
    sc_.PC_MAX_RADIUS = number("sc/max_radius", 80.0);
    sc_.PC_UNIT_SECTORANGLE = 360.0 / sc_.PC_NUM_SECTOR;
    sc_.PC_UNIT_RINGGAP = sc_.PC_MAX_RADIUS / sc_.PC_NUM_RING;
    sc_.LIDAR_HEIGHT = 2.0;
    sc_.SC_DIST_THRES = number("sc/distance_threshold", 0.3);
    sc_.NUM_EXCLUDE_RECENT = 30;
    sc_.NUM_CANDIDATES_FROM_TREE = 3;
    gravity_align_ = false;
    detection_period_ = 1.0;
    min_interval_ = 0.0;
    loop_interval_ = 0.0;
    submap_neighbors_ = 25;
    registration_.max_correspondence = 150.0;
    registration_.iterations = 100;
    registration_.max_fitness = 0.3;
    const double prior_rotation = 1e-6;
    const double prior_translation = 1e-6;
    const double odom_rotation = 0.001;
    const double odom_translation = 0.01;
    const double loop_rotation = std::sqrt(0.5);
    const double loop_translation = std::sqrt(0.5);
    const double cauchy = 1.0;
    map_voxel_ = number("output/map_voxel", 0.3);
    map_period_ = 10.0;
    auto output_root = string("output/directory", "");
    if (output_root.empty()) output_root = DEFAULT_RESULTS_DIR;
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
    boost::filesystem::create_directories(output_root);
    const auto root = boost::filesystem::canonical(output_root);
    const std::time_t start_time = std::time(nullptr);
    std::tm local_start{};
    localtime_r(&start_time, &local_start);
    std::ostringstream run_name;
    run_name << "run-" << std::put_time(&local_start, "%Y%m%d-%H%M%S");
    output_ = (root / run_name.str()).string();
    for (size_t suffix = 2; !boost::filesystem::create_directory(output_); ++suffix)
      output_ = (root / (run_name.str() + "_" + std::to_string(suffix))).string();
    path_pub_ = node_.advertise<nav_msgs::Path>("/loop/path", 1, true);
    map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/loop/map", 1, true);
    edge_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/loop/edges", 1, true);
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
  ~LoopClosure() { stop(); }
  void stop() {
    size_t discarded = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      for (auto& pending : tasks_) {
        if (pending.save)
          pending.save->set_exception(std::make_exception_ptr(std::runtime_error("backend stopping")));
        else ++discarded;
      }
      tasks_.clear(); odometry_.clear(); clouds_.clear();
    }
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      discard_display_maps_ = true; map_pending_.reset();
    }
    if (discarded)
      ROS_INFO_STREAM("Stopping: discarded " << discarded << " pending scan pairs; saving processed keyframes");
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
      if (stopping_) return;
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
    if (stopping_) return;
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
        if (stopping_) break;
        task = std::move(tasks_.front()); tasks_.pop_front();
      }
      if (task.save) {
        try { exportResults(task.save); }
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
    try {
      if (!keyframes_.empty()) {
        ROS_INFO_STREAM("Saving final map from " << keyframes_.size() << " processed keyframes to " << output_);
        exportResults();
      }
    }
    catch (const std::exception& e) { ROS_ERROR_STREAM("Final loop export failed: " << e.what()); }
    stopMapWorker();
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
    if (history_changed) {
      ++map_revision_;
      std::lock_guard<std::mutex> lock(map_mutex_);
      latest_requested_revision_ = map_revision_;
    }
  }
  geometry_msgs::PoseStamped pose(const Keyframe& keyframe) const {
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = frame_; msg.header.stamp = keyframe.stamp;
    const auto& T = keyframe.optimized;
    const auto t = T.translation(); const auto q = T.rotation().toQuaternion();
    msg.pose.position.x = t.x(); msg.pose.position.y = t.y(); msg.pose.position.z = t.z();
    msg.pose.orientation.x = q.x(); msg.pose.orientation.y = q.y(); msg.pose.orientation.z = q.z(); msg.pose.orientation.w = q.w();
    return msg;
  }
  void publishPaths() {
    nav_msgs::Path optimized;
    optimized.header.frame_id = frame_;
    optimized.header.stamp = keyframes_.back().stamp;
    for (const auto& keyframe : keyframes_) {
      optimized.poses.push_back(pose(keyframe));
    }
    path_pub_.publish(optimized);
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
      marker.points.push_back(pose(keyframes_[edge.first]).pose.position);
      marker.points.push_back(pose(keyframes_[edge.second]).pose.position);
    }
    visualization_msgs::MarkerArray array; array.markers.push_back(marker); edge_pub_.publish(array);
  }
  bool publishMapCloud(const Cloud::ConstPtr& cloud, const MapSnapshot& snapshot) {
    sensor_msgs::PointCloud2 msg; pcl::toROSMsg(*cloud, msg);
    const auto stamp = snapshot.keyframes.back().stamp;
    msg.header.frame_id = frame_; msg.header.stamp = stamp;
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (snapshot.revision < latest_requested_revision_ ||
        (snapshot.revision == latest_requested_revision_ && snapshot.keyframes.size() < latest_requested_count_) ||
        stamp < published_map_stamp_ || snapshot.revision < published_map_revision_) return false;
    map_pub_.publish(msg);
    published_map_stamp_ = stamp; published_map_revision_ = snapshot.revision;
    return true;
  }
  void enqueueMap(std::unique_ptr<MapSnapshot> snapshot) {
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      latest_requested_revision_ = std::max(latest_requested_revision_, snapshot->revision);
      latest_requested_count_ = snapshot->keyframes.size();
      if (snapshot->export_files) {
        if (map_saves_.size() >= 2) throw std::runtime_error("Map export queue full");
        map_saves_.push_back(std::move(snapshot));
      } else if (!discard_display_maps_) map_pending_ = std::move(snapshot);
    }
    map_condition_.notify_one();
  }
  void requestMap() {
    refreshPoses();
    if (!edges_.empty()) updateCorrection();
    enqueueMap(std::unique_ptr<MapSnapshot>(new MapSnapshot{keyframes_, map_revision_}));
    last_map_ = ros::WallTime::now();
  }
  void stopMapWorker() {
    { std::lock_guard<std::mutex> lock(map_mutex_); map_stopping_ = true; map_pending_.reset(); }
    map_condition_.notify_one();
    if (map_worker_.joinable()) map_worker_.join();
  }
  void saveMap(const Cloud::ConstPtr& cloud) {
    if (pcl::io::savePCDFileBinary(output_ + "/map_optimized.pcd", *cloud) != 0)
      throw std::runtime_error("Cannot save map_optimized.pcd");
  }
  void mapWork() {
    VoxelMap optimized(map_voxel_);
    size_t count = 0, revision = std::numeric_limits<size_t>::max();
    for (;;) {
      std::unique_ptr<MapSnapshot> snapshot;
      {
        std::unique_lock<std::mutex> lock(map_mutex_);
        map_condition_.wait(lock, [&] { return map_stopping_ || map_pending_ || !map_saves_.empty(); });
        if (!map_saves_.empty()) {
          snapshot = std::move(map_saves_.front()); map_saves_.pop_front();
        } else {
          if (map_stopping_) break;
          snapshot = std::move(map_pending_);
        }
      }
      try {
        if (snapshot->revision != revision || snapshot->keyframes.size() < count) {
          optimized.clear(); count = 0; revision = snapshot->revision;
        }
        bool cancelled = false;
        for (; count < snapshot->keyframes.size(); ++count) {
          if (!snapshot->export_files) {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (discard_display_maps_ || map_stopping_ || snapshot->revision < latest_requested_revision_ || !map_saves_.empty()) {
              cancelled = true; break;
            }
          }
          const auto& k = snapshot->keyframes[count];
          optimized.add(*k.cloud, k.optimized.matrix());
        }
        if (cancelled) continue;
        const auto cloud = optimized.cloud();
        if (snapshot->export_files) saveMap(cloud);
        publishMapCloud(cloud, *snapshot);
        if (snapshot->save) snapshot->save->set_value(output_);
        if (snapshot->export_files)
          ROS_INFO_STREAM("Saved " << snapshot->keyframes.size() << " keyframes to " << output_);
      } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Loop map task failed: " << e.what());
        if (snapshot->save) snapshot->save->set_exception(std::current_exception());
        optimized.clear(); count = 0; revision = std::numeric_limits<size_t>::max();
      }
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
    text << ",\"invalid_inputs\":" << invalid_inputs_ << ",\"output\":" << std::quoted(output_);
    std_msgs::String msg; msg.data = text.str() + "}"; status_pub_.publish(msg);
  }
  void exportResults(std::shared_ptr<std::promise<std::string>> save = nullptr) {
    if (keyframes_.empty()) throw std::runtime_error("No keyframes to save");
    refreshPoses();
    if (!edges_.empty()) updateCorrection();
    std::unique_ptr<MapSnapshot> snapshot(new MapSnapshot{keyframes_, map_revision_});
    snapshot->export_files = true; snapshot->save = std::move(save);
    enqueueMap(std::move(snapshot));
    publishPaths(); publishEdges(); publishStatus();
  }

  ros::NodeHandle node_, private_;
  ros::Subscriber odom_sub_, cloud_sub_, highrate_sub_;
  ros::Publisher path_pub_, map_pub_, edge_pub_, status_pub_;
  ros::Publisher corrected_odom_pub_, corrected_highrate_pub_, correction_pub_;
  ros::ServiceServer save_service_;
  std::mutex mutex_; std::condition_variable condition_; std::thread worker_;
  bool stopping_ = false, gravity_align_ = false;
  std::mutex map_mutex_, correction_mutex_;
  std::condition_variable map_condition_; std::thread map_worker_;
  std::unique_ptr<MapSnapshot> map_pending_;
  std::deque<std::unique_ptr<MapSnapshot>> map_saves_;
  size_t latest_requested_revision_ = 0, latest_requested_count_ = 0;
  bool map_stopping_ = false, discard_display_maps_ = false, have_previous_raw_ = false;
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
  std::string frame_, body_frame_, corrected_body_frame_, output_;
  ros::Time last_stamp_, last_loop_stamp_; ros::WallTime last_map_, last_path_;
  double translation_, rotation_, cloud_voxel_, min_interval_, loop_interval_, detection_period_, map_voxel_, map_period_;
  int min_points_, sync_buffer_, max_queue_, submap_neighbors_;
  size_t paired_ = 0, dropped_odom_ = 0, dropped_cloud_ = 0, dropped_pairs_ = 0, invalid_inputs_ = 0;
};
}  // namespace loop_fusion

int main(int argc, char** argv) {
  ros::init(argc, argv, "fastlio_loop", ros::init_options::NoSigintHandler);
  std::signal(SIGINT, loop_fusion::signalHandler);
  try {
    loop_fusion::LoopClosure loop;
    ros::AsyncSpinner spinner(2); spinner.start();
    // Keep ROS alive during Ctrl+C export, even after the bag's /clock stops.
    ros::WallRate rate(100);
    while (ros::ok() && !loop_fusion::exit_requested) rate.sleep();
    loop.stop();
    spinner.stop();
  } catch (const std::exception& e) { ROS_FATAL_STREAM(e.what()); return 1; }
  return 0;
}

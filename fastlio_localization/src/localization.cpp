// SPDX-License-Identifier: GPL-2.0-only
// FAST_LIO_LOCALIZATION: HViktorTsoi and contributors.
// https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION
#include "fastlio_localization/localization.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <boost/bind/bind.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>

namespace fastlio_localization {
void Localization::loadParameters() {
  const Config defaults;
  private_.param("map_path", config_.map_path, defaults.map_path);
  private_.param("map_frame", config_.map_frame, defaults.map_frame);
  private_.param("localized_body_frame", config_.localized_body_frame, defaults.localized_body_frame);
  private_.param("map_voxel", config_.map_voxel, defaults.map_voxel);
  private_.param("scan_voxel", config_.scan_voxel, defaults.scan_voxel);
  private_.param("coarse_scale", config_.coarse_scale, defaults.coarse_scale);
  private_.param("correspondence_distance", config_.correspondence_distance, defaults.correspondence_distance);
  private_.param("frequency", config_.frequency, defaults.frequency);
  private_.param("min_fitness", config_.min_fitness, defaults.min_fitness);
  private_.param("max_rmse", config_.max_rmse, defaults.max_rmse);
  private_.param("submap_radius", config_.submap_radius, defaults.submap_radius);
  private_.param("initial_time_tolerance", config_.initial_time_tolerance, defaults.initial_time_tolerance);
  private_.param("min_points", config_.min_points, defaults.min_points);
  private_.param("max_iterations", config_.max_iterations, defaults.max_iterations);
  for (double value : {config_.map_voxel, config_.scan_voxel, config_.coarse_scale,
                      config_.correspondence_distance, config_.frequency, config_.min_fitness,
                      config_.max_rmse, config_.submap_radius, config_.initial_time_tolerance})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("numeric parameters must be positive and finite");
  if (config_.min_fitness > 1 || config_.coarse_scale < 1 || config_.min_points <= 0 || config_.max_iterations <= 0)
    throw std::invalid_argument("invalid ICP limits");
  const std::set<std::string> frames{config_.map_frame, config_.localized_body_frame, "camera_init", "body", "body_loop"};
  if (frames.size() != 5 || config_.map_frame.empty() || config_.localized_body_frame.empty())
    throw std::invalid_argument("localization frames must be distinct from frontend/loop frames");
}

Localization::Localization(const ros::NodeHandle& private_node) : private_(private_node) {
  loadParameters();
  matcher_.reset(new Matcher(config_));
  odom_pub_ = private_.advertise<nav_msgs::Odometry>("odometry", 100);
  high_pub_ = private_.advertise<nav_msgs::Odometry>("odometry_highrate", 1000);
  map_pub_ = private_.advertise<sensor_msgs::PointCloud2>("map", 1, true);
  scan_pub_ = private_.advertise<sensor_msgs::PointCloud2>("scan", 1);
  correction_pub_ = private_.advertise<geometry_msgs::TransformStamped>("map_to_odom", 1);
  status_pub_ = private_.advertise<diagnostic_msgs::DiagnosticArray>("status", 1, true);
  map_pub_.publish(matcher_->mapMessage());
  publishStatus();
  odom_sub_.subscribe(node_, "/Odometry", 100);
  cloud_sub_.subscribe(node_, "/cloud_registered_body", 10);
  sync_.reset(new message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>(odom_sub_, cloud_sub_, 100));
  using namespace boost::placeholders;
  sync_->registerCallback(boost::bind(&Localization::paired, this, _1, _2));
  odom_sub_.registerCallback(boost::bind(&Localization::publishOdometry, this, _1, odom_pub_, true));
  high_sub_ = node_.subscribe<nav_msgs::Odometry>("/Odometry_highrate", 1000,
      boost::bind(&Localization::publishOdometry, this, _1, high_pub_, false));
  initial_sub_ = node_.subscribe("/initialpose", 1, &Localization::initialPose, this);
  worker_ = std::thread(&Localization::run, this);
  ROS_INFO("C++ localization ready; waiting for synchronized scan and /initialpose");
}

Localization::~Localization() {
  { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; }
  condition_.notify_one();
  if (worker_.joinable()) worker_.join();
}

void Localization::paired(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::PointCloud2ConstPtr& cloud) {
  if (odom->header.frame_id != "camera_init" || odom->child_frame_id != "body" || cloud->header.frame_id != "body") {
    ROS_WARN_THROTTLE(5, "Localization expects camera_init/body odometry and body cloud");
    return;
  }
  try { poseMatrix(odom->pose.pose); }
  catch (const std::exception& e) { ROS_WARN_THROTTLE(5, "%s", e.what()); return; }
  std::lock_guard<std::mutex> lock(mutex_);
  if (latest_.odom && odom->header.stamp < latest_.odom->header.stamp) {
    // 前端回放时间回退时，局部坐标系可能已重建，旧修正与在途计算全部失效。
    ++generation_;
    initialized_ = have_seed_ = attempted_ = false;
    correction_stamp_ = ros::Time();
    fitness_ = rmse_ = 0;
    history_.clear();
    state_ = "UNINITIALIZED";
    reason_ = "input time moved backwards; provide /initialpose again";
    publishStatus();
  }
  latest_ = {odom, cloud};
  history_.push_back(latest_);
  if (history_.size() > 100) history_.pop_front();
}

void Localization::initialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
  try {
    if (msg->header.frame_id != config_.map_frame)
      throw std::invalid_argument("/initialpose must be expressed in " + config_.map_frame);
    const Pose initial = poseMatrix(msg->pose.pose);
    std::lock_guard<std::mutex> lock(mutex_);
    if (history_.empty()) throw std::invalid_argument("wait for synchronized odometry/cloud before /initialpose");
    Pair pair = latest_;
    if (!msg->header.stamp.isZero()) {
      pair = *std::min_element(history_.begin(), history_.end(), [&](const Pair& a, const Pair& b) {
        return std::abs((a.odom->header.stamp - msg->header.stamp).toSec()) <
               std::abs((b.odom->header.stamp - msg->header.stamp).toSec());
      });
      if (std::abs((pair.odom->header.stamp - msg->header.stamp).toSec()) > config_.initial_time_tolerance)
        throw std::invalid_argument("/initialpose timestamp has no nearby synchronized scan");
    }
    // 输入表示地图中的机器人位姿，必须乘对应局部位姿的逆，才能得到 map->odom。
    seed_ = initial * poseMatrix(pair.odom->pose.pose).inverse();
    initial_pair_ = pair;
    initialized_ = attempted_ = false;
    have_seed_ = immediate_ = true;
    correction_stamp_ = ros::Time();
    fitness_ = rmse_ = 0;
    ++generation_;
    state_ = "UNINITIALIZED"; reason_ = "initial pose accepted; awaiting ICP";
    publishStatus();
    condition_.notify_one();
  } catch (const std::exception& e) { ROS_WARN("Rejected initial pose: %s", e.what()); }
}

void Localization::run() {
  using Clock = std::chrono::steady_clock;
  auto next_match = Clock::now();
  while (ros::ok()) {
    Pair pair;
    Pose seed;
    uint64_t generation;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      // 使用墙上时间，bag 暂停后仍可初始化；仅初值立即唤醒，普通帧不会突破频率限制。
      condition_.wait_for(lock, std::chrono::milliseconds(100), [&] { return stopping_ || immediate_; });
      if (stopping_) return;
      const bool immediate = immediate_;
      immediate_ = false;
      if (!have_seed_ || !latest_.odom || (!immediate && Clock::now() < next_match)) continue;
      pair = attempted_ ? latest_ : initial_pair_;
      if (attempted_ && pair.odom->header.stamp == attempt_stamp_) continue;
      attempted_ = true;
      attempt_stamp_ = pair.odom->header.stamp;
      generation = generation_;
      seed = initialized_ ? correction_ : seed_;
    }
    next_match = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / config_.frequency));
    MatchResult result;
    try { result = matcher_->match(*pair.cloud, poseMatrix(pair.odom->pose.pose), seed); }
    catch (const std::exception& e) { result.reason = std::string("ICP failed: ") + e.what(); ROS_WARN("%s", result.reason.c_str()); }
    finishMatch(result, pair, generation);
  }
}

void Localization::finishMatch(const MatchResult& result, const Pair& pair, uint64_t generation) {
  std::lock_guard<std::mutex> lock(mutex_);
  // 重初始化不等待旧 ICP；结果提交时检查代数，旧计算不能恢复已撤销的修正。
  if (stopping_ || generation != generation_) return;
  fitness_ = result.fitness; rmse_ = result.rmse;
  state_ = result.accepted ? "MATCHED" : "MATCH_FAILED";
  reason_ = result.reason;
  if (result.accepted) {
    correction_ = result.correction;
    correction_stamp_ = pair.odom->header.stamp;
    initialized_ = true;
    correction_pub_.publish(transformMessage(correction_, correction_stamp_, "camera_init"));
    scan_pub_.publish(result.scan);
  }
  publishStatus();
}

geometry_msgs::TransformStamped Localization::transformMessage(const Pose& pose, const ros::Time& stamp,
                                                                const std::string& child) const {
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = stamp; msg.header.frame_id = config_.map_frame; msg.child_frame_id = child;
  const auto p = poseMessage(pose);
  msg.transform.translation.x = p.position.x; msg.transform.translation.y = p.position.y;
  msg.transform.translation.z = p.position.z; msg.transform.rotation = p.orientation;
  return msg;
}

void Localization::publishOdometry(const nav_msgs::OdometryConstPtr& msg, const ros::Publisher& publisher, bool broadcast) {
  if (msg->header.frame_id != "camera_init" || msg->child_frame_id != "body") return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || msg->header.stamp < correction_stamp_) return;
  try {
    const auto output = correctedOdometry(*msg, correction_, config_);
    publisher.publish(output);
    // 仅普通里程计发布独立 child TF，避免两路时间戳乱序，也不改变前端/回环的 TF 树。
    if (broadcast) broadcaster_.sendTransform(transformMessage(poseMatrix(output.pose.pose), msg->header.stamp,
                                                               config_.localized_body_frame));
  } catch (const std::exception& e) { ROS_WARN_THROTTLE(5, "%s", e.what()); }
}

void Localization::publishStatus() {
  diagnostic_msgs::DiagnosticStatus status;
  status.name = "localization"; status.hardware_id = "scan_to_map";
  status.level = state_ == "MATCHED" ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::WARN;
  status.message = state_;
  for (const auto& item : std::vector<std::pair<std::string, std::string>>{
      {"initialized", initialized_ ? "true" : "false"}, {"reason", reason_},
      {"fitness", std::to_string(fitness_)}, {"rmse", std::to_string(rmse_)},
      {"last_success_stamp", std::to_string(correction_stamp_.toSec())}, {"generation", std::to_string(generation_)}}) {
    diagnostic_msgs::KeyValue value; value.key = item.first; value.value = item.second;
    status.values.push_back(value);
  }
  diagnostic_msgs::DiagnosticArray message;
  message.header.stamp = ros::Time::now(); message.status.push_back(status);
  status_pub_.publish(message);
}
}  // namespace fastlio_localization

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization");
  fastlio_localization::Localization node;
  ros::spin();
  return 0;
}

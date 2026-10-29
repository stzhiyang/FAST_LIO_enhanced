// SPDX-License-Identifier: GPL-2.0-only
// FAST_LIO_LOCALIZATION: HViktorTsoi and contributors.
#pragma once

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

namespace fastlio_localization {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Pose = Eigen::Isometry3d;

struct Config {
  std::string map_path, map_frame = "map", localized_body_frame = "localization_body";
  double map_voxel = 0.4, scan_voxel = 0.1, coarse_scale = 5.0;
  double correspondence_distance = 1.0, frequency = 0.5, min_fitness = 0.95;
  double max_rmse = 0.5, submap_radius = 150.0, initial_time_tolerance = 1.0;
  int min_points = 30, max_iterations = 20;
};

Pose poseMatrix(const geometry_msgs::Pose& msg);
geometry_msgs::Pose poseMessage(const Pose& pose);
Cloud::Ptr readCloud(const sensor_msgs::PointCloud2& msg);
nav_msgs::Odometry correctedOdometry(const nav_msgs::Odometry& input, const Pose& correction,
                                     const Config& config);

struct MatchResult {
  bool accepted = false;
  Pose correction = Pose::Identity();
  double fitness = 0.0, rmse = 0.0;
  std::string reason;
  sensor_msgs::PointCloud2 scan;
};

// 地图只在构造时载入；配准只由一个后台线程调用。
class Matcher {
 public:
  explicit Matcher(const Config& config);
  MatchResult match(const sensor_msgs::PointCloud2& scan, const Pose& odom, const Pose& seed) const;
  sensor_msgs::PointCloud2 mapMessage() const;
 private:
  Config config_;
  Cloud::Ptr map_;
};

class Localization {
 public:
  explicit Localization(const ros::NodeHandle& private_node = ros::NodeHandle("~"));
  ~Localization();
 private:
  friend class LocalizationTestAccess;
  struct Pair {
    nav_msgs::OdometryConstPtr odom;
    sensor_msgs::PointCloud2ConstPtr cloud;
  };
  void loadParameters();
  void paired(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::PointCloud2ConstPtr& cloud);
  void initialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  void publishOdometry(const nav_msgs::OdometryConstPtr& msg, const ros::Publisher& publisher, bool broadcast);
  void run();
  void finishMatch(const MatchResult& result, const Pair& pair, uint64_t generation);
  void publishStatus();  // 调用者持有 mutex_，构造阶段除外。
  geometry_msgs::TransformStamped transformMessage(const Pose& pose, const ros::Time& stamp,
                                                   const std::string& child) const;
  ros::NodeHandle node_, private_;
  Config config_;
  std::unique_ptr<Matcher> matcher_;
  ros::Publisher odom_pub_, high_pub_, map_pub_, scan_pub_, correction_pub_, status_pub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  std::unique_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>> sync_;
  ros::Subscriber high_sub_, initial_sub_;
  tf2_ros::TransformBroadcaster broadcaster_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::thread worker_;
  bool stopping_ = false, immediate_ = false, have_seed_ = false, initialized_ = false;
  bool attempted_ = false;
  uint64_t generation_ = 0;
  ros::Time attempt_stamp_, correction_stamp_;
  Pair latest_, initial_pair_;
  std::deque<Pair> history_;
  Pose seed_ = Pose::Identity(), correction_ = Pose::Identity();
  std::string state_ = "UNINITIALIZED", reason_ = "waiting for synchronized scan and /initialpose";
  double fitness_ = 0.0, rmse_ = 0.0;
};
}  // namespace fastlio_localization

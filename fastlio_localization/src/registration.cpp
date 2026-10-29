// SPDX-License-Identifier: GPL-2.0-only
// FAST_LIO_LOCALIZATION: HViktorTsoi and contributors.
// https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION
#include "fastlio_localization/localization.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace fastlio_localization {
Pose poseMatrix(const geometry_msgs::Pose& msg) {
  Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  Eigen::Vector3d t(msg.position.x, msg.position.y, msg.position.z);
  if (!q.coeffs().allFinite() || !t.allFinite() || q.norm() < 1e-6)
    throw std::invalid_argument("invalid pose");
  Pose result = Pose::Identity();
  result.linear() = q.normalized().toRotationMatrix();
  result.translation() = t;
  return result;
}
geometry_msgs::Pose poseMessage(const Pose& pose) {
  geometry_msgs::Pose msg;
  msg.position.x = pose.translation().x(); msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();
  const Eigen::Quaterniond q(pose.linear());
  msg.orientation.x = q.x(); msg.orientation.y = q.y();
  msg.orientation.z = q.z(); msg.orientation.w = q.w();
  return msg;
}

Cloud::Ptr readCloud(const sensor_msgs::PointCloud2& msg) {
  // 显式读取 XYZ，兼容字段重排、行填充、float32/64 和大小端；不假设 PCL 的内存布局。
  std::array<sensor_msgs::PointField, 3> fields;
  const std::array<std::string, 3> names{{"x", "y", "z"}};
  for (size_t axis = 0; axis < 3; ++axis) {
    auto it = std::find_if(msg.fields.begin(), msg.fields.end(), [&](const sensor_msgs::PointField& f) {
      return f.name == names[axis];
    });
    if (it == msg.fields.end() || it->count != 1 ||
        (it->datatype != sensor_msgs::PointField::FLOAT32 && it->datatype != sensor_msgs::PointField::FLOAT64))
      throw std::invalid_argument("cloud requires scalar float32/64 x/y/z fields");
    const size_t size = it->datatype == sensor_msgs::PointField::FLOAT32 ? 4 : 8;
    if (static_cast<uint64_t>(it->offset) + size > msg.point_step)
      throw std::invalid_argument("XYZ field exceeds point_step");
    fields[axis] = *it;
  }
  if (static_cast<uint64_t>(msg.width) * msg.point_step > msg.row_step ||
      static_cast<uint64_t>(msg.height) * msg.row_step > msg.data.size())
    throw std::invalid_argument("invalid point cloud strides/data length");
  const uint16_t endian = 1;
  const bool host_big = *reinterpret_cast<const uint8_t*>(&endian) == 0;
  Cloud::Ptr cloud(new Cloud);
  cloud->reserve(static_cast<size_t>(msg.width) * msg.height);
  for (size_t row = 0; row < msg.height; ++row) {
    for (size_t col = 0; col < msg.width; ++col) {
      pcl::PointXYZ point;
      for (size_t axis = 0; axis < 3; ++axis) {
        const auto& field = fields[axis];
        const size_t size = field.datatype == sensor_msgs::PointField::FLOAT32 ? 4 : 8;
        std::array<uint8_t, 8> bytes{};
        std::memcpy(bytes.data(), msg.data.data() + row * msg.row_step + col * msg.point_step + field.offset, size);
        if (msg.is_bigendian != host_big) std::reverse(bytes.begin(), bytes.begin() + size);
        if (size == 4) std::memcpy(&point.data[axis], bytes.data(), 4);
        else { double value; std::memcpy(&value, bytes.data(), 8); point.data[axis] = static_cast<float>(value); }
      }
      if (point.getVector3fMap().allFinite()) cloud->push_back(point);
    }
  }
  cloud->is_dense = true;
  return cloud;
}

nav_msgs::Odometry correctedOdometry(const nav_msgs::Odometry& input, const Pose& correction,
                                     const Config& config) {
  nav_msgs::Odometry output = input;
  output.header.frame_id = config.map_frame;
  output.child_frame_id = config.localized_body_frame;
  output.pose.pose = poseMessage(correction * poseMatrix(input.pose.pose));
  // child 与实际 body 同轴，twist 保持不变。此处只传递前端协方差，不估计 ICP 的不确定度。
  Eigen::Matrix<double, 6, 6> axes = Eigen::Matrix<double, 6, 6>::Zero();
  axes.topLeftCorner<3, 3>() = axes.bottomRightCorner<3, 3>() = correction.linear();
  using Covariance = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
  Eigen::Map<const Covariance> covariance(input.pose.covariance.data());
  Eigen::Map<Covariance> out_covariance(output.pose.covariance.data());
  out_covariance = axes * covariance * axes.transpose();
  return output;
}

namespace {
Cloud::Ptr downsample(const Cloud::ConstPtr& cloud, double voxel) {
  Cloud::Ptr output(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(voxel, voxel, voxel);
  filter.filter(*output);
  return output;
}
}  // namespace

Matcher::Matcher(const Config& config) : config_(config) {
  pcl::PCLPointCloud2 raw;
  if (config.map_path.empty() || pcl::io::loadPCDFile(config.map_path, raw) < 0)
    throw std::runtime_error("cannot read map: " + config.map_path);
  sensor_msgs::PointCloud2 message;
  pcl_conversions::fromPCL(raw, message);
  map_ = downsample(readCloud(message), config.map_voxel);
  if (map_->size() < static_cast<size_t>(config.min_points))
    throw std::runtime_error("map is empty or too small");
}

sensor_msgs::PointCloud2 Matcher::mapMessage() const {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*map_, msg);
  msg.header.frame_id = config_.map_frame;
  return msg;
}

MatchResult Matcher::match(const sensor_msgs::PointCloud2& message, const Pose& odom, const Pose& seed) const {
  MatchResult result;
  auto scan = readCloud(message);
  const Pose predicted = seed * odom;
  Cloud::Ptr target(new Cloud);
  const double radius2 = config_.submap_radius * config_.submap_radius;
  // 球形子图不假定 body x 轴是雷达光轴，兼容前视与环视传感器。
  for (const auto& p : *map_)
    if ((p.getVector3fMap().cast<double>() - predicted.translation()).squaredNorm() <= radius2)
      target->push_back(p);
  if (std::min(scan->size(), target->size()) < static_cast<size_t>(config_.min_points))
    throw std::runtime_error("too few scan/submap points");

  Eigen::Matrix4f transform = predicted.matrix().cast<float>();
  Cloud::Ptr fine_source, fine_target;
  bool converged = false;
  for (double scale : {config_.coarse_scale, 1.0}) {
    auto source_down = downsample(scan, config_.scan_voxel * scale);
    auto target_down = downsample(target, config_.map_voxel * scale);
    if (std::min(source_down->size(), target_down->size()) < 3)
      throw std::runtime_error("too few voxelized points");
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source_down);
    icp.setInputTarget(target_down);
    icp.setMaxCorrespondenceDistance(config_.correspondence_distance * scale);
    icp.setMaximumIterations(config_.max_iterations);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-6);
    Cloud aligned;
    icp.align(aligned, transform);
    transform = icp.getFinalTransformation();
    converged = icp.hasConverged();
    fine_source = source_down; fine_target = target_down;
    if (!transform.allFinite()) throw std::runtime_error("non-finite ICP result");
  }

  // PCL getFitnessScore 是均方距离，不能替代原有 fitness（source 内点比例）。
  // 在精配准的最终位姿上重新计算内点数与 RMSE，距离门限保持原参数语义。
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(fine_target);
  Cloud aligned;
  pcl::transformPointCloud(*fine_source, aligned, transform);
  size_t inliers = 0;
  double squared_error = 0;
  const double distance2 = config_.correspondence_distance * config_.correspondence_distance;
  std::vector<int> index(1);
  std::vector<float> distance(1);
  for (const auto& p : aligned) {
    if (tree.nearestKSearch(p, 1, index, distance) == 1 && distance[0] <= distance2) {
      ++inliers; squared_error += distance[0];
    }
  }
  result.fitness = static_cast<double>(inliers) / fine_source->size();
  result.rmse = inliers ? std::sqrt(squared_error / inliers) : 0.0;
  result.accepted = converged && inliers >= static_cast<size_t>(config_.min_points) &&
      result.fitness >= config_.min_fitness && result.rmse <= config_.max_rmse;
  result.reason = result.accepted ? "ICP accepted" : "ICP rejected by convergence/fitness/RMSE/correspondence checks";
  Pose map_body = Pose::Identity();
  map_body.matrix() = transform.cast<double>();
  result.correction = map_body * odom.inverse();
  if (result.accepted) {
    pcl::transformPointCloud(*scan, aligned, transform);
    pcl::toROSMsg(aligned, result.scan);
    result.scan.header = message.header;
    result.scan.header.frame_id = config_.map_frame;
  }
  return result;
}
}  // namespace fastlio_localization

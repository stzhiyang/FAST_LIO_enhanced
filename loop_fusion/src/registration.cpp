// ICP verification follows FAST_LIO_SLAM SC-PGO by Giseop Kim.
#include "loop_fusion/registration.h"
#include <pcl/common/common.h>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <cmath>

namespace loop_fusion {
void VoxelMap::add(const Cloud& cloud, const Eigen::Matrix4d& T) {
  for (const auto& point : cloud) {
    if (!pcl::isFinite(point)) continue;
    const Eigen::Vector3d p = T.topLeftCorner<3, 3>() *
        Eigen::Vector3d(point.x, point.y, point.z) + T.topRightCorner<3, 1>();
    const Key key{static_cast<int64_t>(std::floor(p.x() / voxel_)),
                  static_cast<int64_t>(std::floor(p.y() / voxel_)),
                  static_cast<int64_t>(std::floor(p.z() / voxel_))};
    auto& cell = cells_[key];
    cell.x += p.x(); cell.y += p.y(); cell.z += p.z();
    cell.intensity += point.intensity; ++cell.count;
  }
}
Cloud::Ptr VoxelMap::cloud() const {
  Cloud::Ptr result(new Cloud);
  result->reserve(cells_.size());
  for (const auto& item : cells_) {
    const auto& cell = item.second;
    Point point;
    point.x = cell.x / cell.count; point.y = cell.y / cell.count; point.z = cell.z / cell.count;
    point.intensity = cell.intensity / cell.count;
    result->push_back(point);
  }
  result->is_dense = true;
  return result;
}
Cloud::Ptr downsample(const Cloud::ConstPtr& input, double voxel) {
  Cloud::Ptr result(new Cloud);
  if (input->empty()) return result;
  Eigen::Vector4f low, high;
  pcl::getMinMax3D(*input, low, high);
  long double volume = 1;
  for (int axis = 0; axis < 3; ++axis)
    volume *= std::floor((static_cast<long double>(high[axis]) - low[axis]) / voxel) + 1;
  if (volume > std::numeric_limits<int32_t>::max()) {
    VoxelMap sparse(voxel);
    sparse.add(*input, Eigen::Matrix4d::Identity());
    return sparse.cloud();
  }
  pcl::VoxelGrid<Point> filter;
  filter.setLeafSize(voxel, voxel, voxel);
  filter.setInputCloud(input);
  filter.filter(*result);
  return result;
}
Cloud::Ptr transform(const Cloud::ConstPtr& input, const Eigen::Matrix4d& T) {
  Cloud::Ptr result(new Cloud);
  pcl::transformPointCloud(*input, *result, T.cast<float>());
  return result;
}
RegistrationResult registerClouds(const Cloud::ConstPtr& source,
    const Cloud::ConstPtr& target, const Eigen::Matrix4d& initial,
    const RegistrationConfig& config) {
  RegistrationResult result;
  if (source->size() < 3 || target->size() < 3) {
    result.reason = "too_few_points";
    return result;
  }
  pcl::IterativeClosestPoint<Point, Point> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaxCorrespondenceDistance(config.max_correspondence);
  icp.setMaximumIterations(config.iterations);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);
  Cloud aligned;
  // The complete current-body -> history-body measurement includes the guess.
  icp.align(aligned, initial.cast<float>().eval());
  result.converged = icp.hasConverged();
  result.T_target_source = icp.getFinalTransformation().cast<double>();
  if (!result.converged || !result.T_target_source.allFinite()) {
    result.reason = "not_converged";
    return result;
  }
  // Same SC-PGO score, over all source nearest neighbors. The large search
  // radius does not relax the final geometric check.
  result.fitness = icp.getFitnessScore();
  result.rmse = std::sqrt(result.fitness);
  if (!std::isfinite(result.fitness) || result.fitness > config.max_fitness)
    result.reason = "fitness";
  else { result.accepted = true; result.reason = "accepted"; }
  return result;
}
}  // namespace loop_fusion

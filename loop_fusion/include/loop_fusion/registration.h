#pragma once
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>
#include <string>
#include <cstdint>
#include <unordered_map>

namespace loop_fusion {
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
struct RegistrationConfig {
  double max_correspondence = 150.0;
  int iterations = 100;
  double max_fitness = 0.3;  // PCL mean squared nearest-neighbor distance, m^2
};
struct RegistrationResult {
  Eigen::Matrix4d T_target_source = Eigen::Matrix4d::Identity();
  bool converged = false, accepted = false;
  double fitness = std::numeric_limits<double>::infinity();
  double rmse = std::numeric_limits<double>::infinity();
  std::string reason = "not_run";
};
// Sparse world voxels avoid PCL's flattened 32-bit grid-index overflow.
// Accumulate transformed scans without constructing a full-resolution global cloud.
class VoxelMap {
 public:
  explicit VoxelMap(double voxel) : voxel_(voxel) {}
  void add(const Cloud& cloud, const Eigen::Matrix4d& T);
  Cloud::Ptr cloud() const;
  void clear() { cells_.clear(); }
 private:
  struct Key {
    int64_t x, y, z;
    bool operator==(const Key& other) const { return x == other.x && y == other.y && z == other.z; }
  };
  struct Hash {
    size_t operator()(const Key& key) const {
      size_t h = std::hash<int64_t>{}(key.x);
      h ^= std::hash<int64_t>{}(key.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
      return h ^ (std::hash<int64_t>{}(key.z) + 0x9e3779b9 + (h << 6) + (h >> 2));
    }
  };
  struct Cell { double x = 0, y = 0, z = 0, intensity = 0; uint64_t count = 0; };
  double voxel_;
  std::unordered_map<Key, Cell, Hash> cells_;
};
Cloud::Ptr downsample(const Cloud::ConstPtr& input, double voxel);
Cloud::Ptr transform(const Cloud::ConstPtr& input, const Eigen::Matrix4d& T);
RegistrationResult registerClouds(const Cloud::ConstPtr& source,
    const Cloud::ConstPtr& target, const Eigen::Matrix4d& initial,
    const RegistrationConfig& config);
}  // namespace loop_fusion

// Scan Context: Giseop Kim and Ayoung Kim, IROS 2018.
// Modified 2026-10-10: configurable geometry, safe historical KNN, match diagnostics.
#pragma once
#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "scancontext/nanoflann.hpp"
#include "scancontext/KDTreeVectorOfVectorsAdaptor.h"

using namespace Eigen;
using std::cout;
using std::endl;
using std::make_pair;
using SCPointType = pcl::PointXYZI;
using KeyMat = std::vector<std::vector<float>>;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor<KeyMat, float>;

float xy2theta(const float& x, const float& y);
MatrixXd circshift(MatrixXd& matrix, int shift);
std::vector<float> eig2stdvec(MatrixXd matrix);

class SCManager {
 public:
    Eigen::MatrixXd makeScancontext(pcl::PointCloud<SCPointType>& cloud);
    Eigen::MatrixXd makeRingkeyFromScancontext(Eigen::MatrixXd& descriptor);
    Eigen::MatrixXd makeSectorkeyFromScancontext(Eigen::MatrixXd& descriptor);
    int fastAlignUsingVkey(MatrixXd& first, MatrixXd& second);
    double distDirectSC(MatrixXd& first, MatrixXd& second);
    std::pair<double, int> distanceBtnScanContext(MatrixXd& first, MatrixXd& second);
    void makeAndSaveScancontextAndKeys(pcl::PointCloud<SCPointType>& cloud);
    std::pair<int, float> detectLoopClosureID(size_t history_limit = std::numeric_limits<size_t>::max());
    const Eigen::MatrixXd& getConstRefRecentSCD();
    void setSCdistThres(double threshold);
    void setMaximumRadius(double radius);

    // Configure before inserting the first descriptor; gravity leveling is optional.
    double LIDAR_HEIGHT = 2.0;
    int PC_NUM_RING = 20;
    int PC_NUM_SECTOR = 60;
    double PC_MAX_RADIUS = 80.0;
    double PC_UNIT_SECTORANGLE = 360.0 / PC_NUM_SECTOR;
    double PC_UNIT_RINGGAP = PC_MAX_RADIUS / PC_NUM_RING;
    int NUM_EXCLUDE_RECENT = 30;
    int NUM_CANDIDATES_FROM_TREE = 3;
    const double SEARCH_RATIO = 0.1;
    double SC_DIST_THRES = 0.2;

    std::vector<Eigen::MatrixXd> polarcontexts_;
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
    std::vector<Eigen::MatrixXd> polarcontext_vkeys_;
    KeyMat polarcontext_invkeys_mat_;
    KeyMat polarcontext_invkeys_to_search_;
    std::unique_ptr<InvKeyTree> polarcontext_tree_;
    double last_distance = std::numeric_limits<double>::infinity();
    int last_candidate = -1;
};

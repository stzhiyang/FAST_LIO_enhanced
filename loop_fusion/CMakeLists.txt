cmake_minimum_required(VERSION 3.10)
project(loop_fusion)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs sensor_msgs geometry_msgs
  std_msgs std_srvs visualization_msgs pcl_ros pcl_conversions)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration kdtree io)
find_package(GTSAM REQUIRED)
find_package(Threads REQUIRED)
catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp nav_msgs sensor_msgs
  geometry_msgs std_msgs std_srvs visualization_msgs pcl_ros pcl_conversions)
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
add_definitions(${PCL_DEFINITIONS})
add_library(loop_geometry src/registration.cpp include/scancontext/Scancontext.cpp)
target_link_libraries(loop_geometry ${PCL_LIBRARIES})
add_executable(fastlio_loop src/loop_closure.cpp)
target_compile_definitions(fastlio_loop PRIVATE
  DEFAULT_RESULTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/PCD")
target_link_libraries(fastlio_loop loop_geometry gtsam ${catkin_LIBRARIES} Threads::Threads)
install(TARGETS fastlio_loop loop_geometry
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY launch config rviz_cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

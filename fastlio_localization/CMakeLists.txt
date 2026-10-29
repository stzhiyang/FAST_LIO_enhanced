cmake_minimum_required(VERSION 3.10)
project(fastlio_localization)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED COMPONENTS roscpp message_filters nav_msgs sensor_msgs geometry_msgs
  diagnostic_msgs tf2_ros pcl_conversions)
find_package(PCL REQUIRED COMPONENTS common filters registration kdtree io)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
# FindEigen3 提供 EIGEN3_INCLUDE_DIRS；catkin_package 导出检查的是 Eigen3_INCLUDE_DIRS。
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp message_filters
  nav_msgs sensor_msgs geometry_msgs diagnostic_msgs tf2_ros pcl_conversions DEPENDS PCL Eigen3)
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
# 小模块不单独拆分入口文件：节点入口与定位逻辑同在一个源文件。
add_executable(fastlio_localization_node src/localization.cpp src/registration.cpp)
target_link_libraries(fastlio_localization_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} Threads::Threads)
install(TARGETS fastlio_localization_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# FAST_LIO_enhanced

**面向无人机定位与三维建图的激光惯性里程计**

基于 FAST-LIO2，融合激光雷达与 IMU 数据完成运动状态估计和三维点云建图。新增 IMU 高频状态传播，在激光帧更新之间持续输出位姿、线速度与角速度，为飞行控制和导航提供高频运动状态。

主要技术栈：**C++14、ROS Noetic、Eigen、PCL、IKFoM、ikd-Tree**。

## 功能

- **激光惯性融合与建图**：通过迭代误差状态卡尔曼滤波完成状态估计，结合点云运动补偿、scan-to-map 匹配和 ikd-Tree 增量地图维护完成建图。
- **高频里程计**：在激光帧更新之间，使用 IMU 中值积分传播运动状态，发布 `/Odometry_highrate`。
- **校正后状态重传播**：将高频状态同步至激光校正时刻，重放该时刻之后已缓存的 IMU 测量，使预测状态跟进最新测量。
- **Livox 驱动接口**：采用 `livox_ros_driver2` 消息接口；AVIA 与 MID360 启动配置默认开启高频里程计，支持参数开关。
- **ROS 可视化接口**：输出里程计、配准点云、轨迹及坐标变换，支持 RViz 查看定位与建图结果。

## 高频里程计方法

高频传播维护独立的位姿、速度、IMU 偏置与重力状态。每次收到 IMU 测量后，使用相邻测量的平均角速度更新姿态，将起止时刻的加速度变换到世界坐标系并补偿重力，再通过中值积分更新速度和位置。

激光校正完成后，将前端状态同步到高频传播状态，插值得到校正时刻的边界 IMU 测量，并按时间顺序重传播后续缓存测量。高频传播使用与前端一致的时间偏移和加速度尺度。

`/Odometry_highrate` 的位姿以 `camera_init` 为参考坐标系，速度以 `body` 为参考坐标系，消息时间戳对应 IMU 测量时刻。原 `/Odometry` 保持激光帧更新频率。

## 验证结果

在 `hku_campus_seq_00.bag` 前 25 秒数据、1 倍速回放下：

| 指标 | 结果 |
| --- | --- |
| 原里程计输出频率 | 约 10 Hz |
| 高频里程计输出频率 | 约 204 Hz |
| 输出频率提升 | 约 20 倍 |
| 开关高频功能前后的激光帧位置输出 | 245 个对应帧的位置差为 0 |
| 高频消息时间戳 | 严格递增 |

已通过 ROS Noetic 编译、IMU 积分与重传播检查，以及数据回放验证。上述频率按消息时间戳统计。

## 工程结构

```text
FAST_LIO_enhanced/
├── FAST_LIO/
│   ├── config/                 # 传感器、外参与估计参数
│   ├── include/                # 状态定义、滤波与地图数据结构
│   ├── launch/                 # ROS 启动配置
│   ├── src/
│   │   ├── imu_odometry.hpp    # 高频 IMU 传播与状态同步
│   │   ├── IMU_Processing.hpp  # IMU 初始化与点云运动补偿
│   │   └── laserMapping.cpp    # 激光惯性估计、建图与输出
│   ├── CMakeLists.txt
│   └── package.xml
└── README.md
```

ROS 包名为 `fast_lio`，主节点为 `fastlio_mapping`。

## 环境与运行

使用 ROS Noetic 和 catkin 构建，依赖 Eigen、PCL 及 `livox_ros_driver2`。以下示例约定驱动工作空间位于 `~/sensor_ws`，项目位于 `~/slam_learning_ws/src/FAST_LIO_enhanced`：

```bash
source /opt/ros/noetic/setup.bash
source ~/sensor_ws/devel/setup.bash

cd ~/slam_learning_ws
catkin build
source devel/setup.bash

# Livox AVIA
roslaunch fast_lio mapping_avia.launch

# Livox MID360
roslaunch fast_lio mapping_mid360.launch
```

关闭 RViz 或高频里程计：

```bash
roslaunch fast_lio mapping_avia.launch rviz:=false
roslaunch fast_lio mapping_avia.launch highrate_odom:=false
```

查看两路里程计输出频率：

```bash
rostopic hz /Odometry
rostopic hz /Odometry_highrate
```

运行前根据传感器和数据集配置 `FAST_LIO/config/` 中的输入话题、时间偏移、LiDAR–IMU 外参及预处理参数。

## 上游与授权

本项目基于 FAST_LIO 扩展。上游作者信息、论文引用及使用说明见 [FAST_LIO README](FAST_LIO/README.md)，授权条款见 [FAST_LIO/LICENSE](FAST_LIO/LICENSE)。

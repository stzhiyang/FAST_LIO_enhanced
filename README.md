# FAST_LIO_enhanced

**面向无人机定位与三维建图的激光惯性里程计**

基于 FAST-LIO2，融合激光雷达与 IMU 数据完成运动状态估计和三维点云建图。已实现 IMU 高频里程计，在激光帧更新之间持续输出位姿、线速度与角速度。

主要技术栈：**C++14、ROS Noetic、Eigen、PCL、IKFoM、ikd-Tree**。

## 功能

- **激光惯性融合与建图**：通过迭代误差状态卡尔曼滤波完成状态估计，结合点云运动补偿、scan-to-map 匹配和 ikd-Tree 增量地图维护完成建图。
- **高频里程计（已实现）**：使用 IMU 中值积分传播运动状态，激光校正后同步状态并重传播缓存 IMU，发布 `/Odometry_highrate`。
- **Livox 驱动接口**：采用 `livox_ros_driver2` 消息接口；AVIA、MID360 与 HKU Campus 启动配置默认开启高频里程计，支持参数开关。
- **ROS 可视化接口**：输出里程计、配准点云、轨迹及坐标变换，支持 RViz 查看定位与建图结果。

## 高频里程计

每处理一条新 IMU，补偿偏置和重力后通过中值积分更新运动状态；激光校正后同步前端状态，插值校正时刻的 IMU 测量并重传播后续缓存。时间偏移和加速度尺度与前端一致。

`/Odometry_highrate` 的位姿以 `camera_init` 为参考坐标系，速度以 `body` 为参考坐标系，消息时间戳对应 IMU 测量时刻。原 `/Odometry` 保持激光帧更新频率。

### 效果对比

对比同一次运行中的三维位置差、实际输出频率和三维轨迹。位置差以原始里程计为参考，在其时间戳处匹配最近的高频样本（相差不超过 3 ms），计算三维位置差的模长；频率按实际接收时间统计。

<!-- highrate-comparison:start -->

![原始里程计与高频里程计的位置差、频率和三维轨迹对比](assets/highrate_comparison.png)

<!-- highrate-comparison:end -->

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

## 上游与授权

本项目基于 FAST_LIO 扩展。上游作者信息、论文引用及使用说明见 [FAST_LIO README](FAST_LIO/README.md)，授权条款见 [FAST_LIO/LICENSE](FAST_LIO/LICENSE)。

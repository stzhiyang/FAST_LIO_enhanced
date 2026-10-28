#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
# FAST_LIO_LOCALIZATION: HViktorTsoi and contributors.
# https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION
# Python 3 / FAST_LIO_enhanced adaptation; see ../NOTICE and ../LICENSE.
"""独立的 scan-to-map 定位；前端状态和回环后端均不被修改。"""
import copy
import threading
import time
from collections import deque

import message_filters
import numpy as np
import open3d as o3d
import rospy
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf.transformations import quaternion_matrix, quaternion_from_matrix


def pose_matrix(pose):
    q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    p = np.array([pose.position.x, pose.position.y, pose.position.z])
    if not np.isfinite(q).all() or not np.isfinite(p).all() or np.linalg.norm(q) < 1e-6:
        raise ValueError('invalid pose')
    result = quaternion_matrix(q / np.linalg.norm(q))
    result[:3, 3] = p
    return result


def cloud_xyz(msg):
    # ROS 按字段名、offset、point_step、row_step 和大小端解码，兼容字段重排和行填充。
    if not {'x', 'y', 'z'}.issubset({field.name for field in msg.fields}):
        raise ValueError('cloud requires x/y/z fields')
    xyz = np.asarray(list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'),
                                                 skip_nans=True)), dtype=np.float64).reshape(-1, 3)
    ordered = [field.name for field in sorted(msg.fields, key=lambda field: field.offset)
               if field.name in ('x', 'y', 'z')]
    if any(field.count != 1 for field in msg.fields if field.name in ('x', 'y', 'z')):
        raise ValueError('x/y/z fields must be scalar')
    xyz = xyz[:, [ordered.index(name) for name in ('x', 'y', 'z')]]
    return xyz[np.isfinite(xyz).all(axis=1)]


def point_cloud(xyz):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)
    return cloud


def corrected_odometry(msg, correction, map_frame, body_frame):
    result = copy.deepcopy(msg)
    transform = correction @ pose_matrix(msg.pose.pose)
    result.header.frame_id = map_frame
    result.child_frame_id = body_frame
    result.pose.pose.position.x, result.pose.pose.position.y, result.pose.pose.position.z = transform[:3, 3]
    q = quaternion_from_matrix(transform)
    result.pose.pose.orientation.x, result.pose.pose.orientation.y, result.pose.pose.orientation.z, result.pose.pose.orientation.w = q
    # 两个 child frame 同轴，twist 保持 body 表达；只旋转前端位姿协方差，未估计 ICP 不确定度。
    axes = np.zeros((6, 6))
    axes[:3, :3] = axes[3:, 3:] = correction[:3, :3]
    result.pose.covariance = (axes @ np.reshape(msg.pose.covariance, (6, 6)) @ axes.T).ravel().tolist()
    return result


class Localization:
    def __init__(self):
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_frame = 'camera_init'
        self.body_frame = 'body'
        self.localized_body = rospy.get_param('~localized_body_frame', 'localization_body')
        if (not self.map_frame or not self.localized_body or
                len({self.map_frame, self.localized_body, 'camera_init', 'body', 'body_loop'}) != 5):
            raise ValueError('localization frames must be distinct from frontend/loop frames')
        # 默认值集中在此；距离以 m、角度以 rad、频率以 Hz 表达。
        defaults = dict(map_voxel=0.4, scan_voxel=0.1, coarse_scale=5.0,
                        correspondence_distance=1.0, frequency=0.5, min_fitness=0.95,
                        max_rmse=0.5, submap_radius=150.0, initial_time_tolerance=1.0,
                        min_points=30, max_iterations=20)
        self.params = {key: rospy.get_param('~' + key, value) for key, value in defaults.items()}
        for key, value in self.params.items():
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise ValueError('invalid parameter: ' + key)
        if self.params['min_fitness'] > 1 or self.params['coarse_scale'] < 1:
            raise ValueError('min_fitness must be <= 1; coarse_scale must be >= 1')
        for key in ('min_points', 'max_iterations'):
            if not isinstance(self.params[key], int):
                raise ValueError(key + ' must be an integer')
        self.map = o3d.io.read_point_cloud(rospy.get_param('~map_path'))
        xyz = np.asarray(self.map.points)
        self.map = point_cloud(xyz[np.isfinite(xyz).all(axis=1)]).voxel_down_sample(self.params['map_voxel'])
        if len(self.map.points) < self.params['min_points']:
            raise ValueError('map is missing, empty or too small')
        self.map_xyz = np.asarray(self.map.points)
        self.lock = threading.RLock()
        self.wake = threading.Event()
        self.history = deque(maxlen=100)
        self.latest = None
        self.seed = None
        self.correction = None
        self.correction_stamp = rospy.Time()
        self.generation = 0
        self.last_attempt = None
        self.state = 'UNINITIALIZED'
        self.reason = 'waiting for synchronized scan and /initialpose'
        self.fitness = 0.0
        self.rmse = 0.0
        self.odom_pub = rospy.Publisher('~odometry', Odometry, queue_size=100)
        self.high_pub = rospy.Publisher('~odometry_highrate', Odometry, queue_size=1000)
        self.map_pub = rospy.Publisher('~map', PointCloud2, queue_size=1, latch=True)
        self.scan_pub = rospy.Publisher('~scan', PointCloud2, queue_size=1)
        self.correction_pub = rospy.Publisher('~map_to_odom', TransformStamped, queue_size=1)
        self.status_pub = rospy.Publisher('~status', DiagnosticArray, queue_size=1, latch=True)
        self.tf = tf2_ros.TransformBroadcaster()
        self.map_pub.publish(point_cloud2.create_cloud_xyz32(Header(frame_id=self.map_frame), self.map_xyz))
        self.publish_status()
        self.odom_sub = message_filters.Subscriber('/Odometry', Odometry, queue_size=100)
        self.cloud_sub = message_filters.Subscriber('/cloud_registered_body', PointCloud2, queue_size=10)
        self.sync = message_filters.TimeSynchronizer([self.odom_sub, self.cloud_sub], 100)
        self.sync.registerCallback(self.paired)
        self.odom_sub.registerCallback(self.publish_odometry, self.odom_pub, True)
        self.high_sub = rospy.Subscriber('/Odometry_highrate', Odometry, self.publish_odometry,
                                         callback_args=(self.high_pub, False), queue_size=1000)
        self.initial_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped,
                                            self.initial_pose, queue_size=1)
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()
        rospy.on_shutdown(self.wake.set)

    def valid_odom(self, odom):
        return odom.header.frame_id == self.odom_frame and odom.child_frame_id == self.body_frame

    def paired(self, odom, cloud):
        if not self.valid_odom(odom) or cloud.header.frame_id != self.body_frame:
            rospy.logwarn_throttle(5, 'Localization expects camera_init/body odometry and body cloud')
            return
        try:
            pose_matrix(odom.pose.pose)
        except ValueError as error:
            rospy.logwarn_throttle(5, str(error))
            return
        with self.lock:
            if self.latest and odom.header.stamp < self.latest[0].header.stamp:
                # 回放时间回退意味着局部里程计可能重建，旧修正不可继续使用。
                self.generation += 1
                self.correction = self.seed = None
                self.history.clear()
                self.state, self.reason = 'UNINITIALIZED', 'input time moved backwards; provide /initialpose again'
                self.publish_status()
            self.latest = (odom, cloud)
            self.history.append(self.latest)

    def initial_pose(self, msg):
        try:
            if msg.header.frame_id != self.map_frame:
                raise ValueError('/initialpose must be expressed in ' + self.map_frame)
            initial = pose_matrix(msg.pose.pose)
            with self.lock:
                if not self.history:
                    raise ValueError('wait for synchronized odometry/cloud before /initialpose')
                pair = self.latest if msg.header.stamp == rospy.Time() else min(
                    self.history, key=lambda pair: abs((pair[0].header.stamp - msg.header.stamp).to_sec()))
                if (msg.header.stamp != rospy.Time() and
                        abs((pair[0].header.stamp - msg.header.stamp).to_sec()) > self.params['initial_time_tolerance']):
                    raise ValueError('/initialpose timestamp has no nearby synchronized scan')
                # 初值是 map 中的 body 位姿。用同一帧的局部位姿换算修正，不能直接当 map->odom。
                self.seed = initial @ np.linalg.inv(pose_matrix(pair[0].pose.pose))
                self.initial_pair = pair
                self.correction = None
                self.correction_stamp = rospy.Time()
                self.fitness = self.rmse = 0.0
                self.generation += 1
                self.last_attempt = None
                self.state, self.reason = 'UNINITIALIZED', 'initial pose accepted; awaiting ICP'
                self.publish_status()
                self.wake.set()
        except ValueError as error:
            rospy.logwarn('Rejected initial pose: %s', error)

    def register(self, cloud_msg, odom, seed):
        scan = point_cloud(cloud_xyz(cloud_msg))
        predicted = seed @ pose_matrix(odom.pose.pose)
        # 球形局部子图兼容前视和环视雷达，不假设 body 的 x 轴就是雷达光轴。
        indices = np.linalg.norm(self.map_xyz - predicted[:3, 3], axis=1) <= self.params['submap_radius']
        target = point_cloud(self.map_xyz[indices])
        if min(len(scan.points), len(target.points)) < self.params['min_points']:
            raise ValueError('too few scan/submap points')
        transform = predicted
        registration = o3d.pipelines.registration
        # 保留粗(scale=5)、精(scale=1)两级 point-to-point ICP；body 点云输出 map->body。
        for scale in (self.params['coarse_scale'], 1.0):
            source_down = scan.voxel_down_sample(self.params['scan_voxel'] * scale)
            target_down = target.voxel_down_sample(self.params['map_voxel'] * scale)
            if min(len(source_down.points), len(target_down.points)) < 3:
                raise ValueError('too few voxelized points')
            result = registration.registration_icp(
                source_down, target_down, self.params['correspondence_distance'] * scale, transform,
                registration.TransformationEstimationPointToPoint(),
                registration.ICPConvergenceCriteria(max_iteration=self.params['max_iterations']))
            transform = result.transformation
        accepted = (np.isfinite(transform).all() and np.isfinite(result.inlier_rmse) and
                    result.fitness >= self.params['min_fitness'] and
                    result.inlier_rmse <= self.params['max_rmse'] and
                    len(result.correspondence_set) >= self.params['min_points'])
        correction = transform @ np.linalg.inv(pose_matrix(odom.pose.pose))
        return accepted, correction, result.fitness, result.inlier_rmse, np.asarray(scan.points)

    def run(self):
        # 墙上时间调度允许 bag 暂停后输入初值；同一帧在同一代初始化中只处理一次。
        next_match = 0.0
        while not rospy.is_shutdown():
            self.wake.wait(0.1)
            immediate = self.wake.is_set()
            self.wake.clear()
            with self.lock:
                if self.seed is None or self.latest is None:
                    continue
                if not immediate and time.monotonic() < next_match:
                    continue
                pair = self.initial_pair if self.last_attempt is None else self.latest
                token = (self.generation, pair[0].header.stamp.to_nsec())
                if token == self.last_attempt:
                    continue
                self.last_attempt = token
                seed = (self.correction if self.correction is not None else self.seed).copy()
            next_match = time.monotonic() + 1.0 / self.params['frequency']
            try:
                accepted, correction, fitness, rmse, xyz = self.register(pair[1], pair[0], seed)
                reason = 'ICP accepted' if accepted else 'ICP rejected by fitness/RMSE/correspondence checks'
            except Exception as error:
                accepted, fitness, rmse = False, 0.0, 0.0
                reason = 'ICP failed: ' + str(error)
                rospy.logwarn('%s', reason)
            with self.lock:
                # 新初值到达后，正在运行的旧 ICP 结果必须丢弃。
                if token[0] != self.generation:
                    continue
                self.fitness, self.rmse = fitness, rmse
                self.state = 'MATCHED' if accepted else 'MATCH_FAILED'
                self.reason = reason
                if accepted:
                    self.correction = correction
                    self.correction_stamp = pair[0].header.stamp
                    transform = self.transform_msg(correction, self.correction_stamp, self.odom_frame)
                    self.correction_pub.publish(transform)
                    map_body = correction @ pose_matrix(pair[0].pose.pose)
                    mapped = xyz @ map_body[:3, :3].T + map_body[:3, 3]
                    self.scan_pub.publish(point_cloud2.create_cloud_xyz32(
                        Header(stamp=self.correction_stamp, frame_id=self.map_frame), mapped))
                self.publish_status()

    def transform_msg(self, matrix, stamp, child):
        msg = TransformStamped()
        msg.header = Header(stamp=stamp, frame_id=self.map_frame)
        msg.child_frame_id = child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = matrix[:3, 3]
        q = quaternion_from_matrix(matrix)
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
        return msg

    def publish_odometry(self, msg, publisher, broadcast=None):
        # rospy callback_args 与 message_filters 的参数传递方式不同。
        if broadcast is None:
            publisher, broadcast = publisher
        if not self.valid_odom(msg):
            return
        with self.lock:
            if self.correction is None or msg.header.stamp < self.correction_stamp:
                return
            try:
                result = corrected_odometry(msg, self.correction, self.map_frame, self.localized_body)
            except ValueError:
                return
            publisher.publish(result)
            # 仅普通里程计负责 TF，避免两路输出乱序覆盖同一 child；不连接 camera_init/body。
            if broadcast:
                self.tf.sendTransform(self.transform_msg(pose_matrix(result.pose.pose), msg.header.stamp,
                                                         self.localized_body))

    def publish_status(self):
        status = DiagnosticStatus(name='localization', hardware_id='scan_to_map',
                                  level=DiagnosticStatus.OK if self.state == 'MATCHED' else DiagnosticStatus.WARN,
                                  message=self.state)
        values = dict(initialized=str(self.correction is not None).lower(), reason=self.reason,
                      fitness=str(self.fitness), rmse=str(self.rmse),
                      last_success_stamp=str(self.correction_stamp.to_sec()), generation=str(self.generation))
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        self.status_pub.publish(DiagnosticArray(header=Header(stamp=rospy.Time.now()), status=[status]))


if __name__ == '__main__':
    rospy.init_node('localization')
    try:
        node = Localization()
        rospy.spin()
    except (ValueError, KeyError, RuntimeError) as error:
        rospy.logfatal('Localization startup failed: %s', error)
        raise SystemExit(1)

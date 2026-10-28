#!/usr/bin/env python3
"""通过真实 ROS 消息与 Open3D 验证定位契约，不依赖用户地图。"""
import importlib.util
import os
import tempfile
import threading
import time
import unittest
from pathlib import Path

import numpy as np
import open3d as o3d
import rospy
import rostest
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointField, PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from tf.transformations import quaternion_from_matrix, euler_matrix

spec = importlib.util.spec_from_file_location('localization', str(Path(__file__).resolve().parents[1] / 'scripts/localization.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def pose(message, matrix):
    message.position.x, message.position.y, message.position.z = matrix[:3, 3]
    q = quaternion_from_matrix(matrix)
    message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = q


def wait_for(condition, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(0.02)
    raise AssertionError('timed out waiting for condition')


class Contracts(unittest.TestCase):
    def test_localization(self):
        rng = np.random.RandomState(7)
        # 不规则三维结构提供已知变换，避免平面/对称场景中多解。
        xyz = rng.uniform([-8, -5, -2], [8, 5, 3], (2500, 3))
        body_map = euler_matrix(0.05, -0.03, 0.4)
        body_map[:3, 3] = [12, -6, 2]
        body_odom = euler_matrix(-0.02, 0.04, -0.3)
        body_odom[:3, 3] = [4, 3, -1]
        expected = body_map @ np.linalg.inv(body_odom)
        with tempfile.TemporaryDirectory(prefix='fastlio_localization_test_') as directory:
            map_path = os.path.join(directory, 'map.pcd')
            o3d.io.write_point_cloud(map_path, module.point_cloud(xyz @ body_map[:3, :3].T + body_map[:3, 3]))
            rospy.set_param('~map_path', map_path)
            rospy.set_param('~frequency', 10.0)
            rospy.set_param('~map_voxel', 0.1)
            rospy.set_param('~min_fitness', 0.95)
            node = module.Localization()
            normal, high = [], []
            subscribers = [rospy.Subscriber('~odometry', Odometry, normal.append),
                           rospy.Subscriber('~odometry_highrate', Odometry, high.append)]
            odom_pub = rospy.Publisher('/Odometry', Odometry, queue_size=10)
            high_pub = rospy.Publisher('/Odometry_highrate', Odometry, queue_size=10)
            cloud_pub = rospy.Publisher('/cloud_registered_body', PointCloud2, queue_size=10)
            initial_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
            wait_for(lambda: all(pub.get_num_connections() for pub in [odom_pub, high_pub, cloud_pub, initial_pub])
                     and node.odom_pub.get_num_connections() and node.high_pub.get_num_connections())

            def odom(stamp):
                msg = Odometry(header=Header(stamp=rospy.Time.from_sec(stamp), frame_id='camera_init'), child_frame_id='body')
                pose(msg.pose.pose, body_odom)
                msg.twist.twist.linear.x = 2.0
                msg.pose.covariance = np.diag([1, 2, 3, 4, 5, 6]).ravel().tolist()
                return msg

            def cloud(stamp, points=xyz):
                # 物理顺序 z,x,y，fields 列表也不按 offset 排序，附加 intensity。
                fields = [PointField('x', 4, PointField.FLOAT32, 1), PointField('intensity', 12, PointField.FLOAT32, 1),
                          PointField('z', 0, PointField.FLOAT32, 1), PointField('y', 8, PointField.FLOAT32, 1)]
                return point_cloud2.create_cloud(Header(stamp=rospy.Time.from_sec(stamp), frame_id='body'), fields,
                                                 np.column_stack([points[:, 2], points[:, 0], points[:, 1], np.ones(len(points))]))

            def pair(stamp, points=xyz):
                odom_pub.publish(odom(stamp))
                cloud_pub.publish(cloud(stamp, points))
                wait_for(lambda: node.latest is not None and node.latest[0].header.stamp == rospy.Time.from_sec(stamp))

            np.testing.assert_allclose(module.cloud_xyz(cloud(10)), xyz, atol=1e-6)
            odom_pub.publish(odom(9))
            cloud_pub.publish(cloud(9.1))
            time.sleep(0.2)
            self.assertIsNone(node.latest, 'mismatched timestamps must not pair')
            pair(10)
            high_pub.publish(odom(10))
            time.sleep(0.1)
            self.assertEqual(node.state, 'UNINITIALIZED')
            self.assertFalse(normal or high, 'uninitialized output must be suppressed')
            initial = PoseWithCovarianceStamped(header=Header(stamp=rospy.Time.from_sec(10), frame_id='map'))
            guess = body_map.copy()
            guess[:3, 3] += [0.1, -0.1, 0.05]
            pose(initial.pose.pose, guess)
            initial_pub.publish(initial)
            wait_for(lambda: node.state == 'MATCHED')
            np.testing.assert_allclose(node.correction, expected, atol=0.015)
            odom_pub.publish(odom(10.01))
            high_pub.publish(odom(10.01))
            wait_for(lambda: normal and high)
            np.testing.assert_allclose(module.pose_matrix(normal[-1].pose.pose), body_map, atol=0.015)
            np.testing.assert_allclose(module.pose_matrix(normal[-1].pose.pose), module.pose_matrix(high[-1].pose.pose))
            self.assertEqual(normal[-1].header.stamp, high[-1].header.stamp)
            self.assertEqual(normal[-1].child_frame_id, 'localization_body')
            self.assertEqual(high[-1].twist.twist.linear.x, 2.0)
            np.testing.assert_allclose(normal[-1].pose.covariance, high[-1].pose.covariance)
            axes = np.zeros((6, 6)); axes[:3, :3] = axes[3:, 3:] = node.correction[:3, :3]
            np.testing.assert_allclose(np.reshape(normal[-1].pose.covariance, (6, 6)), axes @ np.diag([1,2,3,4,5,6]) @ axes.T)

            saved = node.correction.copy()
            pair(11, xyz + 1000)
            wait_for(lambda: node.state == 'MATCH_FAILED')
            np.testing.assert_array_equal(saved, node.correction)
            self.assertIsNotNone(node.correction, 'failed tracking retains last correction')
            high_pub.publish(odom(11.01))
            wait_for(lambda: high[-1].header.stamp == rospy.Time.from_sec(11.01))
            count = len(high)
            high_pub.publish(odom(9))
            time.sleep(0.1)
            self.assertEqual(len(high), count, 'old stamps cannot use future correction')

            # 阻塞旧 ICP，再输入新初值，确保旧计算不能覆盖重新初始化。
            original_register = node.register
            entered, release = threading.Event(), threading.Event()
            def delayed(*args):
                entered.set()
                release.wait(5)
                return original_register(*args)
            node.register = delayed
            pair(12)
            self.assertTrue(entered.wait(5))
            initial.header.stamp = rospy.Time.from_sec(12)
            bad_guess = body_map.copy(); bad_guess[:3, 3] += 1000
            pose(initial.pose.pose, bad_guess)
            generation = node.generation
            initial_pub.publish(initial)
            wait_for(lambda: node.generation > generation)
            self.assertIsNone(node.correction)
            self.assertEqual(node.state, 'UNINITIALIZED')
            node.register = original_register
            release.set()
            wait_for(lambda: node.state == 'MATCH_FAILED')
            self.assertIsNone(node.correction, 'old in-flight result must be discarded')
            count = len(high)
            high_pub.publish(odom(12.01))
            time.sleep(0.1)
            self.assertEqual(len(high), count)
            pose(initial.pose.pose, body_map)
            initial_pub.publish(initial)
            wait_for(lambda: node.state == 'MATCHED')
            np.testing.assert_allclose(node.correction, expected, atol=0.015)
            generation = node.generation
            initial.header.frame_id = 'camera_init'
            initial_pub.publish(initial)
            time.sleep(0.1)
            self.assertEqual(node.generation, generation, 'wrong-frame initial pose must be rejected')
            pair(1)
            self.assertEqual(node.state, 'UNINITIALIZED')
            self.assertIsNone(node.correction, 'clock rollback invalidates correction')
            self.assertTrue(subscribers)


if __name__ == '__main__':
    rospy.init_node('localization_contracts')
    rostest.rosrun('fastlio_localization', 'localization_contracts', Contracts)

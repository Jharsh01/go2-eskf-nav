#!/usr/bin/env python3
# test_ekf_integration.py
# 4 integration tests that spin up the EKF node with synthetic publishers and
# assert correct behaviour on /ekf/pose. Run with: colcon test --packages-select ekf_estimator

import math
import time
import unittest
from threading import Event

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


@pytest.mark.launch_test
def generate_test_description():
    ekf_node = launch_ros.actions.Node(
        package="ekf_estimator",
        executable="ekf_estimator_node",
        name="ekf_estimator",
        parameters=[{
            "predict_rate_hz": 50.0,
            "publish_tf": False,
            "imu_yaw_rate_var": 1e-4,
            "odom_v_var":       1e-4,
            "odom_w_var":       1e-4,
        }],
        output="screen",
    )
    return launch.LaunchDescription([
        ekf_node,
        launch_testing.actions.ReadyToTest(),
    ])


class EkfIntegrationFixture(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("ekf_test_client")
        self.imu_pub  = self.node.create_publisher(Imu,      "/imu",  10)
        self.odom_pub = self.node.create_publisher(Odometry, "/odom", 10)
        self.latest_pose = None
        self.pose_received = Event()

        def cb(msg):
            self.latest_pose = msg
            self.pose_received.set()

        self.node.create_subscription(
            PoseWithCovarianceStamped, "/ekf/pose", cb, 10)
        # Give pub/sub graph a moment to settle.
        end = time.time() + 2.0
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def tearDown(self):
        self.node.destroy_node()

    # --- Helpers ------------------------------------------------------------
    def make_imu(self, omega_z=0.0):
        msg = Imu()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.angular_velocity.z = omega_z
        return msg

    def make_odom(self, v=0.0, w=0.0):
        msg = Odometry()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.twist.twist.linear.x  = v
        msg.twist.twist.angular.z = w
        return msg

    def spin_and_publish(self, imu_msg, odom_msg, duration_s):
        """Publish at 50 Hz for duration_s seconds, spinning the test node."""
        end = time.time() + duration_s
        while time.time() < end:
            self.imu_pub.publish(imu_msg)
            self.odom_pub.publish(odom_msg)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def yaw(self, msg):
        q = msg.pose.pose.orientation
        # ZYX yaw extraction.
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # --- Tests --------------------------------------------------------------
    def test_01_stationary_robot_stays_put(self):
        self.spin_and_publish(self.make_imu(0.0), self.make_odom(0.0, 0.0), 2.0)
        self.assertTrue(self.pose_received.is_set(), "no /ekf/pose received")
        self.assertAlmostEqual(self.latest_pose.pose.pose.position.x, 0.0, delta=0.1)
        self.assertAlmostEqual(self.latest_pose.pose.pose.position.y, 0.0, delta=0.1)

    def test_02_pure_rotation_tracks_yaw(self):
        self.spin_and_publish(self.make_imu(0.5), self.make_odom(0.0, 0.5), 3.0)
        # Expected yaw ~ 0.5 rad/s * ~3s, but EKF lags initial estimate; just
        # check sign and that translation stayed near zero.
        self.assertGreater(self.yaw(self.latest_pose), 0.3)
        self.assertAlmostEqual(self.latest_pose.pose.pose.position.x, 0.0, delta=0.2)
        self.assertAlmostEqual(self.latest_pose.pose.pose.position.y, 0.0, delta=0.2)

    def test_03_straight_line_x_increases(self):
        self.spin_and_publish(self.make_imu(0.0), self.make_odom(1.0, 0.0), 2.0)
        self.assertGreater(self.latest_pose.pose.pose.position.x, 0.5)
        self.assertAlmostEqual(self.latest_pose.pose.pose.position.y, 0.0, delta=0.2)
        # Position covariance should be finite, not exploding.
        self.assertLess(self.latest_pose.pose.covariance[0], 5.0)

    def test_04_survives_sensor_dropout(self):
        # Drive forward for 1 s with both sensors.
        self.spin_and_publish(self.make_imu(0.0), self.make_odom(0.5, 0.0), 1.0)
        # Then stop publishing odom, IMU only.
        end = time.time() + 1.0
        while time.time() < end:
            self.imu_pub.publish(self.make_imu(0.0))
            rclpy.spin_once(self.node, timeout_sec=0.02)
        # Node should still be alive and publishing.
        self.pose_received.clear()
        end = time.time() + 1.0
        while time.time() < end and not self.pose_received.is_set():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(self.pose_received.is_set(),
                        "EKF stopped publishing after odom dropout")

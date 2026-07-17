#!/usr/bin/env python3
# Copyright 2026 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Fault-injection regression for the localization fusion executable."""

import signal
import subprocess
import time
import unittest

from ament_index_python.packages import get_package_prefix
from ats_navigation_interfaces.msg import LocalizationStatus
from ats_navigation_interfaces.msg import RelocalizationObservation
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


TOPIC_PREFIX = "/localization_fusion_fault_test"


class TestLocalizationFusionNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        executable = (
            get_package_prefix("small_gicp_relocalization")
            + "/lib/small_gicp_relocalization/localization_fusion_node"
        )
        parameters = {
            "odom_topic": f"{TOPIC_PREFIX}/odometry",
            "localization_topic": f"{TOPIC_PREFIX}/localization",
            "observation_topic": f"{TOPIC_PREFIX}/observation",
            "status_topic": f"{TOPIC_PREFIX}/status",
            "map_frame": "map",
            "odom_frame": "odom",
            "robot_base_frame": "base_test",
            "publish_tf": True,
            "allow_initial_identity": False,
            "odom_timeout_s": 0.25,
            "observation_timeout_s": 0.8,
            "observation_lost_timeout_s": 2.0,
            "history_duration_s": 3.0,
            "history_boundary_tolerance_s": 0.02,
            "maximum_interpolation_gap_s": 0.10,
            "transform_future_offset_s": 0.0,
            "relocalizing_hold_s": 0.08,
            "max_consecutive_rejections": 2,
            "epoch_translation_threshold": 0.05,
            "epoch_yaw_threshold": 0.05,
            "max_correction_translation": 0.5,
            "max_correction_yaw": 0.5,
            "min_observation_quality": 0.2,
            "min_observation_inliers": 10,
            "max_registration_error": 2.0,
        }
        command = [executable, "--ros-args", "-r", "__node:=fusion_fault_test"]
        for name, value in parameters.items():
            encoded = str(value).lower() if isinstance(value, bool) else str(value)
            command.extend(("-p", f"{name}:={encoded}"))
        cls.process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        rclpy.init()
        cls.node = rclpy.create_node("localization_fusion_fault_driver")
        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.odom_pub = cls.node.create_publisher(
            Odometry, f"{TOPIC_PREFIX}/odometry", qos_profile_sensor_data
        )
        cls.observation_pub = cls.node.create_publisher(
            RelocalizationObservation, f"{TOPIC_PREFIX}/observation", 10
        )
        cls.statuses = []
        cls.localizations = []
        cls.transforms = []
        cls.node.create_subscription(
            LocalizationStatus,
            f"{TOPIC_PREFIX}/status",
            cls.statuses.append,
            transient_qos,
        )
        cls.node.create_subscription(
            Odometry,
            f"{TOPIC_PREFIX}/localization",
            cls.localizations.append,
            qos_profile_sensor_data,
        )
        cls.node.create_subscription(TFMessage, "/tf", cls._on_tf, 100)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        if cls.process.poll() is None:
            cls.process.send_signal(signal.SIGINT)
            try:
                cls.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                cls.process.terminate()
                try:
                    cls.process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    cls.process.kill()
                    cls.process.wait(timeout=2.0)

    @classmethod
    def _on_tf(cls, message):
        cls.transforms.extend(
            transform
            for transform in message.transforms
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom"
        )

    @classmethod
    def spin_until(cls, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    @classmethod
    def publish_odometry(cls, x):
        message = Odometry()
        message.header.stamp = cls.node.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_test"
        message.pose.pose.position.x = x
        message.pose.pose.orientation.w = 1.0
        cls.odom_pub.publish(message)
        return message

    @classmethod
    def publish_observation(
        cls,
        stamp,
        sequence,
        map_base_x,
        accepted=True,
        status=RelocalizationObservation.STATUS_ACCEPTED,
        message="accepted",
    ):
        observation = RelocalizationObservation()
        observation.header.stamp = stamp
        observation.header.frame_id = "map"
        observation.child_frame_id = "base_test"
        observation.sequence = sequence
        observation.accepted = accepted
        observation.status = status
        observation.inlier_count = 50
        observation.source_points = 100
        observation.registration_error = 0.5
        observation.quality = 0.8
        observation.message = message
        observation.pose.pose.position.x = map_base_x
        observation.pose.pose.orientation.w = 1.0
        cls.observation_pub.publish(observation)

    def wait_for_status(self, predicate, start_index=0, timeout=3.0):
        return self.spin_until(
            lambda: any(predicate(status) for status in self.statuses[start_index:]),
            timeout=timeout,
        )

    def test_faults_epoch_and_recovery(self):
        self.assertTrue(
            self.spin_until(
                lambda: self.odom_pub.get_subscription_count() == 1
                and self.observation_pub.get_subscription_count() == 1
            )
        )

        first_odom = self.publish_odometry(0.0)
        self.assertTrue(self.spin_until(lambda: len(self.localizations) >= 1))
        second_odom = self.publish_odometry(0.1)
        self.assertTrue(self.spin_until(lambda: len(self.localizations) >= 2))

        self.publish_observation(second_odom.header.stamp, 1, 1.1)
        self.assertTrue(
            self.wait_for_status(
                lambda status: status.state == LocalizationStatus.STATE_TRACKING
                and status.epoch == 1
                and status.observation_sequence == 1
            )
        )
        self.assertTrue(
            self.spin_until(
                lambda: any(
                    abs(transform.transform.translation.x - 1.0) < 1e-3
                    for transform in self.transforms
                )
            )
        )

        status_index = len(self.statuses)
        self.publish_observation(first_odom.header.stamp, 99, 1.0)
        self.assertTrue(
            self.wait_for_status(
                lambda status: status.consecutive_rejections == 1,
                start_index=status_index,
            )
        )

        rejected_odom = self.publish_odometry(0.2)
        self.assertTrue(self.spin_until(lambda: len(self.localizations) >= 3))
        status_index = len(self.statuses)
        self.publish_observation(
            rejected_odom.header.stamp,
            2,
            1.2,
            accepted=False,
            status=RelocalizationObservation.STATUS_REJECTED,
            message="synthetic GICP rejection",
        )
        self.assertTrue(
            self.wait_for_status(
                lambda status: status.state == LocalizationStatus.STATE_DEGRADED
                and status.consecutive_rejections >= 2,
                start_index=status_index,
            )
        )

        recovery_odom = self.publish_odometry(0.3)
        self.assertTrue(self.spin_until(lambda: len(self.localizations) >= 4))
        self.publish_observation(recovery_odom.header.stamp, 3, 1.32)
        self.assertTrue(
            self.wait_for_status(
                lambda status: status.state == LocalizationStatus.STATE_TRACKING
                and status.epoch == 1
                and status.consecutive_rejections == 0
            )
        )
        self.assertAlmostEqual(
            self.transforms[-1].transform.translation.x, 1.0, places=3
        )

        false_match_odom = self.publish_odometry(0.4)
        self.assertTrue(self.spin_until(lambda: len(self.localizations) >= 5))
        status_index = len(self.statuses)
        self.publish_observation(false_match_odom.header.stamp, 4, 2.4)
        self.assertTrue(
            self.wait_for_status(
                lambda status: "plausibility gate" in status.message
                and status.epoch == 1,
                start_index=status_index,
            )
        )

        status_index = len(self.statuses)
        future_stamp = (self.node.get_clock().now() + Duration(seconds=0.12)).to_msg()
        self.publish_observation(future_stamp, 5, 1.4)
        self.assertTrue(
            self.wait_for_status(
                lambda status: "outside odometry history" in status.message,
                start_index=status_index,
            )
        )

        end_wait = time.monotonic() + 0.15
        while time.monotonic() < end_wait:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        jump_odom = self.publish_odometry(0.5)
        self.assertTrue(self.spin_until(lambda: len(self.localizations) >= 6))
        self.publish_observation(jump_odom.header.stamp, 6, 1.7)
        self.assertTrue(
            self.wait_for_status(
                lambda status: status.state == LocalizationStatus.STATE_TRACKING
                and status.epoch == 2
                and status.observation_sequence == 6
            )
        )
        self.assertTrue(
            self.spin_until(
                lambda: abs(self.transforms[-1].transform.translation.x - 1.2) < 1e-3
            )
        )

        status_index = len(self.statuses)
        self.assertTrue(
            self.wait_for_status(
                lambda status: status.state == LocalizationStatus.STATE_LOST
                and "odometry input stale" in status.message,
                start_index=status_index,
                timeout=1.0,
            )
        )

        restored_odom = self.publish_odometry(0.6)
        self.assertTrue(self.spin_until(lambda: len(self.localizations) >= 7))
        self.publish_observation(restored_odom.header.stamp, 7, 1.8)
        self.assertTrue(
            self.wait_for_status(
                lambda status: status.state == LocalizationStatus.STATE_TRACKING
                and status.epoch == 2
                and status.observation_sequence == 7
            )
        )
        self.assertEqual(self.localizations[-1].header.frame_id, "odom")
        self.assertAlmostEqual(
            self.localizations[-1].pose.pose.position.x, 0.6, places=6
        )

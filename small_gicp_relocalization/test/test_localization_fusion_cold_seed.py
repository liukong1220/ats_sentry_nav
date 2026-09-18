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
"""A parameter-seeded map->odom must not be reported as earned TRACKING."""

# Regression for the fail-open found in the Gazebo relocalization matrix: the
# observation-silence timers were armed by the FIRST ACCEPTED observation only,
# so a cold start on a wrong ``initial_map_to_odom`` stayed ``TRACKING`` forever.
# That blocked both autonomous LOST lattice search and the wider LOST correction
# budget, which made every deviation above the TRACKING gate unrecoverable.

import math
import signal
import subprocess
import time
import unittest

from ament_index_python.packages import get_package_prefix
from ats_navigation_interfaces.msg import LocalizationStatus
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data


TOPIC_PREFIX = "/localization_fusion_cold_seed_test"
DEGRADED_TIMEOUT_S = 0.6
LOST_TIMEOUT_S = 1.6


class TestLocalizationFusionColdSeed(unittest.TestCase):
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
            # The whole point of the fixture: map->odom exists from a parameter,
            # no accepted observation has ever arrived.
            "use_initial_map_to_odom": True,
            "initial_map_to_odom_x": 2.5,
            "initial_map_to_odom_y": -1.5,
            "initial_map_to_odom_yaw": 0.4,
            "publish_tf": False,
            "allow_initial_identity": False,
            "odom_timeout_s": 5.0,
            "observation_timeout_s": DEGRADED_TIMEOUT_S,
            "observation_lost_timeout_s": LOST_TIMEOUT_S,
            "relocalizing_hold_s": 0.0,
        }
        command = [executable, "--ros-args", "-r", "__node:=fusion_cold_seed_test"]
        for name, value in parameters.items():
            encoded = str(value).lower() if isinstance(value, bool) else str(value)
            command.extend(("-p", f"{name}:={encoded}"))
        cls.process = subprocess.Popen(
            command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

        rclpy.init()
        cls.node = rclpy.create_node("localization_fusion_cold_seed_driver")
        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.odom_pub = cls.node.create_publisher(
            Odometry, f"{TOPIC_PREFIX}/odometry", qos_profile_sensor_data
        )
        cls.statuses = []
        cls.node.create_subscription(
            LocalizationStatus,
            f"{TOPIC_PREFIX}/status",
            cls.statuses.append,
            transient_qos,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        if cls.process.poll() is None:
            cls.process.send_signal(signal.SIGINT)
            try:
                cls.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                cls.process.kill()
                cls.process.wait(timeout=2.0)

    @classmethod
    def publish_odometry(cls) -> None:
        message = Odometry()
        message.header.stamp = cls.node.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_test"
        message.pose.pose.orientation.w = 1.0
        cls.odom_pub.publish(message)

    def drive_until(self, predicate, timeout: float):
        """Keep odometry healthy while waiting, so only observation silence decides."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.publish_odometry()
            rclpy.spin_once(self.node, timeout_sec=0.05)
            match = next((s for s in reversed(self.statuses) if predicate(s)), None)
            if match is not None:
                return match
        return None

    def test_seeded_map_to_odom_degrades_then_reports_lost(self):
        degraded = self.drive_until(
            lambda s: s.state == LocalizationStatus.STATE_DEGRADED, timeout=8.0
        )
        self.assertIsNotNone(
            degraded,
            "a parameter-seeded map->odom without any accepted observation must "
            "reach DEGRADED once the observation timeout elapses",
        )
        self.assertIn(
            "awaiting first accepted relocalization observation", degraded.message
        )
        self.assertTrue(
            math.isfinite(degraded.observation_silence_sec),
            "observation silence must be armed at map-frame init, not left infinite",
        )
        self.assertGreaterEqual(degraded.observation_silence_sec, DEGRADED_TIMEOUT_S)

        lost = self.drive_until(
            lambda s: s.state == LocalizationStatus.STATE_LOST, timeout=8.0
        )
        self.assertIsNotNone(
            lost,
            "silence past observation_lost_timeout_s must report LOST so the "
            "autonomous lattice and the LOST correction budget can engage",
        )
        self.assertIn(
            "no accepted relocalization observation since map-frame init", lost.message
        )
        self.assertGreaterEqual(lost.observation_silence_sec, LOST_TIMEOUT_S)
        self.assertEqual(
            lost.epoch, 1, "a seeded map->odom stays epoch 1 until a real correction"
        )
        # NOTE: the first sub-timeout window still reports TRACKING. Tightening
        # that would make every cold boot start DEGRADED, which is a separate
        # downstream contract change and is deliberately not done here.


if __name__ == "__main__":
    unittest.main()

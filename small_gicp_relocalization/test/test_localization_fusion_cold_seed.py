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
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
import uuid

from ament_index_python.packages import get_package_prefix
from ats_navigation_interfaces.msg import LocalizationStatus
from ats_navigation_interfaces.msg import RelocalizationObservation
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
    use_seed = True
    odom_timeout = 5.0
    paused_clock = False

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
            "use_initial_map_to_odom": cls.use_seed,
            "initial_map_to_odom_x": 2.5,
            "initial_map_to_odom_y": -1.5,
            "initial_map_to_odom_yaw": 0.4,
            "publish_tf": False,
            "allow_initial_identity": False,
            "odom_timeout_s": cls.odom_timeout,
            "observation_timeout_s": DEGRADED_TIMEOUT_S,
            "observation_lost_timeout_s": LOST_TIMEOUT_S,
            "relocalizing_hold_s": 0.0,
            "use_sim_time": cls.paused_clock,
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
        cls.observation_pub = cls.node.create_publisher(
            RelocalizationObservation, f"{TOPIC_PREFIX}/observation", 10
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
        bootstrap = self.drive_until(
            lambda s: s.state == LocalizationStatus.STATE_BOOTSTRAP, timeout=4.0
        )
        self.assertIsNotNone(bootstrap, "valid odometry must expose unearned BOOTSTRAP")
        self.assertEqual(bootstrap.observation_sequence, 0)
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
            "observation silence must be armed by first valid odometry",
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
            "no accepted relocalization observation since first odometry", lost.message
        )
        self.assertGreaterEqual(lost.observation_silence_sec, LOST_TIMEOUT_S)
        self.assertEqual(
            lost.epoch, int(self.use_seed), "seed alone must not advance the epoch"
        )
        self.assertFalse(any(s.state == LocalizationStatus.STATE_TRACKING for s in self.statuses))
        self.assertFalse(any(s.state == LocalizationStatus.STATE_CONFIRMED for s in self.statuses))

        # LOST remains latched through odometry and rejected observations.
        start_index = len(self.statuses)
        observation = RelocalizationObservation()
        observation.header.stamp = self.node.get_clock().now().to_msg()
        observation.status = RelocalizationObservation.STATUS_REJECTED
        observation.sequence = 1
        self.observation_pub.publish(observation)
        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            self.publish_odometry()
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(self.statuses[start_index:])
        self.assertTrue(all(s.state == LocalizationStatus.STATE_LOST for s in self.statuses[start_index:]))

        # Pending carries zero accepted-quality; repeated pending cannot renew
        # its steady deadline or mutate the seed/epoch/accepted sequence.
        start_index = len(self.statuses)
        deadline = time.monotonic() + LOST_TIMEOUT_S + 1.0
        recovery_seen = False
        timeout_seen = False
        sequence = 2
        while time.monotonic() < deadline and not timeout_seen:
            self.publish_odometry()
            observation.header.stamp = self.node.get_clock().now().to_msg()
            observation.header.frame_id = "map"
            observation.child_frame_id = "base_test"
            observation.pose.pose.orientation.w = 1.0
            observation.inlier_count = 50
            observation.source_points = 100
            observation.registration_error = 0.5
            observation.quality = 0.0
            observation.status = RelocalizationObservation.STATUS_PENDING_CONFIRMATION
            observation.sequence = sequence
            sequence += 1
            self.observation_pub.publish(observation)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            for status in self.statuses[start_index:]:
                recovery_seen |= status.state == LocalizationStatus.STATE_RELOCALIZING
                timeout_seen |= (
                    recovery_seen and status.state == LocalizationStatus.STATE_LOST
                    and "recovery episode" in status.message
                )
                self.assertEqual(status.epoch, int(self.use_seed))
                self.assertEqual(status.observation_sequence, 0)
            start_index = len(self.statuses)
        self.assertTrue(recovery_seen)
        self.assertTrue(timeout_seen, "repeated pending must not renew recovery")


class TestLocalizationFusionUnseeded(TestLocalizationFusionColdSeed):
    use_seed = False


class TestLocalizationFusionNoInput(TestLocalizationFusionColdSeed):
    odom_timeout = 0.3
    paused_clock = True

    def test_seeded_map_to_odom_degrades_then_reports_lost(self):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if any(s.state == LocalizationStatus.STATE_LOST for s in self.statuses):
                break
        self.assertTrue(any(s.state == LocalizationStatus.STATE_LOST for s in self.statuses))
        self.assertTrue(all(s.state in (
            LocalizationStatus.STATE_UNINITIALIZED, LocalizationStatus.STATE_LOST
        ) for s in self.statuses))
        self.assertTrue(all(s.observation_sequence == 0 for s in self.statuses))


class TestRelocalizationStartupShutdown(unittest.TestCase):
    def test_map_loads_without_extrinsic_tf_and_paused_clock_sigint_exits_cleanly(self):
        executable = (
            get_package_prefix("small_gicp_relocalization")
            + "/lib/small_gicp_relocalization/small_gicp_relocalization_node"
        )
        with tempfile.TemporaryDirectory(prefix="relocalization_startup_") as directory:
            pcd = Path(directory) / "map.pcd"
            points = [f"{x} {y} {z}" for x in range(4) for y in range(4) for z in range(4)]
            pcd.write_text(
                "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                f"COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
                f"POINTS {len(points)}\nDATA ascii\n" + "\n".join(points) + "\n"
            )
            token = uuid.uuid4().hex
            command = [
                executable, "--ros-args",
                "-r", f"__node:=startup_shutdown_{token}",
                "-p", f"prior_pcd_file:={pcd}",
                "-p", f"map_frame:=prior_map_{token}",
                "-p", f"odom_frame:=registered_odom_{token}",
                "-p", f"robot_base_frame:=missing_body_{token}",
                "-p", "use_sim_time:=true",
            ]
            log_path = Path(directory) / "process.log"
            with log_path.open("w") as log:
                process = subprocess.Popen(command, stdout=log, stderr=log)
                try:
                    deadline = time.monotonic() + 10.0
                    while time.monotonic() < deadline:
                        output = log_path.read_text()
                        if "scan input gate:" in output:
                            break
                        self.assertIsNone(process.poll(), output)
                        time.sleep(0.05)
                    else:
                        self.fail("Map startup incorrectly depends on extrinsic TF: " + output)
                    self.assertIn("GICP coarse-fine ready:", output)
                    self.assertIn(f"Loaded global map: frame='prior_map_{token}' points=64", output)
                    self.assertIn(f"expected_frame='registered_odom_{token}'", output)
                    process.send_signal(signal.SIGINT)
                    self.assertEqual(
                        process.wait(timeout=5.0), 0,
                        "Startup shutdown must not abort: " + log_path.read_text(),
                    )
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.wait(timeout=5.0)


if __name__ == "__main__":
    unittest.main()

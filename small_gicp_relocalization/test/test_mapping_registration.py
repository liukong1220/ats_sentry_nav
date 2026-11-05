#!/usr/bin/env python3
# Copyright 2026
"""No prior: independent scan windows must match before fusion accepts."""
import signal
import subprocess
import tempfile
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from ats_navigation_interfaces.msg import IdentifiedOdometry, LocalizationStatus, RelocalizationObservation
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def test_growing_map_registration_requires_new_windows_and_fusion_commit():
    rclpy.init()
    node = rclpy.create_node("s3_growing_map_driver")
    processes, logs = [], []
    config = str(Path(get_package_share_directory("ats_sentry_bringup")) / "params/node_params.yaml")

    def start(executable, parameters):
        arguments = [str(Path(get_package_prefix("small_gicp_relocalization")) /
                     "lib/small_gicp_relocalization" / executable), "--ros-args", "--params-file", config]
        for value in parameters:
            arguments.extend(["-p", value])
        log = tempfile.TemporaryFile(mode="w+")
        logs.append(log)
        processes.append(subprocess.Popen(arguments, stdout=log, stderr=log))

    try:
        start("localization_fusion_node", ["use_identified_odometry:=true",
              "use_initial_map_to_odom:=true", "odom_timeout_s:=2.0",
              "observation_lost_timeout_s:=30.0"])
        start("small_gicp_relocalization_node", ["mapping_mode:=true",
              "require_odom_identity:=true", "prior_pcd_file:=/definitely/absent/s3.pcd"])
        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "odom", "gimbal_yaw_odom"
        t.transform.rotation.w = 1.0
        broadcaster = StaticTransformBroadcaster(node)
        broadcaster.sendTransform(t)
        odom_pub = node.create_publisher(IdentifiedOdometry, "/odometry_identified", 10)
        cloud_pub = node.create_publisher(PointCloud2, "/registered_scan", 10)
        observations, statuses = [], []
        node.create_subscription(RelocalizationObservation, "/relocalization_observation", observations.append, 20)
        node.create_subscription(LocalizationStatus, "/localization/status", statuses.append,
                                 QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Three non-parallel planes constrain the full rigid transform.
        points = []
        for a in range(-20, 21):
            for b in range(0, 31):
                points.extend([(2.0, a * 0.05, b * 0.05),
                               (a * 0.05, 2.5, b * 0.05),
                               (a * 0.05, b * 0.05, 0.0)])

        def drive(instance, seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                stamp = node.get_clock().now().to_msg()
                odom = IdentifiedOdometry(instance_id=instance)
                odom.header = Header(stamp=stamp, frame_id="odom")
                odom.odometry.header = odom.header
                odom.odometry.child_frame_id = "gimbal_yaw_odom"
                odom.odometry.pose.pose.orientation.w = 1.0
                odom_pub.publish(odom)
                cloud_pub.publish(point_cloud2.create_cloud_xyz32(odom.header, points))
                rclpy.spin_once(node, timeout_sec=0.02)
                time.sleep(0.05)

        drive(0, 1.0)
        assert not any(o.accepted for o in observations)
        drive(101, 7.0)
        assert any(o.accepted and o.odom_instance_id == 101 for o in observations)
        assert any(s.state == LocalizationStatus.STATE_TRACKING and s.observation_sequence > 0 for s in statuses)
        committed = [s for s in statuses if s.observation_sequence > 0]
        assert committed and all(s.map_to_odom_valid for s in committed)
        assert all(s.map_to_odom.header == s.header and s.map_to_odom.child_frame_id == "odom"
                   for s in committed)
        drive(202, 5.0)
        assert any(s.state == LocalizationStatus.STATE_LOST and s.odom_epoch == 2 for s in statuses)
        assert any(o.accepted and o.odom_instance_id == 202 and o.odom_epoch == 2 for o in observations)
        assert statuses[-1].odom_instance_id == 202
        assert processes[0].poll() is None and processes[1].poll() is None
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in processes:
            process.wait(timeout=10.0)
        for log in logs:
            log.seek(0)
            print(log.read())
            log.close()
        node.destroy_node()
        rclpy.shutdown()

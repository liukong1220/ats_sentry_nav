#!/usr/bin/env python3
# Copyright 2026
"""Real ROGMap + in-process planning publisher: frontend reset cannot resurrect an old map."""
import re
import signal
import subprocess
import tempfile
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from ats_navigation_interfaces.msg import LocalizationStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def test_epoch_reset_blocks_queued_clouds_and_mapping_ready_requires_identity():
    rclpy.init()
    node = rclpy.create_node("s3_epoch_route_driver")
    processes, logs = [], []

    def start(package, executable, arguments):
        log = tempfile.NamedTemporaryFile(mode="w+")
        logs.append(log)
        processes.append(subprocess.Popen(
            [str(Path(get_package_prefix(package)) / "lib" / package / executable),
             "--ros-args", *arguments], stdout=log, stderr=log))

    try:
        config = Path(get_package_share_directory("ats_rog_map")) / "config/rog_map_ground_planning_mujoco.yaml"
        start("ats_rog_map", "ats_rog_map_node", [
            "--params-file", str(config), "-p", "use_sim_time:=false",
            "-p", "debug_rate_hz:=0.0", "-p", "require_odom_identity:=true",
            "-p", "identity_timeout_sec:=2.0",
            # Parameters of the co-located planning publisher (ats_rog_map_adapter).
            "-p", "mapping_mode:=true",
            "-p", "allow_missing_static_map:=true", "-p", "require_terrain_inputs:=false",
            "-p", "projection_rate_hz:=10.0", "-p", "localization_status_timeout_sec:=2.0"])
        broadcaster = StaticTransformBroadcaster(node)
        transforms = []
        for parent, child, z in [("odom", "gimbal_yaw_odom", 0.0),
                                 ("gimbal_yaw_odom", "front_mid360", 0.2)]:
            t = TransformStamped()
            t.header.stamp = node.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.translation.z, t.transform.rotation.w = z, 1.0
            transforms.append(t)
        broadcaster.sendTransform(transforms)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        status_pub = node.create_publisher(LocalizationStatus, "/localization/status", qos)
        cloud_pub = node.create_publisher(PointCloud2, "/registered_scan", 10)
        odom_pub = node.create_publisher(Odometry, "/localization", 10)
        ready = []
        node.create_subscription(Bool, "/rog_map_adapter/ready", lambda m: ready.append(m.data), qos)

        def projection_ends():
            with open(logs[0].name, encoding="utf-8", errors="replace") as text:
                return [dict(re.findall(r"(\w+)=(\S+)", line))
                        for line in text if "P2 adapter projection end" in line]
        points = [(x / 100, y / 100, 0.0) for x in range(50, 120, 5) for y in range(-20, 25, 5)]
        points += [(1.05, 0.05, z / 100) for z in range(0, 55, 5)]
        boundary = node.get_clock().now().to_msg()

        def drive(instance, epoch, seconds=0.6, clouds=True, cloud_stamp=None, latched=False):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                stamp = node.get_clock().now().to_msg()
                s = LocalizationStatus()
                s.header.stamp = stamp
                s.header.frame_id = "map"
                s.state = LocalizationStatus.STATE_BOOTSTRAP
                s.epoch, s.odom_instance_id, s.odom_epoch = epoch, instance, epoch
                s.odom_epoch_start_stamp = boundary
                s.odom_identity_latched = latched
                s.odometry_silence_sec = 0.0
                status_pub.publish(s)
                if clouds:
                    h = Header(stamp=cloud_stamp or stamp, frame_id="odom")
                    cloud_pub.publish(point_cloud2.create_cloud_xyz32(h, points))
                    o = Odometry(header=h)
                    o.pose.pose.orientation.w = 1.0
                    odom_pub.publish(o)
                rclpy.spin_once(node, timeout_sec=0.02)
                time.sleep(0.02)

        drive(0, 0)
        assert ready and not any(ready)
        drive(10, 1, 2.0)
        assert ready[-1], "known fresh stable mapping epoch never became ready"
        old_stamp = node.get_clock().now().to_msg()
        boundary = node.get_clock().now().to_msg()
        ready.clear()
        drive(20, 2, 0.8, cloud_stamp=old_stamp)
        assert ready and not any(ready), "queued old cloud revived reset map"
        # The publisher asked the map for the new identity and got a not-ready
        # projection: the reset map has no new-epoch evidence yet.
        new_epoch = [e for e in projection_ends()
                     if (e["odom_instance_id"], e["odom_epoch"]) == ("20", "2")]
        assert new_epoch, "no projection was requested for the reset frontend epoch"
        assert new_epoch[0]["ready"] == "0"
        drive(20, 2, 1.2)
        assert ready[-1]
        ready.clear()
        drive(20, 2, 0.6, latched=True)
        assert ready and not any(ready), "concurrent source latch allowed planning"
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

#!/usr/bin/env python3
# Copyright 2026
"""Exercise the real save service and map transform, independently of shutdown."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from ats_rog_map_interfaces.srv import SaveOdomMap
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml


def test_explicit_save_transforms_static_cloud_and_counts():
    rclpy.init()
    node = rclpy.create_node("s3_save_service_test")
    log = tempfile.TemporaryFile(mode="w+")
    executable = (
        Path(get_package_prefix("ats_rog_map")) / "lib/ats_rog_map/ats_rog_map_node"
    )
    config = (
        Path(get_package_share_directory("ats_rog_map"))
        / "config/rog_map_ground_planning_mujoco.yaml"
    )
    process = subprocess.Popen(
        [
            str(executable),
            "--ros-args",
            "--params-file",
            str(config),
            "-p",
            "use_sim_time:=false",
            "-p",
            "debug_rate_hz:=0.0",
            "-p",
            "odom_layer.static_promotion_seconds:=0.15",
        ],
        stdout=log,
        stderr=log,
    )
    try:
        broadcaster = StaticTransformBroadcaster(node)
        transforms = []
        for parent, child, x, y, z, yaw in [
            ("map", "odom", 1.0, -2.0, 0.0, 0.3),
            ("odom", "gimbal_yaw_odom", 0.0, 0.0, 0.0, 0.0),
            ("gimbal_yaw_odom", "front_mid360", 0.0, 0.0, 0.2, 0.0),
        ]:
            t = TransformStamped()
            t.header.stamp = node.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = parent, child
            (
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ) = (x, y, z)
            t.transform.rotation.w = math.cos(yaw / 2)
            t.transform.rotation.z = math.sin(yaw / 2)
            transforms.append(t)
        broadcaster.sendTransform(transforms)
        client = node.create_client(SaveOdomMap, "/rog_map/save_map")
        assert client.wait_for_service(timeout_sec=15.0)
        clouds = node.create_publisher(PointCloud2, "/registered_scan", 10)
        odometry = node.create_publisher(Odometry, "/localization", 10)
        points = [
            (x / 100, y / 100, 0.0)
            for x in range(50, 120, 5)
            for y in range(-20, 25, 5)
        ]
        points += [(1.05, 0.05, z / 100) for z in range(0, 55, 5)]
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            stamp = node.get_clock().now().to_msg()
            msg = point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id="odom"), points
            )
            clouds.publish(msg)
            odom = Odometry()
            odom.header = msg.header
            odom.pose.pose.orientation.w = 1.0
            odometry.publish(odom)
            rclpy.spin_once(node, timeout_sec=0.05)
        with tempfile.TemporaryDirectory() as directory:
            prefix = str(Path(directory) / "map")
            future = client.call_async(SaveOdomMap.Request(output_prefix=prefix))
            rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
            assert future.done()
            result = future.result()
            assert result.success, result.message
            assert result.persistent_voxels > 0
            metadata = yaml.safe_load(Path(prefix + ".yaml").read_text())
            assert metadata["frame_id"] == "map"
            assert abs(metadata["origin"][2] - 0.3) < 1e-6
            pcd = Path(prefix + ".pcd").read_text()
            assert f"POINTS {result.persistent_voxels}\n" in pcd
            rows = [
                list(map(float, line.split()))
                for line in pcd.split("DATA ascii\n")[1].splitlines()
            ]
            assert max(p[2] for p in rows) - min(p[2] for p in rows) > 0.2
            assert metadata["persistent_cells"] == result.persistent_cells
            assert process.poll() is None  # Saved while the live node still runs.
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=10)
        log.seek(0)
        print(log.read())
        log.close()
        node.destroy_node()
        rclpy.shutdown()

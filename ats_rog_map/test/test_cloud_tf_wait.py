#!/usr/bin/env python3
# Copyright 2026
"""A delayed TF admits its cloud; an expired cloud cannot be revived by late TF."""
from pathlib import Path
import signal
import statistics
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, UInt64
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


@pytest.mark.parametrize("tf_timeout", [.08, 0.0])
def test_delayed_transform_deadline_and_retry(tf_timeout):
    rclpy.init()
    node = rclpy.create_node("s3_cloud_tf_driver")
    executable = Path(get_package_prefix("ats_rog_map")) / "lib/ats_rog_map/ats_rog_map_node"
    config = Path(get_package_share_directory("ats_rog_map")) / "config/rog_map_ground_planning_mujoco.yaml"
    log = tempfile.TemporaryFile(mode="w+")
    process = subprocess.Popen([str(executable), "--ros-args", "--params-file", str(config),
        "-p", "use_sim_time:=false", "-p", "debug_rate_hz:=0.0", "-p", f"tf_timeout_sec:={tf_timeout}"],
        stdout=log, stderr=log)
    updates = []
    clouds = node.create_publisher(PointCloud2, "/registered_scan", 10)
    odom = node.create_publisher(Odometry, "/localization", 10)
    subscription = node.create_subscription(UInt64, "/rog_map/odom_layer_updated",
                                            lambda msg: updates.append((msg.data, time.monotonic())), 10)
    dynamic = TransformBroadcaster(node)
    static = StaticTransformBroadcaster(node)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.001)

    def transform(stamp):
        msg = TransformStamped()
        msg.header.stamp, msg.header.frame_id = stamp, "odom"
        msg.child_frame_id, msg.transform.rotation.w = "gimbal_yaw_odom", 1.0
        dynamic.sendTransform(msg)

    def cloud(has_transform=False):
        header = Header(stamp=node.get_clock().now().to_msg(), frame_id="odom")
        msg = Odometry(header=header)
        msg.pose.pose.orientation.w = 1.0
        odom.publish(msg)
        if has_transform:
            transform(header.stamp)
            spin(.003)
        started = time.monotonic()
        clouds.publish(point_cloud2.create_cloud_xyz32(header, [(1.05, .05, .2), (1.05, .05, 0.)]))
        return header.stamp, started

    try:
        fixed = TransformStamped()
        fixed.header.frame_id, fixed.child_frame_id = "gimbal_yaw_odom", "front_mid360"
        fixed.transform.translation.z, fixed.transform.rotation.w = .2, 1.0
        static.sendTransform(fixed)
        deadline = time.monotonic() + 15
        while clouds.get_subscription_count() == 0 and time.monotonic() < deadline:
            spin(.02)
        assert clouds.get_subscription_count() > 0
        spin(.2)
        latencies = []
        for _ in range(20):
            before = len(updates)
            stamp, started = cloud(has_transform=(tf_timeout == 0))
            if tf_timeout > 0:
                spin(.003)
                transform(stamp)
            deadline = time.monotonic() + 1
            while len(updates) == before and time.monotonic() < deadline:
                spin(.001)
            assert len(updates) > before, "cloud with delayed TF was discarded"
            latencies.append((updates[-1][1] - started) * 1000)
            spin(.01)
        before = len(updates)
        expired_stamp, _ = cloud()
        spin(.12)
        assert len(updates) == before
        transform(expired_stamp)
        spin(.04)
        assert len(updates) == before, "late TF revived an expired cloud"
        stamp, _ = cloud(has_transform=(tf_timeout == 0))
        if tf_timeout > 0:
            transform(stamp)
        deadline = time.monotonic() + 1
        while len(updates) == before and time.monotonic() < deadline:
            spin(.001)
        assert len(updates) > before, "expired TF wait prevented a fresh cloud retry"
        print("delayed TF cloud-to-commit median_ms=", statistics.median(latencies),
              "samples_ms=", latencies)
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        log.seek(0)
        print(log.read())
        log.close()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()

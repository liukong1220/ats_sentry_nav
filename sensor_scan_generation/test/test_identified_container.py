#!/usr/bin/env python3
# Copyright 2026
"""The production MuJoCo component factory must preserve source identity."""
import signal
import subprocess
import sys
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ats_navigation_interfaces.msg import IdentifiedOdometry
from geometry_msgs.msg import TransformStamped
import rclpy
from rosgraph_msgs.msg import Clock
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def test_production_container_identity_route():
    process = subprocess.Popen(
        [sys.executable, str(Path(__file__).resolve()), "--container"]
    )
    rclpy.init()
    node = rclpy.create_node("s3_container_identity_driver")
    try:
        broadcaster = StaticTransformBroadcaster(node)
        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "gimbal_yaw_odom", "front_mid360"
        t.transform.rotation.w = 1.0
        broadcaster.sendTransform(t)
        publisher = node.create_publisher(
            IdentifiedOdometry, "aft_mapped_to_init_identified", 10
        )
        clock = node.create_publisher(Clock, "/clock", 10)
        received = []
        subscription = node.create_subscription(
            IdentifiedOdometry, "odometry_identified", received.append, 10
        )

        def send(identity, seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                message = IdentifiedOdometry()
                message.header.stamp = node.get_clock().now().to_msg()
                clock.publish(Clock(clock=message.header.stamp))
                message.header.frame_id = "camera_init"
                message.instance_id = identity
                message.odometry.header = message.header
                message.odometry.pose.pose.orientation.w = 1.0
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.05)

        send(101, 4.0)
        assert any(m.instance_id == 101 for m in received)
        send(202, 1.0)
        assert any(m.instance_id == 202 for m in received)
        received.clear()
        send(101, 0.5)
        send(0, 0.5)
        assert all(m.instance_id == 202 for m in received)
        assert all(
            m.header == m.odometry.header and m.header.frame_id == "odom"
            for m in received
        )
        node.destroy_subscription(subscription)
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10.0)
        node.destroy_node()
        rclpy.shutdown()


def run_container():
    from launch import LaunchDescription, LaunchService
    from ats_mujoco_sim.launch_args import SIM_LIO_ARGUMENTS
    from ats_mujoco_sim.lio_launch import lio_chain_nodes

    configuration = {
        **SIM_LIO_ARGUMENTS,
        "localization_source": "lio",
        "use_identified_odometry": "true",
        "base_footprint_height": "0.07",
        "params_file": str(
            Path(get_package_share_directory("ats_sentry_bringup"))
            / "params/node_params.yaml"
        ),
    }
    container = lio_chain_nodes(configuration)[2]
    service = LaunchService()
    service.include_launch_description(LaunchDescription([container]))
    service.run()


if __name__ == "__main__" and "--container" in sys.argv:
    run_container()

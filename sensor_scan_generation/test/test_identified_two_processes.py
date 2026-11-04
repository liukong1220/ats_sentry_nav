#!/usr/bin/env python3
# Copyright 2026
# Licensed under the Apache License, Version 2.0.
"""Check source identity across independent LOAM and scan processes."""

import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from ats_navigation_interfaces.msg import IdentifiedOdometry
from geometry_msgs.msg import TransformStamped
import rclpy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def test_identified_two_processes():
    loam_executable = get_package_prefix('loam_interface') + '/lib/loam_interface/loam_interface_node'
    scan_executable = (get_package_prefix('sensor_scan_generation')
                       + '/lib/sensor_scan_generation/sensor_scan_generation_node')
    processes = []
    rclpy.init()
    node = rclpy.create_node('identified_chain_driver')
    try:
        processes.append(subprocess.Popen([
            loam_executable, '--ros-args', '-p', 'state_estimation_topic:=chain_state',
            '-p', 'registered_scan_topic:=chain_cloud', '-p', 'odom_frame:=odom',
            '-p', 'base_frame:=chain_base', '-p', 'lidar_frame:=chain_lidar',
            '-p', 'use_identified_odometry:=true',
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        processes.append(subprocess.Popen([
            scan_executable, '--ros-args', '-p', 'odom_frame:=odom',
            '-p', 'lidar_frame:=chain_lidar', '-p', 'robot_base_frame:=chain_base',
            '-p', 'publish_tf:=false',
            '-p', 'use_identified_odometry:=true',
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = 'chain_base'
        transform.child_frame_id = 'chain_lidar'
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        publisher = node.create_publisher(IdentifiedOdometry, 'chain_state_identified', 10)
        received = []
        subscription = node.create_subscription(
            IdentifiedOdometry, 'odometry_identified', received.append, 10)

        def send(identity, duration):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                message = IdentifiedOdometry()
                message.header.stamp = node.get_clock().now().to_msg()
                message.header.frame_id = 'camera_init'
                message.instance_id = identity
                message.odometry.header = message.header
                message.odometry.pose.pose.orientation.w = 1.0
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.1)

        send(101, 2.0)
        assert any(message.instance_id == 101 for message in received)
        send(202, 1.0)
        assert any(message.instance_id == 202 for message in received)
        before = len(received)
        send(101, 0.5)
        send(0, 0.5)
        assert all(message.instance_id == 202 for message in received[before:])
        node.destroy_subscription(subscription)
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5.0)
        node.destroy_node()
        rclpy.shutdown()

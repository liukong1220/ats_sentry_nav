#!/usr/bin/env python3
# Copyright 2026
# Licensed under the Apache License, Version 2.0.
"""Exercise identity changes through the running fusion component."""

import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from ats_navigation_interfaces.msg import IdentifiedOdometry, LocalizationStatus
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def test_identified_restart_boundary():
    prefix = '/fusion_identified_test'
    executable = (get_package_prefix('small_gicp_relocalization')
                  + '/lib/small_gicp_relocalization/localization_fusion_node')
    process = subprocess.Popen([
        executable, '--ros-args', '-r', '__node:=fusion_identified_test',
        '-p', 'use_identified_odometry:=true',
        '-p', f'identified_odom_topic:={prefix}/odom',
        '-p', f'identity_monitor_topic:={prefix}/source',
        '-p', 'identity_retired_limit:=5',
        '-p', f'localization_topic:={prefix}/localization',
        '-p', f'status_topic:={prefix}/status',
        '-p', 'odom_frame:=odom', '-p', 'robot_base_frame:=base_test',
        '-p', 'publish_tf:=false', '-p', 'odom_timeout_s:=10.0',
        '-p', 'observation_lost_timeout_s:=30.0',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node('fusion_identified_driver')
    publisher = node.create_publisher(IdentifiedOdometry, f'{prefix}/odom', 10)
    source_publisher = node.create_publisher(IdentifiedOdometry, f'{prefix}/source', 10)
    statuses = []
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(LocalizationStatus, f'{prefix}/status',
                                            statuses.append, qos)

    def spin_for(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

    def send(instance, count=4):
        for _ in range(count):
            message = IdentifiedOdometry()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = 'odom'
            message.instance_id = instance
            message.odometry.header = message.header
            message.odometry.child_frame_id = 'base_test'
            message.odometry.pose.pose.orientation.w = 1.0
            publisher.publish(message)
            spin_for(0.08)

    try:
        spin_for(1.0)
        send(0)
        assert all(status.odom_instance_id == 0 for status in statuses)
        send(101)
        assert any(status.odom_instance_id == 101 and status.odom_epoch == 1
                   for status in statuses)
        send(202)
        assert any(status.odom_instance_id == 202 and status.odom_epoch == 2
                   and status.state == LocalizationStatus.STATE_LOST for status in statuses)
        send(101)
        send(0)
        spin_for(0.2)
        assert statuses[-1].odom_instance_id == 202
        assert statuses[-1].odom_epoch == 2
        for _ in range(7):
            message = IdentifiedOdometry()
            message.instance_id = 101
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = 'camera_init'
            message.odometry.header = message.header
            source_publisher.publish(message)
            spin_for(0.08)
        spin_for(0.2)
        assert any('concurrent odometry instances' in status.message for status in statuses)
        send(202)
        assert statuses[-1].state == LocalizationStatus.STATE_LOST
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5.0)

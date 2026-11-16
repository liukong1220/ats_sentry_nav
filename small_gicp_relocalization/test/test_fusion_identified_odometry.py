#!/usr/bin/env python3
# Copyright 2026
# Licensed under the Apache License, Version 2.0.
"""Exercise identity changes through the running fusion component."""

import signal
import subprocess
import time
import uuid

import pytest

from ament_index_python.packages import get_package_prefix
from ats_navigation_interfaces.msg import IdentifiedOdometry, LocalizationStatus
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


@pytest.mark.parametrize('invalid_pose_first', [False, True])
def test_late_first_identity_only_rearms_startup_timeout(invalid_pose_first):
    """A cold-start deadline is distinct from losing an established input.

    The first valid input can only earn BOOTSTRAP. A malformed input or a later
    stale episode must still require accepted relocalization evidence.
    """
    from ats_navigation_interfaces.msg import RelocalizationObservation

    prefix = '/fusion_late_first_' + uuid.uuid4().hex
    executable = (get_package_prefix('small_gicp_relocalization')
                  + '/lib/small_gicp_relocalization/localization_fusion_node')
    process = subprocess.Popen([
        executable, '--ros-args', '-r', '__node:=' + prefix[1:],
        '-p', 'use_identified_odometry:=true',
        '-p', f'identified_odom_topic:={prefix}/odom',
        '-p', f'identity_monitor_topic:={prefix}/source',
        '-p', f'localization_topic:={prefix}/localization',
        '-p', f'status_topic:={prefix}/status',
        '-p', f'observation_topic:={prefix}/observation',
        '-p', 'odom_frame:=odom', '-p', 'robot_base_frame:=base_test',
        '-p', 'publish_tf:=false', '-p', 'use_initial_map_to_odom:=true',
        '-p', 'initial_map_to_odom_x:=1.0', '-p', 'odom_timeout_s:=0.3',
        '-p', 'observation_timeout_s:=2.0', '-p', 'observation_lost_timeout_s:=4.0',
        '-p', 'relocalizing_hold_s:=0.0',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node(prefix[1:] + '_driver')
    publisher = node.create_publisher(IdentifiedOdometry, f'{prefix}/odom', 10)
    observations = node.create_publisher(
        RelocalizationObservation, f'{prefix}/observation', 10)
    statuses = []
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(
        LocalizationStatus, f'{prefix}/status', statuses.append, qos)

    def wait_for(predicate, timeout=3.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if statuses and predicate(statuses[-1]):
                return statuses[-1]
        pytest.fail('fusion status deadline: ' + (statuses[-1].message if statuses else 'none'))

    def send(count=1, *, instance=101, age=0.0, invalid=False):
        message = None
        for _ in range(count):
            message = IdentifiedOdometry()
            message.header.stamp = (node.get_clock().now()
                                    - rclpy.duration.Duration(seconds=age)).to_msg()
            message.header.frame_id = 'odom'
            message.instance_id = instance
            message.odometry.header = message.header
            message.odometry.child_frame_id = 'base_test'
            message.odometry.pose.pose.orientation.w = 1.0
            if invalid:
                message.odometry.pose.pose.position.x = float('nan')
            publisher.publish(message)
            end = time.monotonic() + 0.04
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.01)
        return message

    try:
        initial = wait_for(lambda s: s.state == s.STATE_LOST)
        assert initial.odom_instance_id == initial.odom_epoch == 0
        assert initial.message == 'initial odometry input timed out'
        send(4, instance=0)
        send(4, age=1.0)
        assert statuses[-1].odom_instance_id == 0
        assert statuses[-1].state == LocalizationStatus.STATE_LOST
        if invalid_pose_first:
            send(3, invalid=True)
            wait_for(lambda s: s.odom_instance_id == 101)
            send(4)
            assert statuses[-1].state == LocalizationStatus.STATE_LOST
            assert statuses[-1].observation_sequence == 0
            return

        message = send(3)
        bootstrap = wait_for(lambda s: s.state == s.STATE_BOOTSTRAP)
        assert bootstrap.odom_instance_id == 101 and bootstrap.odom_epoch == 1
        assert bootstrap.observation_sequence == 0
        assert all(s.state not in (s.STATE_TRACKING, s.STATE_CONFIRMED) for s in statuses)
        observation = RelocalizationObservation()
        observation.header.stamp = message.header.stamp
        observation.header.frame_id = 'map'
        observation.child_frame_id = 'base_test'
        observation.odom_instance_id = 101
        observation.odom_epoch = 1
        observation.pose.pose.position.x = 1.0
        observation.pose.pose.orientation.w = 1.0
        observation.quality = 0.8
        observation.inlier_count = observation.source_points = 20
        observation.registration_error = 0.0
        observation.sequence = 1
        observation.accepted = True
        observation.status = observation.STATUS_ACCEPTED
        observations.publish(observation)
        wait_for(lambda s: s.state == s.STATE_TRACKING)
        wait_for(lambda s: s.state == s.STATE_LOST)
        assert statuses[-1].message == 'odometry input stale'
        send(4)
        assert statuses[-1].state == LocalizationStatus.STATE_LOST
        assert statuses[-1].observation_sequence == 1
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.0)


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

#!/usr/bin/env python3
# Copyright 2026
"""Mapping's fast numerical source must not inherit the legacy 0.5 Hz cadence."""
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
from ats_navigation_interfaces.msg import LocalizationStatus
from ats_rog_map_interfaces.srv import GetRogMapProjection
import rclpy
import pytest
from std_msgs.msg import UInt64
from rclpy.qos import QoSProfile, DurabilityPolicy


@pytest.mark.parametrize("source_updates", [False, True])
def test_mapping_uses_its_own_projection_cadence(source_updates):
    rclpy.init()
    node = rclpy.create_node("s3_mapping_rate_driver")
    received = []

    def projection(request, response):
        received.append(time.monotonic())
        response.ready, response.stale, response.odom_layer_included = True, False, True
        response.odom_instance_id, response.odom_epoch = 101, 1
        response.generation = len(received)
        grid = response.occupancy_grid
        grid.header.frame_id, grid.header.stamp = "odom", node.get_clock().now().to_msg()
        grid.info.resolution, grid.info.width, grid.info.height = 0.1, 10, 10
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 100
        response.signed_distance, response.gradient_x, response.gradient_y = [1.] * 100, [0.] * 100, [0.] * 100
        return response

    service = node.create_service(GetRogMapProjection, "/rog_map/get_ground_projection", projection)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(LocalizationStatus, "/localization/status", qos)
    updates = node.create_publisher(UInt64, "/rog_map/odom_layer_updated", 1)
    last_update = 0.0
    log = tempfile.TemporaryFile(mode="w+")
    executable = Path(get_package_prefix("ats_rog_map_adapter")) / "lib/ats_rog_map_adapter/ats_rog_map_adapter_node"
    process = subprocess.Popen([str(executable), "--ros-args", "-p", "mapping_mode:=true",
        "-p", "projection_rate_hz:=0.5",
        "-p", f"mapping_projection_rate_hz:={0.5 if source_updates else 40.0}", "-p", "allow_missing_static_map:=true",
        "-p", "require_terrain_inputs:=false"], stdout=log, stderr=log)
    try:
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            status = LocalizationStatus()
            status.header.stamp = node.get_clock().now().to_msg()
            status.state = status.STATE_BOOTSTRAP
            status.epoch, status.odom_instance_id, status.odom_epoch = 1, 101, 1
            status.odom_epoch_start_stamp = status.header.stamp
            publisher.publish(status)
            if source_updates and time.monotonic() - last_update >= 0.05:
                updates.publish(UInt64(data=len(received) + 1))
                last_update = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.01)
        minimum = 40 if source_updates else 100
        assert len(received) >= minimum, f"mapping still uses slow projection cadence: {received}"
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        log.seek(0)
        print(log.read())
        log.close()
        node.destroy_node()
        rclpy.shutdown()

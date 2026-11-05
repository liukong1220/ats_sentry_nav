#!/usr/bin/env python3
# Copyright 2026
"""The in-process projection feeds the ground-relative global layer in an odom window."""
import re
import signal
import subprocess
import tempfile
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from ats_navigation_interfaces.msg import PlanningMapSnapshot
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def test_ground_relative_global_layer_is_a_real_projection_source():
    rclpy.init()
    node = rclpy.create_node("s3_layer_projection_driver")
    log = tempfile.NamedTemporaryFile(mode="w+")
    executable = Path(get_package_prefix("ats_rog_map")) / "lib/ats_rog_map/ats_rog_map_node"
    config = Path(get_package_share_directory("ats_rog_map")) / "config/rog_map_ground_planning_mujoco.yaml"
    process = subprocess.Popen([str(executable), "--ros-args", "--params-file", str(config),
        "-p", "use_sim_time:=false", "-p", "debug_rate_hz:=0.0",
        "-p", "enable_test_fault_injection:=true",
        "-p", "odom_layer.origin_x:=-20.0", "-p", "odom_layer.origin_y:=-15.0",
        # The co-located planning publisher (node ats_rog_map_adapter) reads the
        # odom layer in its own projection frame; no prior map, terrain or
        # localization status exists in this fixture.
        "-p", "use_odom_layer:=true", "-p", "output_in_projection_frame:=true",
        "-p", "allow_missing_static_map:=true", "-p", "require_terrain_inputs:=false",
        "-p", "require_localization_status:=false", "-p", "projection_rate_hz:=5.0",
        "-p", "projection_min_height:=0.1", "-p", "projection_max_height:=0.8",
        "-p", "planning_resolution:=0.1"],
        stdout=log, stderr=log)
    try:
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
        clouds = node.create_publisher(PointCloud2, "/registered_scan", 10)
        odom = node.create_publisher(Odometry, "/localization", 10)
        snapshots = []
        node.create_subscription(
            PlanningMapSnapshot, "/rog_map_adapter/planning_snapshot", snapshots.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        def projection_evidence():
            with open(log.name, encoding="utf-8", errors="replace") as text:
                return [dict(re.findall(r"(\w+)=(\S+)", line))
                        for line in text if "P2 rog numeric projection" in line]
        floor = [(x / 100, y / 100, 0.0) for x in range(45, 210, 5) for y in range(-30, 35, 5)]
        points = floor + [(1.05, 0.05, z / 100) for z in range(0, 55, 5)]
        sensor_stamps = set()
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            header = Header(stamp=node.get_clock().now().to_msg(), frame_id="odom")
            sensor_stamps.add((header.stamp.sec, header.stamp.nanosec))
            clouds.publish(point_cloud2.create_cloud_xyz32(header, points))
            o = Odometry(header=header)
            o.pose.pose.orientation.w = 1.0
            odom.publish(o)
            rclpy.spin_once(node, timeout_sec=0.03)
            time.sleep(0.02)
        snapshots.clear()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not any(s.ready for s in snapshots):
            rclpy.spin_once(node, timeout_sec=0.05)
        ready = [s for s in snapshots if s.ready]
        assert ready, "no ready planning snapshot from the in-process odom layer projection"
        grid = ready[-1]
        assert grid.header.frame_id == "odom"
        assert (grid.source_stamp.sec, grid.source_stamp.nanosec) in sensor_stamps, (
            "global layer must retain the integrated scan stamp, not processing wall/sim time")
        assert (grid.info.width, grid.info.height) == (120, 120)
        assert len(grid.signed_distance_m) == len(grid.occupancy)
        included = [e for e in (dict(re.findall(r"(\w+)=(\S+)", line))
                                for line in open(log.name, encoding="utf-8", errors="replace")
                                if "P2 adapter projection end" in line)
                    if e.get("source_generation") == str(grid.source_generation)]
        assert included and included[-1]["odom_layer_included"] == "1"

        def cell(x, y):
            mx = int((x - grid.info.origin.position.x) / grid.info.resolution)
            my = int((y - grid.info.origin.position.y) / grid.info.resolution)
            return grid.occupancy[my * grid.info.width + mx]

        assert cell(1.05, 0.05) == 100, "wall evidence never entered the numerical projection"
        assert cell(0.55, 0.05) == 0, "observed floor did not become known free"
        assert cell(-4.0, -4.0) == -1, "unobserved global cells became free"
        parameters = node.create_client(SetParameters, "/ats_rog_map/set_parameters")
        assert parameters.wait_for_service(timeout_sec=5.0)
        reset = parameters.call_async(SetParameters.Request(parameters=[
            Parameter("test_reset_to_unknown", value=True).to_parameter_msg()]))
        rclpy.spin_until_future_complete(node, reset, timeout_sec=5.0)
        assert reset.done() and all(result.successful for result in reset.result().results)
        for _ in range(8):
            header.stamp = node.get_clock().now().to_msg()
            clouds.publish(point_cloud2.create_cloud_xyz32(header, points))
            odom.publish(Odometry(header=header))
            rclpy.spin_once(node, timeout_sec=0.03)
            time.sleep(0.03)
        # The raw projection itself (before fusion) must be strictly all-unknown:
        # every cell -1 and every numeric sample NaN, at a newer generation.
        deadline = time.monotonic() + 5.0
        evidence = []
        while time.monotonic() < deadline and not evidence:
            rclpy.spin_once(node, timeout_sec=0.05)
            evidence = [e for e in projection_evidence()
                        if int(e["generation"]) > grid.source_generation]
        assert evidence, "no projection after the reset fixture"
        assert evidence[-1]["all_unknown"] == "1", (
            "all-unknown fixture retained old global-layer evidence: %s" % evidence[-1])
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10.0)
        log.seek(0)
        print(log.read())
        log.close()
        node.destroy_node()
        rclpy.shutdown()

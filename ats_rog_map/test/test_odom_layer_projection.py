#!/usr/bin/env python3
# Copyright 2026
"""Production service returns the ground-relative global layer in an odom window."""
import signal
import math
import subprocess
import tempfile
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from ats_rog_map_interfaces.srv import GetRogMapProjection
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def test_ground_relative_global_layer_is_a_real_projection_source():
    rclpy.init()
    node = rclpy.create_node("s3_layer_projection_driver")
    log = tempfile.TemporaryFile(mode="w+")
    executable = Path(get_package_prefix("ats_rog_map")) / "lib/ats_rog_map/ats_rog_map_node"
    config = Path(get_package_share_directory("ats_rog_map")) / "config/rog_map_ground_planning_mujoco.yaml"
    process = subprocess.Popen([str(executable), "--ros-args", "--params-file", str(config),
        "-p", "use_sim_time:=false", "-p", "debug_rate_hz:=0.0",
        "-p", "enable_test_fault_injection:=true",
        "-p", "odom_layer.origin_x:=-20.0", "-p", "odom_layer.origin_y:=-15.0"],
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
        client = node.create_client(GetRogMapProjection, "/rog_map/get_ground_projection")
        assert client.wait_for_service(timeout_sec=15.0)
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
        request = GetRogMapProjection.Request(
            min_height=0.1, max_height=0.8, resolution=0.1, use_odom_layer=True)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        assert future.done()
        response = future.result()
        assert response.ready and not response.stale
        assert response.odom_layer_included
        grid = response.occupancy_grid
        assert grid.header.frame_id == "odom"
        assert (grid.header.stamp.sec, grid.header.stamp.nanosec) in sensor_stamps, (
            "global layer must retain the integrated scan stamp, not processing wall/sim time")
        assert (grid.info.width, grid.info.height) == (120, 120)
        assert len(response.signed_distance) == len(grid.data)

        def cell(x, y):
            mx = int((x - grid.info.origin.position.x) / grid.info.resolution)
            my = int((y - grid.info.origin.position.y) / grid.info.resolution)
            return grid.data[my * grid.info.width + mx]

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
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        assert future.done()
        reset_response = future.result()
        assert reset_response.generation > response.generation
        assert all(value == -1 for value in reset_response.occupancy_grid.data), (
            "all-unknown fixture retained old global-layer evidence")
        assert all(math.isnan(value) for value in reset_response.signed_distance)
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10.0)
        log.seek(0)
        print(log.read())
        log.close()
        node.destroy_node()
        rclpy.shutdown()

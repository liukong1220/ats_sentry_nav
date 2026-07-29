#!/usr/bin/env python3
"""实车 Nav2-free profile 的静态墙 /map 发布器（不启动任何 Nav2 节点）。

P6 零.1 要求 `launch_nav2:=false` 的运行图中不得出现 Nav2 节点与 lifecycle
manager，但 `/rc_esdf/planning_grid` 的两个可能属主都必须拿到静态墙：
- `ats_rog_map_adapter_node` 的 `static_map_topic` 默认 `/map`，缺图时投影直接返回；
- `rc_esdf_map_node` 的 `static_map_topic_` 同为 `/map`，缺图时融合直接返回。

因此这里保持 `nav2_map_server` 的对外数据契约不变：同一个话题、同一个
`map` frame、同样的 transient-local 可靠 QoS、同样的 trinary 占据语义
（100 占据 / 0 空闲 / -1 未知），只是换成一个自研节点，把 Nav2 从实车运行图里
彻底移除。与 `map_server` 互斥启动，保证 `/map` 始终只有一个发布者。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from nav_msgs.msg import OccupancyGrid
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
import yaml


def load_static_occupancy_grid(map_yaml_file: Path, frame_id: str) -> OccupancyGrid:
    """按 Nav2 trinary map 约定解析 ROS map YAML/PGM。"""
    map_yaml_file = map_yaml_file.expanduser().resolve()
    with map_yaml_file.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    if not isinstance(metadata, dict):
        raise ValueError(f"Map YAML is not a mapping: {map_yaml_file}")

    image_name = metadata.get("image")
    origin = metadata.get("origin")
    if not isinstance(image_name, str) or not image_name:
        raise ValueError(f"Map YAML has no image path: {map_yaml_file}")
    if not isinstance(origin, (list, tuple)) or len(origin) != 3:
        raise ValueError(f"Map YAML requires a three-element origin: {map_yaml_file}")

    resolution = float(metadata.get("resolution", 0.0))
    occupied_threshold = float(metadata.get("occupied_thresh", -1.0))
    free_threshold = float(metadata.get("free_thresh", -1.0))
    negate = bool(int(metadata.get("negate", 0)))
    if resolution <= 0.0 or not 0.0 <= free_threshold <= occupied_threshold <= 1.0:
        raise ValueError(f"Map YAML has invalid resolution or thresholds: {map_yaml_file}")

    image_path = Path(image_name)
    if not image_path.is_absolute():
        image_path = map_yaml_file.parent / image_path
    image = np.asarray(Image.open(image_path).convert("L"), dtype=np.uint8)
    if image.ndim != 2 or image.size == 0:
        raise ValueError(f"Map image must be a non-empty grayscale image: {image_path}")

    # ROS OccupancyGrid 以左下角为原点，PGM 行从左上角开始；必须翻转，
    # 否则静态墙坐标与 adapter/rc_esdf 的保守聚合会整体上下镜像。
    probability = image.astype(np.float64) / 255.0
    if not negate:
        probability = 1.0 - probability
    occupancy = np.full(image.shape, -1, dtype=np.int8)
    occupancy[probability > occupied_threshold] = 100
    occupancy[probability < free_threshold] = 0
    occupancy = np.flipud(occupancy)

    grid = OccupancyGrid()
    grid.header.frame_id = frame_id
    grid.info.resolution = resolution
    grid.info.width = int(image.shape[1])
    grid.info.height = int(image.shape[0])
    grid.info.origin.position.x = float(origin[0])
    grid.info.origin.position.y = float(origin[1])
    grid.info.origin.orientation.z = float(np.sin(0.5 * float(origin[2])))
    grid.info.origin.orientation.w = float(np.cos(0.5 * float(origin[2])))
    grid.data = occupancy.reshape(-1).tolist()
    return grid


class StaticMapPublisher(Node):
    """为迟到订阅者发布一份持久静态墙样本。"""

    def __init__(self) -> None:
        super().__init__("static_map_publisher")
        map_yaml_file = Path(
            self.declare_parameter("map_yaml_file", "").get_parameter_value().string_value
        )
        map_topic = self.declare_parameter("map_topic", "/map").value
        frame_id = self.declare_parameter("frame_id", "map").value
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(OccupancyGrid, map_topic, qos)
        try:
            self._map = load_static_occupancy_grid(map_yaml_file, frame_id)
        except (OSError, ValueError, yaml.YAMLError) as error:
            # 静态墙缺失是停止条件，不允许降级成"空图继续跑"。
            self.get_logger().fatal(f"静态墙地图加载失败，实车链不得继续：{error}")
            raise
        self._map.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._map)
        self.get_logger().info(
            f"已发布持久静态墙 /map {self._map.info.width}x{self._map.info.height} @ "
            f"{self._map.info.resolution:.9f} m/cell, frame={frame_id}, topic={map_topic}"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StaticMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

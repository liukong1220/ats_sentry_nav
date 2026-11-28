import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_nav_defaults(pkg_dir):
    cfg_path = os.path.join(pkg_dir, "config", "launch_defaults.yaml")
    with open(cfg_path, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    merged = {}
    for section in ("common", "localization", "auto_mapping"):
        for k, v in (raw.get(section) or {}).items():
            merged[str(k)] = str(v)
    return merged, (raw.get("lidar_mount") or {})


def generate_launch_description():
    bringup_dir = get_package_share_directory("ats_nav_bringup")
    defaults, lidar_mount_cfg = _load_nav_defaults(bringup_dir)
    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_identified_odometry = LaunchConfiguration("use_identified_odometry")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    launch_small_gicp_relocalization = LaunchConfiguration(
        "launch_small_gicp_relocalization"
    )
    launch_localization_fusion = LaunchConfiguration("launch_localization_fusion")
    lidar_mount_mode = LaunchConfiguration("lidar_mount_mode")
    gimbal_use_base_footprint_odom = LaunchConfiguration("gimbal_use_base_footprint_odom")
    log_level = LaunchConfiguration("log_level")
    slam = LaunchConfiguration("slam")

    active_nav_frame = PythonExpression([
        "'base_footprint' if ('", lidar_mount_mode, "'.lower() == 'gimbal' and '",
        gimbal_use_base_footprint_odom, "'.lower() == 'true') else 'gimbal_yaw_odom'",
    ])

    declarations = [
        DeclareLaunchArgument("use_identified_odometry", default_value=defaults["use_identified_odometry"]),
        DeclareLaunchArgument("namespace", default_value=defaults["namespace"]),
        DeclareLaunchArgument("map", description="Static map YAML path"),
        DeclareLaunchArgument("use_sim_time", default_value=defaults["use_sim_time"]),
        DeclareLaunchArgument("prior_pcd_file", default_value=""),
        DeclareLaunchArgument("params_file", description="Root-owned node parameter YAML path"),
        DeclareLaunchArgument("use_respawn", default_value=defaults["use_respawn"]),
        DeclareLaunchArgument("launch_small_gicp_relocalization", default_value=defaults["launch_small_gicp_relocalization"]),
        DeclareLaunchArgument("launch_localization_fusion", default_value=defaults["launch_localization_fusion"]),
        DeclareLaunchArgument(
            "lidar_mount_mode",
            default_value="chassis",
            description="LiDAR mount mode: 'chassis' (fixed on chassis) or 'gimbal' (mounted on big yaw).",
        ),
        DeclareLaunchArgument(
            "gimbal_use_base_footprint_odom",
            default_value="True",
            description="In gimbal mode, output odometry/localization in base_footprint (chassis frame).",
        ),
        DeclareLaunchArgument("log_level", default_value=defaults["log_level"]),
        DeclareLaunchArgument("slam", default_value=defaults["slam"]),
    ]

    prior_preflight = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, "launch", "prior_pcd_preflight.launch.py")),
        launch_arguments={
            "world": LaunchConfiguration("world", default=defaults["prior_preflight_world"]),
            "prior_pcd_file": prior_pcd_file,
            "launch_small_gicp_relocalization": launch_small_gicp_relocalization,
        }.items(),
        condition=UnlessCondition(slam),
    )
    static_map_publisher = Node(
        package="ats_nav_bringup",
        executable="static_map_publisher.py",
        name="static_map_publisher",
        condition=UnlessCondition(slam),
        namespace=namespace,
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "map_yaml_file": map_yaml_file,
            "map_topic": "/map",
            "frame_id": "map",
        }],
        arguments=["--ros-args", "--log-level", log_level],
    )

    def _make_point_lio(context):
        mode = lidar_mount_mode.perform(context).strip().lower()
        if mode not in ("chassis", "gimbal"):
            raise ValueError(
                f"lidar_mount_mode must be 'chassis' or 'gimbal', got {mode!r}"
            )
        gravity = [float(v) for v in lidar_mount_cfg[mode]["point_lio_gravity"]]
        return [
            Node(
                package="point_lio",
                executable="pointlio_mapping",
                name="point_lio",
                namespace=namespace,
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file, {
                    "use_sim_time": use_sim_time,
                    "mapping.gravity": gravity,
                }],
                arguments=["--ros-args", "--log-level", log_level],
            )
        ]

    point_lio = OpaqueFunction(function=_make_point_lio)
    static_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_map_to_odom",
        namespace=namespace,
        condition=IfCondition(PythonExpression([
            "'", launch_localization_fusion, "'.lower() == 'false' and '",
            launch_small_gicp_relocalization, "'.lower() == 'false' and '",
            slam, "'.lower() == 'false'",
        ])),
        output="screen",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0", "--roll", "0.0",
            "--pitch", "0.0", "--yaw", "0.0", "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
    )
    relocalization = Node(
        package="small_gicp_relocalization",
        executable="small_gicp_relocalization_node",
        name="small_gicp_relocalization",
        namespace=namespace,
        condition=IfCondition(PythonExpression([
            "'", slam, "'.lower() == 'true' or '",
            launch_small_gicp_relocalization, "'.lower() == 'true'",
        ])),
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[params_file, {
            "use_sim_time": use_sim_time,
            "prior_pcd_file": prior_pcd_file,
            "robot_base_frame": active_nav_frame,
            "mapping_mode": ParameterValue(slam, value_type=bool),
            "require_odom_identity": ParameterValue(use_identified_odometry, value_type=bool),
            "publish_tf": PythonExpression(["not ", launch_localization_fusion]),
        }],
        arguments=["--ros-args", "--log-level", log_level],
    )
    fusion = Node(
        package="small_gicp_relocalization",
        executable="localization_fusion_node",
        name="localization_fusion",
        namespace=namespace,
        condition=IfCondition(launch_localization_fusion),
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[params_file, {
            "use_sim_time": use_sim_time,
            "robot_base_frame": active_nav_frame,
            "use_identified_odometry": use_identified_odometry,
            "identified_odom_topic": "odometry_identified",
            "identity_monitor_topic": "aft_mapped_to_init_identified",
            "use_initial_map_to_odom": ParameterValue(slam, value_type=bool),
        }],
        arguments=["--ros-args", "--log-level", log_level],
    )

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))
    ld.add_action(SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"))
    for declaration in declarations:
        ld.add_action(declaration)
    ld.add_action(prior_preflight)
    ld.add_action(static_map_publisher)
    ld.add_action(static_map_to_odom)
    ld.add_action(GroupAction(actions=[point_lio, relocalization, fusion]))
    return ld

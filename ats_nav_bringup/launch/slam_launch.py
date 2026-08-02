import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _filtered_ld_library_path():
    blocked_entries = {"/opt/MVS/lib/64", "/opt/MVS/lib/32"}
    raw_value = os.environ.get("LD_LIBRARY_PATH", "")
    filtered_parts = []
    for part in raw_value.split(":"):
        normalized = part.strip()
        if not normalized or normalized in blocked_entries:
            continue
        if normalized not in filtered_parts:
            filtered_parts.append(normalized)
    return ":".join(filtered_parts)


def generate_launch_description():
    # Getting directories and launch-files
    bringup_dir = get_package_share_directory("ats_nav_bringup")

    # Input parameters declaration
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    sanitize_ld_library_path = SetEnvironmentVariable(
        "LD_LIBRARY_PATH", _filtered_ld_library_path()
    )

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file", description="Root-owned ROS 2 parameter YAML path"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation clock if true",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    start_pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("cloud_in", "terrain_map_ext"),
            ("scan", "obstacle_scan"),
        ],
    )

    start_sync_slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
            ("/map_updates", "map_updates"),
        ],
    )

    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
            {"prior_pcd.enable": False},
            {"pcd_save.pcd_save_en": True},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )
    
    start_static_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_map2odom",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    # start_static_transform_baselink2footprint_node = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher_baselink2footprint",
    #     output="screen",
    #     arguments=[
    #         "--x",
    #         "0.0",
    #         "--y",
    #         "0.0",
    #         "--z",
    #         "0.0",
    #         "--roll",
    #         "0.0",
    #         "--pitch",
    #         "0.0",
    #         "--yaw",
    #         "0.0",
    #         "--frame-id",
    #         "base_footprint",
    #         "--child-frame-id",
    #         "base_link",
    #     ],
    # )
    
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(sanitize_ld_library_path)

    ld.add_action(start_pointcloud_to_laserscan_node)
    ld.add_action(start_sync_slam_toolbox_node)
    ld.add_action(start_point_lio_node)
    ld.add_action(start_static_transform_node)

    return ld

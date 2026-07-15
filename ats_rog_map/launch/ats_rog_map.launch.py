from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="gimbal_yaw_odom"),
        DeclareLaunchArgument("sensor_frame", default_value="front_mid360"),
        DeclareLaunchArgument("odom_topic", default_value="/localization"),
        DeclareLaunchArgument("cloud_topic", default_value="/registered_scan"),
        DeclareLaunchArgument("self_filter_radius", default_value="0.45"),
        DeclareLaunchArgument(
            "map_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ats_rog_map"), "config", "rog_map.yaml",
            ]),
        ),
        Node(
            package="ats_rog_map",
            executable="ats_rog_map_node",
            name="ats_rog_map",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map_frame": LaunchConfiguration("map_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "sensor_frame": LaunchConfiguration("sensor_frame"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "cloud_topic": LaunchConfiguration("cloud_topic"),
                "self_filter_radius": LaunchConfiguration("self_filter_radius"),
                "map_config_file": LaunchConfiguration("map_config_file"),
            }],
        ),
    ])

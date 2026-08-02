import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("ats_nav_bringup")
    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    launch_small_gicp_relocalization = LaunchConfiguration(
        "launch_small_gicp_relocalization"
    )
    launch_localization_fusion = LaunchConfiguration("launch_localization_fusion")
    log_level = LaunchConfiguration("log_level")

    declarations = [
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("map", description="Static map YAML path"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("prior_pcd_file", default_value=""),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(bringup_dir, "config", "reality", "nav2_params.yaml"),
        ),
        DeclareLaunchArgument("use_respawn", default_value="false"),
        DeclareLaunchArgument("launch_small_gicp_relocalization", default_value="True"),
        DeclareLaunchArgument("launch_localization_fusion", default_value="True"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    static_map_publisher = Node(
        package="ats_nav_bringup",
        executable="static_map_publisher.py",
        name="static_map_publisher",
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
    point_lio = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[params_file, {
            "use_sim_time": use_sim_time,
            "prior_pcd.prior_pcd_map_path": prior_pcd_file,
        }],
        arguments=["--ros-args", "--log-level", log_level],
    )
    static_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_map_to_odom",
        namespace=namespace,
        condition=IfCondition(PythonExpression(["not ", launch_small_gicp_relocalization])),
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
        condition=IfCondition(launch_small_gicp_relocalization),
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[params_file, {
            "use_sim_time": use_sim_time,
            "prior_pcd_file": prior_pcd_file,
            "publish_tf": PythonExpression(["not ", launch_localization_fusion]),
        }],
        arguments=["--ros-args", "--log-level", log_level],
    )
    fusion = Node(
        package="small_gicp_relocalization",
        executable="localization_fusion_node",
        name="localization_fusion",
        namespace=namespace,
        condition=IfCondition(PythonExpression([
            launch_small_gicp_relocalization, " and ", launch_localization_fusion,
        ])),
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))
    ld.add_action(SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"))
    for declaration in declarations:
        ld.add_action(declaration)
    ld.add_action(static_map_publisher)
    ld.add_action(static_map_to_odom)
    ld.add_action(GroupAction(actions=[point_lio, relocalization, fusion]))
    return ld

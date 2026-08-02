import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory("ats_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")
    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_respawn = LaunchConfiguration("use_respawn")
    launch_small_gicp_relocalization = LaunchConfiguration("launch_small_gicp_relocalization")
    launch_localization_fusion = LaunchConfiguration("launch_localization_fusion")
    launch_fake_vel_transform = LaunchConfiguration("launch_fake_vel_transform")
    launch_chassis_vel_transform = LaunchConfiguration("launch_chassis_vel_transform")
    fake_vel_output_topic = LaunchConfiguration("fake_vel_output_topic")
    chassis_vel_input_topic = LaunchConfiguration("chassis_vel_input_topic")
    mpc_cmd_vel_topic = LaunchConfiguration("mpc_cmd_vel_topic")
    require_gimbal_status = LaunchConfiguration("require_gimbal_status")
    log_level = LaunchConfiguration("log_level")

    declarations = [
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("map", description="Static map YAML path"),
        DeclareLaunchArgument("prior_pcd_file", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", description="Root-owned node parameter YAML path"),
        DeclareLaunchArgument("use_robot_state_pub", default_value="False"),
        DeclareLaunchArgument("use_respawn", default_value="False"),
        DeclareLaunchArgument("launch_small_gicp_relocalization", default_value="True"),
        DeclareLaunchArgument("launch_localization_fusion", default_value="True"),
        DeclareLaunchArgument("launch_fake_vel_transform", default_value="True"),
        DeclareLaunchArgument("launch_chassis_vel_transform", default_value="True"),
        DeclareLaunchArgument("fake_vel_output_topic", default_value="cmd_vel_gimbal_yaw_odom"),
        DeclareLaunchArgument("chassis_vel_input_topic", default_value="cmd_vel_gimbal_yaw_odom"),
        DeclareLaunchArgument("mpc_cmd_vel_topic", default_value="/cmd_vel_mpc"),
        DeclareLaunchArgument("require_gimbal_status", default_value="True"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "localization_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "prior_pcd_file": prior_pcd_file,
            "params_file": params_file,
            "use_respawn": use_respawn,
            "launch_small_gicp_relocalization": launch_small_gicp_relocalization,
            "launch_localization_fusion": launch_localization_fusion,
            "log_level": log_level,
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "navigation_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "use_robot_state_pub": use_robot_state_pub,
            "use_respawn": use_respawn,
            "launch_fake_vel_transform": launch_fake_vel_transform,
            "launch_chassis_vel_transform": launch_chassis_vel_transform,
            "fake_vel_output_topic": fake_vel_output_topic,
            "chassis_vel_input_topic": chassis_vel_input_topic,
            "mpc_cmd_vel_topic": mpc_cmd_vel_topic,
            "require_gimbal_status": require_gimbal_status,
            "log_level": log_level,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))
    ld.add_action(SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"))
    for declaration in declarations:
        ld.add_action(declaration)
    ld.add_action(localization)
    ld.add_action(navigation)
    return ld

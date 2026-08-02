import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def _filtered_ld_library_path():
    blocked_entries = {"/opt/MVS/lib/64", "/opt/MVS/lib/32"}
    return ":".join(
        part for part in os.environ.get("LD_LIBRARY_PATH", "").split(":")
        if part and part not in blocked_entries
    )


def generate_launch_description():
    bringup_dir = get_package_share_directory("ats_nav_bringup")
    assets_dir = get_package_share_directory("ats_sentry_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")
    namespace = LaunchConfiguration("namespace")
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    rviz_force_software = LaunchConfiguration("rviz_force_software")
    launch_joy_teleop = LaunchConfiguration("launch_joy_teleop")
    launch_small_gicp_relocalization = LaunchConfiguration("launch_small_gicp_relocalization")
    launch_localization_fusion = LaunchConfiguration("launch_localization_fusion")
    launch_fake_vel_transform = LaunchConfiguration("launch_fake_vel_transform")
    launch_chassis_vel_transform = LaunchConfiguration("launch_chassis_vel_transform")
    minco_params_file = LaunchConfiguration("minco_params_file")
    goal_manager_params_file = LaunchConfiguration("goal_manager_params_file")
    mpc_params_file = LaunchConfiguration("mpc_params_file")
    mpc_cmd_vel_topic = LaunchConfiguration("mpc_cmd_vel_topic")
    fake_vel_output_topic = LaunchConfiguration("fake_vel_output_topic")
    chassis_vel_input_topic = LaunchConfiguration("chassis_vel_input_topic")
    require_gimbal_status = LaunchConfiguration("require_gimbal_status")
    log_level = LaunchConfiguration("log_level")

    declarations = [
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("world", default_value="rmul"),
        DeclareLaunchArgument(
            "map",
            default_value=[
                TextSubstitution(text=os.path.join(assets_dir, "map", "")),
                world,
                TextSubstitution(text=".yaml"),
            ],
        ),
        DeclareLaunchArgument(
            "prior_pcd_file",
            default_value=[
                TextSubstitution(text=os.path.join(assets_dir, "pcd", "")),
                world,
                TextSubstitution(text=".pcd"),
            ],
        ),
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(assets_dir, "params", "node_params.yaml"),
        ),
        DeclareLaunchArgument("use_respawn", default_value="False"),
        DeclareLaunchArgument("use_robot_state_pub", default_value="False"),
        DeclareLaunchArgument("use_rviz", default_value="False"),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=os.path.join(assets_dir, "rviz", "sentry_default_view.rviz"),
        ),
        DeclareLaunchArgument("rviz_force_software", default_value="0"),
        DeclareLaunchArgument("launch_joy_teleop", default_value="False"),
        DeclareLaunchArgument("launch_small_gicp_relocalization", default_value="True"),
        DeclareLaunchArgument("launch_localization_fusion", default_value="True"),
        DeclareLaunchArgument("launch_fake_vel_transform", default_value="True"),
        DeclareLaunchArgument("launch_chassis_vel_transform", default_value="True"),
        DeclareLaunchArgument(
            "minco_params_file",
            default_value=os.path.join(
                get_package_share_directory("minco_planner"), "config", "minco_planner_reality.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "goal_manager_params_file",
            default_value=os.path.join(
                get_package_share_directory("ats_goal_manager"), "config", "ats_goal_manager_reality.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "mpc_params_file",
            default_value=os.path.join(
                get_package_share_directory("ats_swerve_mpc"), "config", "ats_swerve_mpc_reality.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "mpc_cmd_vel_topic",
            default_value=IfElseSubstitution(
                launch_fake_vel_transform,
                "/cmd_vel_mpc",
                IfElseSubstitution(launch_chassis_vel_transform, "cmd_vel_gimbal_yaw_odom", "/cmd_vel"),
            ),
        ),
        DeclareLaunchArgument(
            "fake_vel_output_topic",
            default_value=IfElseSubstitution(
                launch_chassis_vel_transform, "cmd_vel_gimbal_yaw_odom", "/cmd_vel"
            ),
        ),
        DeclareLaunchArgument(
            "chassis_vel_input_topic",
            default_value=IfElseSubstitution(
                launch_fake_vel_transform, "cmd_vel_gimbal_yaw_odom", mpc_cmd_vel_topic
            ),
        ),
        DeclareLaunchArgument("require_gimbal_status", default_value="True"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "robot_state_publisher_launch.py")),
        condition=IfCondition(use_robot_state_pub),
        launch_arguments={"namespace": namespace, "use_sim_time": use_sim_time}.items(),
    )
    livox = Node(
        package="livox_ros_driver2", executable="livox_ros_driver2_node", name="livox_ros_driver2",
        namespace=namespace, output="screen", parameters=[params_file, {"use_sim_time": use_sim_time}],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "bringup_launch.py")),
        launch_arguments={
            "namespace": namespace, "map": map_yaml_file, "prior_pcd_file": prior_pcd_file,
            "use_sim_time": use_sim_time, "params_file": params_file, "use_respawn": use_respawn,
            "use_robot_state_pub": use_robot_state_pub,
            "launch_small_gicp_relocalization": launch_small_gicp_relocalization,
            "launch_localization_fusion": launch_localization_fusion,
            "launch_fake_vel_transform": launch_fake_vel_transform,
            "launch_chassis_vel_transform": launch_chassis_vel_transform,
            "minco_params_file": minco_params_file,
            "goal_manager_params_file": goal_manager_params_file,
            "mpc_params_file": mpc_params_file, "mpc_cmd_vel_topic": mpc_cmd_vel_topic,
            "fake_vel_output_topic": fake_vel_output_topic,
            "chassis_vel_input_topic": chassis_vel_input_topic,
            "require_gimbal_status": require_gimbal_status, "log_level": log_level,
        }.items(),
    )
    joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "joy_teleop_launch.py")),
        condition=IfCondition(launch_joy_teleop),
        launch_arguments={
            "namespace": namespace, "use_sim_time": use_sim_time, "joy_config_file": params_file,
        }.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace, "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file, "rviz_force_software": rviz_force_software,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable("LD_LIBRARY_PATH", _filtered_ld_library_path()))
    for declaration in declarations:
        ld.add_action(declaration)
    for action in (robot_state_publisher, livox, navigation, joy, rviz):
        ld.add_action(action)
    return ld

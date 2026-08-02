from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_respawn = LaunchConfiguration("use_respawn")
    launch_fake_vel_transform = LaunchConfiguration("launch_fake_vel_transform")
    launch_chassis_vel_transform = LaunchConfiguration("launch_chassis_vel_transform")
    fake_vel_output_topic = LaunchConfiguration("fake_vel_output_topic")
    chassis_vel_input_topic = LaunchConfiguration("chassis_vel_input_topic")
    mpc_cmd_vel_topic = LaunchConfiguration("mpc_cmd_vel_topic")
    require_gimbal_status = LaunchConfiguration("require_gimbal_status")
    log_level = LaunchConfiguration("log_level")

    declarations = [
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", description="Root-owned node parameter YAML path"),
        DeclareLaunchArgument("use_robot_state_pub", default_value="False"),
        DeclareLaunchArgument("use_respawn", default_value="False"),
        DeclareLaunchArgument("launch_fake_vel_transform", default_value="True"),
        DeclareLaunchArgument("launch_chassis_vel_transform", default_value="True"),
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
        DeclareLaunchArgument(
            "mpc_cmd_vel_topic",
            default_value=IfElseSubstitution(
                launch_fake_vel_transform,
                "/cmd_vel_mpc",
                IfElseSubstitution(launch_chassis_vel_transform, "cmd_vel_gimbal_yaw_odom", "/cmd_vel"),
            ),
        ),
        DeclareLaunchArgument("require_gimbal_status", default_value="True"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    common = {
        "use_sim_time": use_sim_time,
    }
    terrain = Node(
        package="terrain_analysis", executable="terrainAnalysis", name="terrain_analysis",
        namespace=namespace, output="screen", respawn=use_respawn, respawn_delay=2.0,
        parameters=[params_file, common], arguments=["--ros-args", "--log-level", log_level],
    )
    terrain_ext = Node(
        package="terrain_analysis_ext", executable="terrainAnalysisExt", name="terrain_analysis_ext",
        namespace=namespace, output="screen", respawn=use_respawn, respawn_delay=2.0,
        parameters=[params_file, common], arguments=["--ros-args", "--log-level", log_level],
    )
    base_link_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="static_transform_publisher_base_footprint_to_base_link",
        namespace=namespace, condition=UnlessCondition(use_robot_state_pub), output="screen",
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0", "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0", "--frame-id", "base_footprint", "--child-frame-id", "base_link"],
    )
    fake_yaw_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="static_transform_publisher_fake_yaw_compat",
        namespace=namespace, condition=UnlessCondition(launch_fake_vel_transform), output="screen",
        arguments=["--frame-id", "gimbal_yaw_odom", "--child-frame-id", "gimbal_yaw_fake"],
    )
    loam = Node(
        package="loam_interface", executable="loam_interface_node", name="loam_interface",
        namespace=namespace, output="screen", respawn=use_respawn, respawn_delay=2.0,
        parameters=[params_file, common], arguments=["--ros-args", "--log-level", log_level],
    )
    scan_generation = Node(
        package="sensor_scan_generation", executable="sensor_scan_generation_node", name="sensor_scan_generation",
        namespace=namespace, output="screen", respawn=use_respawn, respawn_delay=2.0,
        parameters=[params_file, common], arguments=["--ros-args", "--log-level", log_level],
    )
    fake_transform = Node(
        package="fake_vel_transform", executable="fake_vel_transform_node", name="fake_vel_transform",
        namespace=namespace, condition=IfCondition(launch_fake_vel_transform), output="screen",
        respawn=use_respawn, respawn_delay=2.0,
        parameters=[params_file, {
            "use_sim_time": use_sim_time,
            "input_cmd_vel_topic": mpc_cmd_vel_topic,
            "output_cmd_vel_topic": fake_vel_output_topic,
        }], arguments=["--ros-args", "--log-level", log_level],
    )
    chassis_transform = Node(
        package="sentry_chassis_vel_transform", executable="chassis_vel_transform_node", name="chassis_vel_transform",
        namespace=namespace, condition=IfCondition(launch_chassis_vel_transform), output="screen",
        respawn=use_respawn, respawn_delay=2.0,
        parameters=[params_file, {
            "use_sim_time": use_sim_time,
            "input_cmd_vel_topic": chassis_vel_input_topic,
        }], arguments=["--ros-args", "--log-level", log_level],
    )
    minco = Node(
        package="minco_planner", executable="minco_planner_node", name="minco_planner",
        namespace=namespace, output="screen", respawn=use_respawn, respawn_delay=2.0,
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )
    goal_manager = Node(
        package="ats_goal_manager", executable="ats_goal_manager_node", name="ats_goal_manager",
        namespace=namespace, output="screen", respawn=use_respawn, respawn_delay=2.0,
        parameters=[params_file, {
            "use_sim_time": use_sim_time,
            "require_gimbal_status": require_gimbal_status,
        }], arguments=["--ros-args", "--log-level", log_level],
    )
    mpc = Node(
        package="ats_swerve_mpc", executable="ats_swerve_mpc_node", name="ats_swerve_mpc",
        namespace=namespace, output="screen", respawn=use_respawn, respawn_delay=2.0,
        parameters=[params_file, {
            "use_sim_time": use_sim_time,
            "command_topic": mpc_cmd_vel_topic,
            "execution_command_topic": "/planner/execution_command",
            "require_gimbal_status": require_gimbal_status,
        }], arguments=["--ros-args", "--log-level", log_level],
    )

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))
    ld.add_action(SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"))
    for declaration in declarations:
        ld.add_action(declaration)
    for action in (base_link_tf, fake_yaw_tf, terrain, terrain_ext, loam, scan_generation,
                   fake_transform, chassis_transform, minco, goal_manager, mpc):
        ld.add_action(action)
    return ld

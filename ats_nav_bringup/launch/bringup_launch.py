import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import (
    IfCondition,
    LaunchConfigurationEquals,
    LaunchConfigurationNotEquals,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("ats_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    launch_nav2 = LaunchConfiguration("launch_nav2")
    launch_swerve_mpc = LaunchConfiguration("launch_swerve_mpc")
    launch_trajectory_optimizer = LaunchConfiguration("launch_trajectory_optimizer")
    launch_small_gicp_relocalization = LaunchConfiguration("launch_small_gicp_relocalization")
    launch_localization_fusion = LaunchConfiguration("launch_localization_fusion")
    launch_fake_vel_transform = LaunchConfiguration("launch_fake_vel_transform")
    launch_chassis_vel_transform = LaunchConfiguration("launch_chassis_vel_transform")
    nav_cmd_vel_topic = LaunchConfiguration("nav_cmd_vel_topic")
    fake_vel_output_topic = LaunchConfiguration("fake_vel_output_topic")
    chassis_vel_input_topic = LaunchConfiguration("chassis_vel_input_topic")
    minco_params_file = LaunchConfiguration("minco_params_file")
    goal_manager_params_file = LaunchConfiguration("goal_manager_params_file")
    mpc_params_file = LaunchConfiguration("mpc_params_file")
    mpc_cmd_vel_topic = LaunchConfiguration("mpc_cmd_vel_topic")
    require_gimbal_status = LaunchConfiguration("require_gimbal_status")
    planning_grid_owner = LaunchConfiguration("planning_grid_owner")
    log_level = LaunchConfiguration("log_level")

    # Nav2-free profile 下速度变换级一律不成立（它们会在 MPC 之后二次旋转
    # 或叠加 cmd_spin 角速度），因此这里的"是否存在变换级"必须带 launch_nav2。
    any_velocity_transform = PythonExpression([
        "'", launch_nav2, "'.lower() == 'true' and ('",
        launch_fake_vel_transform, "'.lower() == 'true' or '",
        launch_chassis_vel_transform, "'.lower() == 'true')",
    ])

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

    # Only it applies when `namespace` is not empty.
    # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
    # in config file 'nav2_multirobot_params.yaml' as a default & example.
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("")},
        condition=LaunchConfigurationEquals("namespace", ""),
    )

    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("/", namespace)},
        condition=LaunchConfigurationNotEquals("namespace", ""),
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="False", description="Whether run a SLAM"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", description="Full path to map yaml file to load"
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file", description="Full path to prior PCD file to load"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="False",
        description="Whether robot_state_publisher owns fixed robot-link transforms",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_launch_nav2_cmd = DeclareLaunchArgument(
        "launch_nav2",
        default_value="True",
        description=(
            "Whether to start the Nav2 comparison stack. False selects the "
            "Nav2-free MINCO+MPC profile (no Nav2 server, no lifecycle manager, "
            "no /plan consumer)."
        ),
    )

    declare_launch_swerve_mpc_cmd = DeclareLaunchArgument(
        "launch_swerve_mpc",
        default_value="False",
        description=(
            "Whether to start minco_planner/ats_goal_manager/ats_swerve_mpc. "
            "Only effective together with launch_nav2:=False."
        ),
    )

    declare_minco_params_file_cmd = DeclareLaunchArgument(
        "minco_params_file",
        default_value=os.path.join(
            get_package_share_directory("minco_planner"),
            "config",
            "minco_planner_reality.yaml",
        ),
        description="Authoritative minco_planner parameter file for this profile",
    )

    declare_goal_manager_params_file_cmd = DeclareLaunchArgument(
        "goal_manager_params_file",
        default_value=os.path.join(
            get_package_share_directory("ats_goal_manager"),
            "config",
            "ats_goal_manager_reality.yaml",
        ),
        description="Authoritative ats_goal_manager parameter file for this profile",
    )

    declare_mpc_params_file_cmd = DeclareLaunchArgument(
        "mpc_params_file",
        default_value=os.path.join(
            get_package_share_directory("ats_swerve_mpc"),
            "config",
            "ats_swerve_mpc_reality.yaml",
        ),
        description="Authoritative ats_swerve_mpc parameter file for this profile",
    )

    declare_mpc_cmd_vel_topic_cmd = DeclareLaunchArgument(
        "mpc_cmd_vel_topic",
        default_value="/cmd_vel",
        description=(
            "MPC body-frame [vx, vy, wz] output topic. Real robot uses /cmd_vel "
            "so that the serial chassis is the single consumer with no extra "
            "rotation or gain stage after the MPC."
        ),
    )

    declare_require_gimbal_status_cmd = DeclareLaunchArgument(
        "require_gimbal_status",
        default_value="True",
        description=(
            "Whether ats_goal_manager/ats_swerve_mpc require a fresh "
            "GimbalYawStatus ack. Only a controlled HIL profile may set False, "
            "and BODY_YAW_FOLLOW is forbidden while it is False."
        ),
    )

    declare_planning_grid_owner_cmd = DeclareLaunchArgument(
        "planning_grid_owner",
        default_value="rc_esdf",
        choices=["rc_esdf", "rog_map"],
        description="Single /rc_esdf/planning_grid owner: rc_esdf or rog_map",
    )

    declare_launch_trajectory_optimizer_cmd = DeclareLaunchArgument(
        "launch_trajectory_optimizer",
        default_value="True",
        description="Whether to start the RC-ESDF local elastic path optimizer",
    )

    declare_launch_small_gicp_relocalization_cmd = DeclareLaunchArgument(
        "launch_small_gicp_relocalization",
        default_value="True",
        description="Whether to start small_gicp map->odom relocalization",
    )

    declare_launch_localization_fusion_cmd = DeclareLaunchArgument(
        "launch_localization_fusion",
        default_value="True",
        description="Whether localization_fusion owns map->odom and /localization",
    )

    declare_launch_chassis_vel_transform_cmd = DeclareLaunchArgument(
        "launch_chassis_vel_transform",
        default_value="False",
        description="Whether to start sentry chassis velocity transform node",
    )

    declare_launch_fake_vel_transform_cmd = DeclareLaunchArgument(
        "launch_fake_vel_transform",
        default_value="True",
        description="Whether to start the fake-yaw command frame adapter.",
    )

    declare_nav_cmd_vel_topic_cmd = DeclareLaunchArgument(
        "nav_cmd_vel_topic",
        default_value=IfElseSubstitution(
            any_velocity_transform, "cmd_vel_nav2_result", "/cmd_vel"
        ),
        description="Nav2 velocity output selected for the enabled transform chain",
    )

    declare_fake_vel_output_topic_cmd = DeclareLaunchArgument(
        "fake_vel_output_topic",
        default_value=IfElseSubstitution(
            launch_chassis_vel_transform, "cmd_vel_gimbal_yaw_odom", "/cmd_vel"
        ),
        description="Fake-yaw adapter output topic",
    )

    declare_chassis_vel_input_topic_cmd = DeclareLaunchArgument(
        "chassis_vel_input_topic",
        default_value=IfElseSubstitution(
            launch_fake_vel_transform, "cmd_vel_gimbal_yaw_odom", "cmd_vel_nav2_result"
        ),
        description="Chassis-frame adapter input topic",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            Node(
                condition=IfCondition(use_composition),
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": autostart}],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "slam_launch.py")
                ),
                condition=IfCondition(slam),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "use_respawn": use_respawn,
                    "params_file": params_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "localization_launch.py")
                ),
                condition=IfCondition(PythonExpression(["not ", slam])),
                launch_arguments={
                    "namespace": namespace,
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "prior_pcd_file": prior_pcd_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                    "launch_small_gicp_relocalization": launch_small_gicp_relocalization,
                    "launch_localization_fusion": launch_localization_fusion,
                    # Nav2-free 时定位组不得残留 map_server 与 lifecycle manager，
                    # /map 改由 ats_nav_bringup/static_map_publisher.py 发布。
                    "launch_nav2": launch_nav2,
                    "log_level": log_level,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                    "launch_nav2": launch_nav2,
                    "launch_swerve_mpc": launch_swerve_mpc,
                    "minco_params_file": minco_params_file,
                    "goal_manager_params_file": goal_manager_params_file,
                    "mpc_params_file": mpc_params_file,
                    "mpc_cmd_vel_topic": mpc_cmd_vel_topic,
                    "require_gimbal_status": require_gimbal_status,
                    "planning_grid_owner": planning_grid_owner,
                    "launch_trajectory_optimizer": launch_trajectory_optimizer,
                    "use_robot_state_pub": use_robot_state_pub,
                    "launch_fake_vel_transform": launch_fake_vel_transform,
                    "launch_chassis_vel_transform": launch_chassis_vel_transform,
                    "nav_cmd_vel_topic": nav_cmd_vel_topic,
                    "fake_vel_output_topic": fake_vel_output_topic,
                    "chassis_vel_input_topic": chassis_vel_input_topic,
                    "log_level": log_level,
                }.items(),
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_launch_nav2_cmd)
    ld.add_action(declare_launch_swerve_mpc_cmd)
    ld.add_action(declare_minco_params_file_cmd)
    ld.add_action(declare_goal_manager_params_file_cmd)
    ld.add_action(declare_mpc_params_file_cmd)
    ld.add_action(declare_mpc_cmd_vel_topic_cmd)
    ld.add_action(declare_require_gimbal_status_cmd)
    ld.add_action(declare_planning_grid_owner_cmd)
    ld.add_action(declare_launch_trajectory_optimizer_cmd)
    ld.add_action(declare_launch_small_gicp_relocalization_cmd)
    ld.add_action(declare_launch_localization_fusion_cmd)
    ld.add_action(declare_launch_chassis_vel_transform_cmd)
    ld.add_action(declare_launch_fake_vel_transform_cmd)
    ld.add_action(declare_nav_cmd_vel_topic_cmd)
    ld.add_action(declare_fake_vel_output_topic_cmd)
    ld.add_action(declare_chassis_vel_input_topic_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld

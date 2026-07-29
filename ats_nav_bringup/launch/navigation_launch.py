
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("ats_nav_bringup")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    use_respawn = LaunchConfiguration("use_respawn")
    launch_nav2 = LaunchConfiguration("launch_nav2")
    launch_swerve_mpc = LaunchConfiguration("launch_swerve_mpc")
    launch_trajectory_optimizer = LaunchConfiguration("launch_trajectory_optimizer")
    planning_grid_owner = LaunchConfiguration("planning_grid_owner")
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
    log_level = LaunchConfiguration("log_level")

    # Nav2 对照链与自研 MINCO+MPC 链必须互斥：Nav2-free profile 显式
    # launch_nav2:=false，此时不启动任何 Nav2 节点、lifecycle manager，
    # 也不启动依赖 /plan 与 Nav2 cmd_vel 的 trajectory_optimizer/speed_governor。
    # 自研链只在 launch_swerve_mpc=true 且 launch_nav2=false 时成立，
    # 保证 /cmd_vel 与 /planner/execution_command 只有一个授权来源。
    swerve_chain_enabled = PythonExpression([
        "'", launch_swerve_mpc, "'.lower() == 'true' and '",
        launch_nav2, "'.lower() == 'false'",
    ])
    # Nav2-free 下 fake/chassis 速度变换级会在 MPC 之后二次旋转、增益放大或
    # 叠加 cmd_spin 角速度，必须强制关闭，命令通路只允许 MPC -> /cmd_vel。
    fake_vel_transform_enabled = PythonExpression([
        "'", launch_fake_vel_transform, "'.lower() == 'true' and '",
        launch_nav2, "'.lower() == 'true'",
    ])
    chassis_vel_transform_enabled = PythonExpression([
        "'", launch_chassis_vel_transform, "'.lower() == 'true' and '",
        launch_nav2, "'.lower() == 'true'",
    ])

    # 与上面两级保持同一判据：Nav2-free 下不存在任何变换级，
    # 不得为一个不存在的 Nav2 预留 cmd_vel_nav2_result 通路。
    any_velocity_transform = PythonExpression([
        "'", launch_nav2, "'.lower() == 'true' and ('",
        launch_fake_vel_transform, "'.lower() == 'true' or '",
        launch_chassis_vel_transform, "'.lower() == 'true')",
    ])

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "autostart": autostart}

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

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "simulation", "nav2_params.yaml"
        ),
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
        default_value="False",
        description="Use composed bringup if True",
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="the name of container that nodes will load in if use composition",
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

    declare_launch_trajectory_optimizer_cmd = DeclareLaunchArgument(
        "launch_trajectory_optimizer",
        default_value="True",
        description="Whether to start the RC-ESDF local elastic path optimizer",
    )

    declare_planning_grid_owner_cmd = DeclareLaunchArgument(
        "planning_grid_owner",
        default_value="rc_esdf",
        choices=["rc_esdf", "rog_map"],
        description="Single /rc_esdf/planning_grid owner: rc_esdf or rog_map",
    )

    declare_launch_fake_vel_transform_cmd = DeclareLaunchArgument(
        "launch_fake_vel_transform",
        default_value="True",
        description="Whether to start the Nav2 command frame adapter",
    )

    declare_launch_chassis_vel_transform_cmd = DeclareLaunchArgument(
        "launch_chassis_vel_transform",
        default_value="False",
        description="Whether to start sentry chassis velocity transform node",
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

    start_terrain_analysis_cmd = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrain_analysis",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
    )

    start_terrain_analysis_ext_cmd = Node(
        package="terrain_analysis_ext",
        executable="terrainAnalysisExt",
        name="terrain_analysis_ext",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
    )

    static_tf_base_footprint_to_base_link_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_base_footprint_to_base_link",
        condition=UnlessCondition(use_robot_state_pub),
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
            "base_footprint",
            "--child-frame-id",
            "base_link",
        ],
    )

    static_tf_fake_yaw_compat_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_fake_yaw_compat",
        condition=UnlessCondition(fake_vel_transform_enabled),
        output="screen",
        arguments=[
            "--frame-id",
            "gimbal_yaw_odom",
            "--child-frame-id",
            "gimbal_yaw_fake",
        ],
    )

    start_chassis_vel_transform_cmd = Node(
        package="sentry_chassis_vel_transform",
        executable="chassis_vel_transform_node",
        name="chassis_vel_transform",
        condition=IfCondition(chassis_vel_transform_enabled),
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            configured_params,
            {"input_cmd_vel_topic": chassis_vel_input_topic},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    load_nodes = GroupAction(
        condition=UnlessCondition(use_composition),
        actions=[
            Node(
                package="loam_interface",
                executable="loam_interface_node",
                name="loam_interface",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="sensor_scan_generation",
                executable="sensor_scan_generation_node",
                name="sensor_scan_generation",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="fake_vel_transform",
                executable="fake_vel_transform_node",
                name="fake_vel_transform",
                condition=IfCondition(fake_vel_transform_enabled),
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    {
                        "input_cmd_vel_topic": nav_cmd_vel_topic,
                        "output_cmd_vel_topic": fake_vel_output_topic,
                    },
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="trajectory_optimizer",
                executable="rc_esdf_map_node",
                name="rc_esdf_map",
                condition=IfCondition(
                    PythonExpression([
                        "'", planning_grid_owner, "'.lower() == 'rc_esdf'"
                    ])
                ),
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ],
    )

    # Nav2 对照链（含依赖 /plan 与 Nav2 cmd_vel 的 trajectory_optimizer/
    # trajectory_speed_governor）只在 launch_nav2:=true 且非 composition 时启动。
    load_nav2_nodes = GroupAction(
        condition=IfCondition(
            PythonExpression([
                "'", use_composition, "'.lower() == 'false' and '",
                launch_nav2, "'.lower() == 'true'",
            ])
        ),
        actions=[
            Node(
                package="trajectory_optimizer",
                executable="trajectory_optimizer_node",
                name="trajectory_optimizer",
                condition=IfCondition(launch_trajectory_optimizer),
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="trajectory_optimizer",
                executable="trajectory_speed_governor_node",
                name="trajectory_speed_governor",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[("cmd_vel", "cmd_vel_controller")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("cmd_vel", nav_cmd_vel_topic),  # recovery output
                ],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("cmd_vel", nav_cmd_vel_topic),  # remap output
                ],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("cmd_vel", "cmd_vel_controller_governed"),  # remap input
                    ("cmd_vel_smoothed", nav_cmd_vel_topic),  # remap output
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ],
    )

    # 实车 Nav2-free 自研链：minco_planner -> ats_goal_manager -> ats_swerve_mpc。
    # 参数唯一权威来自各包 `*_reality.yaml`，此处只覆盖 profile 级接线量
    # （use_sim_time、topic 名、授权开关），禁止在此处重复给出动力学限值。
    load_swerve_chain_nodes = GroupAction(
        condition=IfCondition(swerve_chain_enabled),
        actions=[
            Node(
                package="minco_planner",
                executable="minco_planner_node",
                name="minco_planner",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    ParameterFile(minco_params_file, allow_substs=True),
                    {
                        "use_sim_time": use_sim_time,
                        # 实车规划地图唯一所有者：owner 为 rog_map 时用 adapter 心跳。
                        "map_ready_topic": PythonExpression([
                            "'/rog_map_adapter/ready' if '", planning_grid_owner,
                            "'.lower() == 'rog_map' else ''",
                        ]),
                        # Nav2-free 下显式断开 /plan 与直接 goal_pose 订阅，
                        # 只接受目标管理器的带编号请求。
                        "goal_topic": "",
                        "global_plan_topic": "",
                        "goal_request_topic": "/ats_goal_manager/planner_goal",
                        "planner_status_topic": "/minco/planning_status",
                        "candidate_reference_path_topic": (
                            "/minco/reference_path_candidate"
                        ),
                        # 急停与执行授权归 Goal Manager，MINCO 不得越权。
                        "planner_manages_emergency_stop": False,
                    },
                ],
            ),
            Node(
                package="ats_goal_manager",
                executable="ats_goal_manager_node",
                name="ats_goal_manager",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    ParameterFile(goal_manager_params_file, allow_substs=True),
                    {
                        "use_sim_time": use_sim_time,
                        # 实车无云台反馈桥时只能在受控 HIL profile 显式关闭，
                        # 关闭期间禁止 BODY_YAW_FOLLOW；禁止伪造 ack。
                        "require_gimbal_status": require_gimbal_status,
                    },
                ],
            ),
            Node(
                package="ats_swerve_mpc",
                executable="ats_swerve_mpc_node",
                name="ats_swerve_mpc",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    ParameterFile(mpc_params_file, allow_substs=True),
                    {
                        "use_sim_time": use_sim_time,
                        # 实车唯一命令通路：MPC 直接产出车体系 /cmd_vel，
                        # MPC 之后不再有旋转级或增益级。
                        "command_topic": mpc_cmd_vel_topic,
                        "execution_command_topic": "/planner/execution_command",
                        "require_gimbal_status": require_gimbal_status,
                    },
                ],
            ),
        ],
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="loam_interface",
                plugin="loam_interface::LoamInterfaceNode",
                name="loam_interface",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="sensor_scan_generation",
                plugin="sensor_scan_generation::SensorScanGenerationNode",
                name="sensor_scan_generation",
                parameters=[configured_params],
            ),
        ],
    )

    # Nav2 对照链的 composable 版本：同样受 launch_nav2 门控，
    # Nav2-free profile 下不得载入任何 Nav2 组件与 lifecycle manager。
    load_nav2_composable_nodes = LoadComposableNodes(
        condition=IfCondition(
            PythonExpression([
                "'", use_composition, "'.lower() == 'true' and '",
                launch_nav2, "'.lower() == 'true'",
            ])
        ),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="trajectory_optimizer",
                plugin="trajectory_optimizer::TrajectorySpeedGovernor",
                name="trajectory_speed_governor",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_controller",
                plugin="nav2_controller::ControllerServer",
                name="controller_server",
                parameters=[configured_params],
                remappings=[("cmd_vel", "cmd_vel_controller")],
            ),
            ComposableNode(
                package="nav2_smoother",
                plugin="nav2_smoother::SmootherServer",
                name="smoother_server",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_planner",
                plugin="nav2_planner::PlannerServer",
                name="planner_server",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_behaviors",
                plugin="behavior_server::BehaviorServer",
                name="behavior_server",
                parameters=[configured_params],
                remappings=[
                    ("cmd_vel", nav_cmd_vel_topic),  # remap output
                ],
            ),
            ComposableNode(
                package="nav2_bt_navigator",
                plugin="nav2_bt_navigator::BtNavigator",
                name="bt_navigator",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_waypoint_follower",
                plugin="nav2_waypoint_follower::WaypointFollower",
                name="waypoint_follower",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_velocity_smoother",
                plugin="nav2_velocity_smoother::VelocitySmoother",
                name="velocity_smoother",
                parameters=[configured_params],
                remappings=[
                    ("cmd_vel", "cmd_vel_controller_governed"),  # remap input
                    ("cmd_vel_smoothed", nav_cmd_vel_topic),  # remap output
                ],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_navigation",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ],
    )

    # trajectory_optimizer 消费 Nav2 `/plan`，Nav2-free profile 下必须不启动。
    load_trajectory_optimizer_node = LoadComposableNodes(
        condition=IfCondition(
            PythonExpression([
                "'", use_composition, "'.lower() == 'true' and '",
                launch_trajectory_optimizer, "'.lower() == 'true' and '",
                launch_nav2, "'.lower() == 'true'",
            ])
        ),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="trajectory_optimizer",
                plugin="trajectory_optimizer::TrajectoryOptimizerNode",
                name="trajectory_optimizer",
                parameters=[configured_params],
            ),
        ],
    )

    load_rc_esdf_map_node = LoadComposableNodes(
        condition=IfCondition(
            PythonExpression([
                "'", use_composition, "'.lower() == 'true' and '",
                planning_grid_owner, "'.lower() == 'rc_esdf'",
            ])
        ),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="trajectory_optimizer",
                plugin="trajectory_optimizer::RcEsdfMapNode",
                name="rc_esdf_map",
                parameters=[configured_params],
            ),
        ],
    )

    load_fake_vel_transform_node = LoadComposableNodes(
        condition=IfCondition(
            PythonExpression([
                "'", use_composition, "'.lower() == 'true' and '",
                launch_fake_vel_transform, "'.lower() == 'true' and '",
                launch_nav2, "'.lower() == 'true'",
            ])
        ),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="fake_vel_transform",
                plugin="fake_vel_transform::FakeVelTransform",
                name="fake_vel_transform",
                parameters=[
                    configured_params,
                    {
                        "input_cmd_vel_topic": nav_cmd_vel_topic,
                        "output_cmd_vel_topic": fake_vel_output_topic,
                    },
                ],
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_launch_nav2_cmd)
    ld.add_action(declare_launch_swerve_mpc_cmd)
    ld.add_action(declare_minco_params_file_cmd)
    ld.add_action(declare_goal_manager_params_file_cmd)
    ld.add_action(declare_mpc_params_file_cmd)
    ld.add_action(declare_mpc_cmd_vel_topic_cmd)
    ld.add_action(declare_require_gimbal_status_cmd)
    ld.add_action(declare_launch_trajectory_optimizer_cmd)
    ld.add_action(declare_planning_grid_owner_cmd)
    ld.add_action(declare_launch_fake_vel_transform_cmd)
    ld.add_action(declare_launch_chassis_vel_transform_cmd)
    ld.add_action(declare_nav_cmd_vel_topic_cmd)
    ld.add_action(declare_fake_vel_output_topic_cmd)
    ld.add_action(declare_chassis_vel_input_topic_cmd)
    ld.add_action(declare_log_level_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_terrain_analysis_cmd)
    ld.add_action(start_terrain_analysis_ext_cmd)
    ld.add_action(static_tf_base_footprint_to_base_link_cmd)
    ld.add_action(static_tf_fake_yaw_compat_cmd)
    ld.add_action(start_chassis_vel_transform_cmd)
    ld.add_action(load_nodes)
    ld.add_action(load_nav2_nodes)
    ld.add_action(load_swerve_chain_nodes)
    ld.add_action(load_composable_nodes)
    ld.add_action(load_nav2_composable_nodes)
    ld.add_action(load_trajectory_optimizer_node)
    ld.add_action(load_rc_esdf_map_node)
    ld.add_action(load_fake_vel_transform_node)

    return ld

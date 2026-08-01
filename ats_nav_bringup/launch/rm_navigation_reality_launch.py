
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


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
    # Get the launch directory
    bringup_dir = get_package_share_directory("ats_nav_bringup")
    assets_dir = get_package_share_directory("ats_sentry_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    rviz_force_software = LaunchConfiguration("rviz_force_software")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    launch_joy_teleop = LaunchConfiguration("launch_joy_teleop")
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

    # 仅 Nav2 对照链需要把其 cmd_vel 输出重映射进兼容层。实机兼容层本身
    # 由独立开关控制，不能被 launch_nav2 隐式关闭。
    nav2_velocity_transform = PythonExpression([
        "'", launch_nav2, "'.lower() == 'true' and ('",
        launch_fake_vel_transform, "'.lower() == 'true' or '",
        launch_chassis_vel_transform, "'.lower() == 'true')",
    ])

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether run a SLAM. If True, it will disable small_gicp and send static tf (map->odom)",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="rmul",
        description="Select world. Map and PCD file share the same name as this parameter.",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=[
            TextSubstitution(text=os.path.join(assets_dir, "map", "")),
            world,
            TextSubstitution(text=".yaml"),
        ],
        description="Full path to map file to load",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value=[
            TextSubstitution(text=os.path.join(assets_dir, "pcd", "")),
            world,
            TextSubstitution(text=".pcd"),
        ],
        description="Full path to prior pcd file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation clock if True",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(assets_dir, "params", "node_params.yaml"),
        description="Formal real-robot ROS2 parameter file shared by every launched node",
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

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="False",
        description="Whether to start the robot state publisher",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_rviz_force_software_cmd = DeclareLaunchArgument(
        "rviz_force_software",
        default_value="0",
        description="Force RViz to use Mesa software rendering when set to 1",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_launch_joy_teleop_cmd = DeclareLaunchArgument(
        "launch_joy_teleop",
        default_value="False",
        description="Whether to start joystick teleop nodes",
    )

    declare_launch_nav2_cmd = DeclareLaunchArgument(
        "launch_nav2",
        default_value="True",
        description=(
            "Whether to start the Nav2 comparison stack. False selects the "
            "Nav2-free MINCO+MPC profile."
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
        default_value=params_file,
        description="Use the same formal parameter file as the rest of this profile",
    )

    declare_goal_manager_params_file_cmd = DeclareLaunchArgument(
        "goal_manager_params_file",
        default_value=params_file,
        description="Use the same formal parameter file as the rest of this profile",
    )

    declare_mpc_params_file_cmd = DeclareLaunchArgument(
        "mpc_params_file",
        default_value=params_file,
        description="Use the same formal parameter file as the rest of this profile",
    )

    declare_mpc_cmd_vel_topic_cmd = DeclareLaunchArgument(
        "mpc_cmd_vel_topic",
        default_value=IfElseSubstitution(
            launch_fake_vel_transform,
            "/cmd_vel_mpc",
            IfElseSubstitution(
                launch_chassis_vel_transform, "cmd_vel_gimbal_yaw_odom", "/cmd_vel"
            ),
        ),
        description=(
            "MPC body-frame [vx, vy, wz] source before the selected real-robot "
            "velocity compatibility route."
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
        default_value="True",
        description="Keep the gimbal-yaw to chassis command transform enabled.",
    )

    declare_launch_fake_vel_transform_cmd = DeclareLaunchArgument(
        "launch_fake_vel_transform",
        default_value="True",
        description="Whether to start the fake-yaw command frame adapter.",
    )

    declare_nav_cmd_vel_topic_cmd = DeclareLaunchArgument(
        "nav_cmd_vel_topic",
        default_value=IfElseSubstitution(
            nav2_velocity_transform, "cmd_vel_nav2_result", "/cmd_vel"
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
            launch_fake_vel_transform,
            "cmd_vel_gimbal_yaw_odom",
            IfElseSubstitution(launch_nav2, nav_cmd_vel_topic, mpc_cmd_vel_topic),
        ),
        description="Chassis-frame adapter input selected from Nav2 or MPC",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # Create our own temporary YAML files that include substitutions

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "robot_state_publisher_launch.py")
        ),
        # NOTE: This startup file is only used when the navigation module is standalone
        condition=IfCondition(use_robot_state_pub),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    start_livox_ros_driver2_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_ros_driver2",
        output="screen",
        namespace=namespace,
        parameters=[configured_params],
    )

    sanitize_ld_library_path = SetEnvironmentVariable(
        "LD_LIBRARY_PATH", _filtered_ld_library_path()
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
            "rviz_force_software": rviz_force_software,
        }.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "bringup_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "slam": slam,
            "map": map_yaml_file,
            "prior_pcd_file": prior_pcd_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
            "use_robot_state_pub": use_robot_state_pub,
            "launch_nav2": launch_nav2,
            "launch_swerve_mpc": launch_swerve_mpc,
            "minco_params_file": minco_params_file,
            "goal_manager_params_file": goal_manager_params_file,
            "mpc_params_file": mpc_params_file,
            "mpc_cmd_vel_topic": mpc_cmd_vel_topic,
            "require_gimbal_status": require_gimbal_status,
            "planning_grid_owner": planning_grid_owner,
            "launch_trajectory_optimizer": launch_trajectory_optimizer,
            "launch_small_gicp_relocalization": launch_small_gicp_relocalization,
            "launch_localization_fusion": launch_localization_fusion,
            "launch_fake_vel_transform": launch_fake_vel_transform,
            "launch_chassis_vel_transform": launch_chassis_vel_transform,
            "nav_cmd_vel_topic": nav_cmd_vel_topic,
            "fake_vel_output_topic": fake_vel_output_topic,
            "chassis_vel_input_topic": chassis_vel_input_topic,
            "log_level": log_level,
        }.items(),
    )

    joy_teleop_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "joy_teleop_launch.py")),
        condition=IfCondition(launch_joy_teleop),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "joy_config_file": params_file,
        }.items(),
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_rviz_force_software_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_launch_joy_teleop_cmd)
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
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(sanitize_ld_library_path)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_livox_ros_driver2_node)
    ld.add_action(bringup_cmd)
    ld.add_action(joy_teleop_cmd)
    ld.add_action(rviz_cmd)

    return ld

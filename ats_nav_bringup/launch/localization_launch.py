
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("ats_nav_bringup")

    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    use_respawn = LaunchConfiguration("use_respawn")
    launch_small_gicp_relocalization = LaunchConfiguration("launch_small_gicp_relocalization")
    launch_localization_fusion = LaunchConfiguration("launch_localization_fusion")
    launch_nav2 = LaunchConfiguration("launch_nav2")
    log_level = LaunchConfiguration("log_level")

    lifecycle_nodes = ["map_server"]

    # Nav2-free profile 里运行图中不允许出现任何 Nav2 节点与 lifecycle manager，
    # 因此 map_server 与 lifecycle_manager_localization 必须跟着 launch_nav2 走；
    # /map 这个 transient-local 静态墙契约改由 static_map_publisher 承担，
    # 两者互斥，保证 /map 始终只有一个发布者。
    nav2_map_server_enabled = PythonExpression([
        "'", launch_nav2, "'.lower() == 'true'",
    ])
    static_map_publisher_enabled = PythonExpression([
        "'", launch_nav2, "'.lower() == 'false'",
    ])

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

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

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", description="Full path to map yaml file to load"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value="",
        description="Full path to prior PCD file to load",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
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

    declare_launch_nav2_cmd = DeclareLaunchArgument(
        "launch_nav2",
        default_value="True",
        description=(
            "True keeps nav2_map_server plus lifecycle_manager_localization. "
            "False selects the Nav2-free static_map_publisher so the run graph "
            "contains no Nav2 node and no lifecycle manager."
        ),
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # Nav2-free 下 /map 的唯一发布者：保持 map frame + transient-local 数据契约，
    # 但不引入任何 Nav2 节点。两个 planning_grid 属主都要求这张静态墙。
    start_static_map_publisher_cmd = Node(
        package="ats_nav_bringup",
        executable="static_map_publisher.py",
        name="static_map_publisher",
        condition=IfCondition(static_map_publisher_enabled),
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_yaml_file": map_yaml_file,
                "map_topic": "/map",
                "frame_id": "map",
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            configured_params,
            {"prior_pcd.prior_pcd_map_path": prior_pcd_file},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    static_tf_map_to_odom_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_map_to_odom",
        condition=IfCondition(PythonExpression(["not ", launch_small_gicp_relocalization])),
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

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                condition=IfCondition(nav2_map_server_enabled),
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="small_gicp_relocalization",
                executable="small_gicp_relocalization_node",
                name="small_gicp_relocalization",
                condition=IfCondition(launch_small_gicp_relocalization),
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    {
                        "prior_pcd_file": prior_pcd_file,
                        "publish_tf": PythonExpression(["not ", launch_localization_fusion]),
                    },
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="small_gicp_relocalization",
                executable="localization_fusion_node",
                name="localization_fusion",
                condition=IfCondition(
                    PythonExpression([
                        launch_small_gicp_relocalization,
                        " and ",
                        launch_localization_fusion,
                    ])
                ),
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                condition=IfCondition(nav2_map_server_enabled),
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

    # ComposableNode 本身不支持 condition，因此 Nav2 的 map_server 与 lifecycle
    # manager 必须单独成组，由 launch_nav2 控制；否则 Nav2-free profile 无法做到
    # "运行图中不出现 Nav2 节点和 lifecycle manager"。
    load_composable_nav2_map_nodes = LoadComposableNodes(
        condition=IfCondition(
            PythonExpression([use_composition, " and ", nav2_map_server_enabled])
        ),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapServer",
                name="map_server",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_localization",
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

    load_composable_nodes_with_small_gicp_and_fusion = LoadComposableNodes(
        condition=IfCondition(
            PythonExpression([
                use_composition,
                " and ",
                launch_small_gicp_relocalization,
                " and ",
                launch_localization_fusion,
            ])
        ),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="small_gicp_relocalization",
                plugin="small_gicp_relocalization::SmallGicpRelocalizationNode",
                name="small_gicp_relocalization",
                parameters=[
                    configured_params,
                    {
                        "prior_pcd_file": prior_pcd_file,
                        "publish_tf": PythonExpression(["not ", launch_localization_fusion]),
                    },
                ],
            ),
            ComposableNode(
                package="small_gicp_relocalization",
                plugin="small_gicp_relocalization::LocalizationFusionNode",
                name="localization_fusion",
                parameters=[configured_params],
            ),
        ],
    )

    load_composable_nodes_with_small_gicp_only = LoadComposableNodes(
        condition=IfCondition(
            PythonExpression([
                use_composition,
                " and ",
                launch_small_gicp_relocalization,
                " and not ",
                launch_localization_fusion,
            ])
        ),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="small_gicp_relocalization",
                plugin="small_gicp_relocalization::SmallGicpRelocalizationNode",
                name="small_gicp_relocalization",
                parameters=[
                    configured_params,
                    {"prior_pcd_file": prior_pcd_file, "publish_tf": True},
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
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_launch_small_gicp_relocalization_cmd)
    ld.add_action(declare_launch_localization_fusion_cmd)
    ld.add_action(declare_launch_nav2_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(start_point_lio_node)
    ld.add_action(static_tf_map_to_odom_cmd)
    ld.add_action(start_static_map_publisher_cmd)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nav2_map_nodes)
    ld.add_action(load_composable_nodes_with_small_gicp_and_fusion)
    ld.add_action(load_composable_nodes_with_small_gicp_only)

    return ld

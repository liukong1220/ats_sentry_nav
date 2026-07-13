from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ats_swerve_mpc"), "config", "ats_swerve_mpc.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="ats_swerve_mpc",
                executable="ats_swerve_mpc_node",
                name="ats_swerve_mpc",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
        ]
    )

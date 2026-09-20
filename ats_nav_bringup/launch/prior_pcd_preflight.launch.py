"""Reject unavailable map-frame localization priors before launching any nodes."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def validate_prior_pcd(context):
    if not IfCondition(LaunchConfiguration("launch_small_gicp_relocalization")).evaluate(context):
        return []
    world = LaunchConfiguration("world").perform(context)
    path = LaunchConfiguration("prior_pcd_file").perform(context)
    if not path or not os.path.isfile(path) or not os.access(path, os.R_OK):
        raise RuntimeError(
            f"Localization prior unavailable for world '{world}': '{path}'. "
            "Provide prior_pcd_file:=/absolute/path/to/the_matching_world.pcd "
            "containing points already expressed in map_frame. "
            "Hardware startup is blocked; another world's map will not be substituted."
        )
    return []


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="unspecified"),
        DeclareLaunchArgument("prior_pcd_file", default_value=""),
        DeclareLaunchArgument("launch_small_gicp_relocalization", default_value="True"),
        OpaqueFunction(function=validate_prior_pcd),
    ])

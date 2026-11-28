"""Reject unavailable map-frame localization priors before launching any nodes."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def _load_preflight_defaults():
    cfg_path = os.path.join(
        get_package_share_directory("ats_nav_bringup"), "config", "launch_defaults.yaml"
    )
    with open(cfg_path, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    loc = raw.get("localization") or {}
    return {
        "world": str(loc.get("prior_preflight_world", "unspecified")),
        "launch_small_gicp_relocalization": str(loc.get("launch_small_gicp_relocalization", "True")),
    }


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
    defaults = _load_preflight_defaults()
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=defaults["world"]),
        DeclareLaunchArgument("prior_pcd_file", default_value=""),
        DeclareLaunchArgument(
            "launch_small_gicp_relocalization",
            default_value=defaults["launch_small_gicp_relocalization"],
        ),
        OpaqueFunction(function=validate_prior_pcd),
    ])

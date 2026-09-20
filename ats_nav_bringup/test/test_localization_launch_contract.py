"""Exercise launch preflight without executing a hardware or localization node."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[2]


def load_launch(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def visit_without_nodes(entity, context):
    if isinstance(entity, Node):
        pytest.fail("A node was reached before the missing-prior preflight failed")
    children = entity.visit(context)
    for child in children or []:
        visit_without_nodes(child, context)


@pytest.fixture
def source_packages(monkeypatch):
    import ament_index_python.packages

    installed_package_share = ament_index_python.packages.get_package_share_directory

    def package_share(name):
        if name == "ats_nav_bringup":
            return str(PACKAGE)
        if name == "ats_sentry_bringup":
            return str(WORKSPACE / "src" / name)
        return installed_package_share(name)

    monkeypatch.setattr(
        ament_index_python.packages, "get_package_share_directory", package_share
    )


@pytest.mark.parametrize("launch_path", [
    WORKSPACE / "src/ats_sentry_bringup/launch/real_robot_navigation.launch.py",
    WORKSPACE / "src/ats_sentry_bringup/launch/bringup.launch.py",
    PACKAGE / "launch/rm_navigation_reality_launch.py",
    PACKAGE / "launch/localization_launch.py",
])
def test_missing_world_prior_blocks_every_node(launch_path, source_packages, tmp_path):
    missing = tmp_path / "rmuc_2026.pcd"
    context = LaunchContext()
    context.launch_configurations.update({
        "world": "rmuc_2026",
        "prior_pcd_file": str(missing),
        "assets_dir": str(tmp_path),
        "map": str(tmp_path / "rmuc_2026.yaml"),
        "params_file": str(tmp_path / "node_params.yaml"),
    })
    description = load_launch(launch_path).generate_launch_description()
    with pytest.raises(RuntimeError) as error:
        visit_without_nodes(description, context)
    assert "rmuc_2026" in str(error.value)
    assert str(missing) in str(error.value)
    assert "prior_pcd_file:=" in str(error.value)


def test_real_default_world_does_not_substitute_2025(source_packages):
    path = WORKSPACE / "src/ats_sentry_bringup/launch/real_robot_navigation.launch.py"
    context = LaunchContext()
    description = load_launch(path).generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.visit(context)
    expected = WORKSPACE / "src/ats_sentry_bringup/pcd/rmuc_2026.pcd"
    assert context.launch_configurations["world"] == "rmuc_2026"
    assert context.launch_configurations["prior_pcd_file"] == str(expected)


@pytest.mark.parametrize("path_kind", ["empty", "directory", "missing"])
def test_enabled_gicp_requires_a_readable_file(path_kind, tmp_path):
    path = {"empty": "", "directory": str(tmp_path), "missing": str(tmp_path / "missing.pcd")}[path_kind]
    context = LaunchContext()
    context.launch_configurations.update({
        "world": "rmuc_2026", "prior_pcd_file": path,
        "launch_small_gicp_relocalization": "true",
    })
    module = load_launch(PACKAGE / "launch/prior_pcd_preflight.launch.py")
    with pytest.raises(RuntimeError, match="rmuc_2026"):
        module.validate_prior_pcd(context)


def test_explicit_matching_prior_is_accepted(tmp_path):
    prior = tmp_path / "surveyed_map.pcd"
    prior.write_text("# PCD preflight checks availability, not registration quality\n")
    context = LaunchContext()
    context.launch_configurations.update({
        "world": "rmuc_2026", "prior_pcd_file": str(prior),
        "launch_small_gicp_relocalization": "true",
    })
    module = load_launch(PACKAGE / "launch/prior_pcd_preflight.launch.py")
    assert module.validate_prior_pcd(context) == []


def test_disabled_gicp_does_not_require_a_prior():
    context = LaunchContext()
    context.launch_configurations.update({
        "world": "fixture", "prior_pcd_file": "",
        "launch_small_gicp_relocalization": "false",
    })
    module = load_launch(PACKAGE / "launch/prior_pcd_preflight.launch.py")
    assert module.validate_prior_pcd(context) == []

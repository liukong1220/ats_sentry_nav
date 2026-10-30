"""Map loading contract of static_map_publisher: origin, yaw, resolution, trinary occupancy."""

import importlib.util
from pathlib import Path

import numpy as np
from PIL import Image
import pytest
import yaml

PUBLISHER = Path(__file__).resolve().parents[1] / "scripts" / "static_map_publisher.py"


@pytest.fixture(scope="module")
def publisher():
    spec = importlib.util.spec_from_file_location("static_map_publisher", PUBLISHER)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def write_map(directory, pixels, **overrides):
    image_path = directory / "map.pgm"
    Image.fromarray(np.asarray(pixels, dtype=np.uint8)).save(image_path)
    metadata = {
        "image": image_path.name,
        "resolution": 0.025,
        "origin": [-4.0, 1.5, 0.4],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.20,
    }
    metadata.update(overrides)
    yaml_path = directory / "map.yaml"
    yaml_path.write_text(yaml.safe_dump(metadata), encoding="utf-8")
    return yaml_path


def test_keeps_origin_yaw_resolution_and_trinary_occupancy(publisher, tmp_path):
    yaml_path = write_map(tmp_path, [[0, 255, 127], [255, 0, 127]])

    grid = publisher.load_static_occupancy_grid(yaml_path, "map")

    assert grid.header.frame_id == "map"
    assert grid.info.resolution == 0.025
    assert (grid.info.width, grid.info.height) == (3, 2)
    assert grid.info.origin.position.x == -4.0
    assert grid.info.origin.position.y == 1.5
    assert np.isclose(grid.info.origin.orientation.z, np.sin(0.2))
    assert np.isclose(grid.info.origin.orientation.w, np.cos(0.2))
    # PGM 行从上到下，OccupancyGrid 行从下到上，必须翻转。
    assert list(grid.data) == [0, 100, -1, 100, 0, -1]


def test_negate_inverts_occupancy(publisher, tmp_path):
    yaml_path = write_map(tmp_path, [[0, 255, 127]], negate=1)

    grid = publisher.load_static_occupancy_grid(yaml_path, "map")

    assert list(grid.data) == [0, 100, -1]


def test_relative_yaml_path_finds_image_next_to_yaml(publisher, tmp_path, monkeypatch):
    yaml_path = write_map(tmp_path, [[0, 255]])
    monkeypatch.chdir(tmp_path.parent)

    grid = publisher.load_static_occupancy_grid(
        Path(tmp_path.name) / yaml_path.name, "odom"
    )

    assert grid.header.frame_id == "odom"
    assert list(grid.data) == [100, 0]


@pytest.mark.parametrize(
    "overrides",
    [
        {"origin": [0.0, 0.0]},
        {"resolution": 0.0},
        {"occupied_thresh": 0.1, "free_thresh": 0.5},
        {"image": ""},
    ],
)
def test_invalid_metadata_is_rejected(publisher, tmp_path, overrides):
    yaml_path = write_map(tmp_path, [[0, 255]], **overrides)

    with pytest.raises(ValueError):
        publisher.load_static_occupancy_grid(yaml_path, "map")

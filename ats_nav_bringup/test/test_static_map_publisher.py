import importlib.util
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


def load_publisher_module():
    source = Path(__file__).parents[1] / "scripts" / "static_map_publisher.py"
    spec = importlib.util.spec_from_file_location("static_map_publisher", source)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_static_map_keeps_origin_yaw_resolution_and_trinary_occupancy(tmp_path):
    image = np.array([[0, 255, 127], [255, 0, 127]], dtype=np.uint8)
    image_path = tmp_path / "map.pgm"
    Image.fromarray(image).save(image_path)
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text(
        yaml.safe_dump(
            {
                "image": image_path.name,
                "resolution": 0.025,
                "origin": [-4.0, 1.5, 0.4],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.20,
            }
        ),
        encoding="utf-8",
    )

    grid = load_publisher_module().load_static_occupancy_grid(yaml_path, "map")

    assert grid.header.frame_id == "map"
    assert grid.info.resolution == 0.025
    assert (grid.info.width, grid.info.height) == (3, 2)
    assert grid.info.origin.position.x == -4.0
    assert grid.info.origin.position.y == 1.5
    assert np.isclose(grid.info.origin.orientation.z, np.sin(0.2))
    assert np.isclose(grid.info.origin.orientation.w, np.cos(0.2))
    # PGM rows are top-to-bottom; OccupancyGrid rows must be bottom-to-top.
    assert list(grid.data) == [0, 100, -1, 100, 0, -1]

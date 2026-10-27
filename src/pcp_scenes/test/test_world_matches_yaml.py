"""The committed Gazebo world must be exactly what make_world.py produces from shelf.yaml.

If this fails, the query generator (which reads shelf.yaml) and Gazebo (which loads shelf.sdf)
disagree about where the shelf is, and every benchmark number is suspect.
"""

import importlib.util
import pathlib

import yaml

HERE = pathlib.Path(__file__).resolve().parent
PKG = HERE.parent
WORLD = PKG.parent / "pcp_bringup" / "worlds" / "shelf.sdf"


def _load_make_world():
    spec = importlib.util.spec_from_file_location("make_world", PKG / "scripts" / "make_world.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_committed_world_is_regenerated_from_yaml():
    scene = yaml.safe_load((PKG / "config" / "shelf.yaml").read_text())
    expected = _load_make_world().make_world(scene)
    assert WORLD.read_text() == expected, (
        "run make_world.py config/shelf.yaml ../pcp_bringup/worlds/shelf.sdf"
    )


def test_target_regions_do_not_intersect_shelf_boxes():
    scene = yaml.safe_load((PKG / "config" / "shelf.yaml").read_text())
    for region in scene["target_regions"]:
        for box in scene["boxes"]:
            lo = [c - s / 2 for c, s in zip(box["position"], box["size"], strict=True)]
            hi = [c + s / 2 for c, s in zip(box["position"], box["size"], strict=True)]
            overlap = all(region["min"][i] < hi[i] and region["max"][i] > lo[i] for i in range(3))
            assert not overlap, f"{region['name']} intersects {box['name']}"

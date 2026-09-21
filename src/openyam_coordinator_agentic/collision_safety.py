"""Fail-closed model coverage and the local static-scene collision correction."""

import xml.etree.ElementTree as ET

from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld


def require_collision_coverage(xml: str) -> None:
    """Reject visual-only links; mesh format conversion is not collision modeling.

    Coverage is necessary, not proof of geometry or gripper-state accuracy.
    """
    root = ET.fromstring(xml)
    missing = [
        link.get("name")
        for link in root.findall("link")
        if link.find("visual/geometry") is not None and link.find("collision/geometry/*") is None
    ]
    if missing or root.find(".//collision/geometry/*") is None:
        raise ValueError(
            f"Unsafe OpenYAM model: missing collision geometry on {missing}. "
            "auto_convert_meshes does not create collision geometry. "
            "A validated robot and gripper collision model is required before startup."
        )


def filter_static_bench_overlap(world: RoboPlanWorld) -> None:
    """Ignore only overlap between the two fixed site obstacles, never the robot.

    RoboPlan registers obstacle/obstacle pairs as well as robot/obstacle pairs.
    A bench intersecting its safety wall must not invalidate every robot state.
    Both complete obstacle shapes and all robot collision pairs remain intact.
    """
    if not isinstance(world, RoboPlanWorld):
        raise TypeError("Static bench correction currently supports RoboPlan only")
    with world._lock:
        if not {"bench", "camera_tripod_wall"}.issubset(world._obstacles):
            raise ValueError("Both fixed bench obstacles must be registered first")
        world._require_scene().setCollisions("bench", "camera_tripod_wall", False)

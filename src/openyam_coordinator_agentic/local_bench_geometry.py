"""Configured local bench and tripod obstacle construction."""

from typing import Any

from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def bench_obstacles(config: dict[str, Any]) -> tuple[Obstacle, Obstacle]:
    """Return the locally measured bench and camera-side wall obstacles."""
    wall = config["camera_wall"]
    return (
        Obstacle(
            name="bench",
            pose=Pose(
                Vector3(*config["bench_center_m"]),
                Quaternion(*config["bench_quaternion_xyzw"]),
            ),
            obstacle_type=ObstacleType.BOX,
            dimensions=tuple(config["bench_size_m"]),
        ),
        Obstacle(
            name="camera_tripod_wall",
            pose=Pose(Vector3(*wall["center_m"]), Quaternion(*wall["quaternion_xyzw"])),
            obstacle_type=ObstacleType.BOX,
            dimensions=tuple(wall["size_m"]),
        ),
    )

"""Construct the obstacles defined by a measured workspace profile."""

from typing import TYPE_CHECKING, Annotated

from pydantic import Field, FiniteFloat, field_validator

from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.service.spec import BaseConfig

if TYPE_CHECKING:
    from openyam_coordinator_agentic.config import OpenYamWorkspaceConfig


Vector3Values = tuple[FiniteFloat, FiniteFloat, FiniteFloat]
QuaternionValues = tuple[FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat]
PositiveLength = Annotated[FiniteFloat, Field(gt=0)]
BoxSize = tuple[PositiveLength, PositiveLength, PositiveLength]


class BoxGeometryConfig(BaseConfig):
    center_m: Vector3Values
    size_m: BoxSize
    quaternion_xyzw: QuaternionValues

    @field_validator("quaternion_xyzw")
    @classmethod
    def _unit_quaternion(cls, value: QuaternionValues) -> QuaternionValues:
        if abs(sum(component * component for component in value) - 1.0) > 1e-4:
            raise ValueError("Box rotations must be unit quaternions (xyzw)")
        return value


class WorkspaceBoxConfig(BoxGeometryConfig):
    name: str = Field(min_length=1)

    def to_obstacle(self) -> Obstacle:
        return Obstacle(
            name=self.name,
            pose=PoseStamped(
                frame_id="world",
                position=Vector3(*self.center_m),
                orientation=Quaternion(*self.quaternion_xyzw),
            ),
            obstacle_type=ObstacleType.BOX,
            dimensions=self.size_m,
        )


def workspace_obstacles(config: "OpenYamWorkspaceConfig") -> tuple[WorkspaceBoxConfig, ...]:
    """Return the locally measured bench and camera-side wall obstacles."""
    wall = config.camera_wall
    return (
        WorkspaceBoxConfig(
            name="bench",
            center_m=config.bench_center_m,
            quaternion_xyzw=config.bench_quaternion_xyzw,
            size_m=config.bench_size_m,
        ),
        WorkspaceBoxConfig(name="camera_tripod_wall", **wall.model_dump()),
    )

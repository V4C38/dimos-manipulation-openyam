"""Measured workspace inputs for composing an OpenYAM grasping blueprint.

Importing this module does not read a profile or connect to hardware. Runtime
modules receive their own configuration; only blueprint composition loads JSON.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Annotated
import xml.etree.ElementTree as ET

from pydantic import Field, FiniteFloat, field_validator

from dimos.manipulation.grasping.grasp_gen_x.module import RigidTransform, SweepVolumeGripperConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.service.spec import BaseConfig
from dimos.robot.assets.model import RobotModel
from openyam_coordinator_agentic.checked_motion import GraspMotionConfig
from openyam_coordinator_agentic.dense_scene_registration import GraspPerceptionConfig
from openyam_coordinator_agentic.grasp_quality import GraspQualityConfig
from openyam_coordinator_agentic.grasping.grasp_gen_x.module import GraspGenXSamplingConfig
from openyam_coordinator_agentic.pick_and_place_module import GraspExecutionConfig
from openyam_coordinator_agentic.workspace_geometry import (
    BoxGeometryConfig,
    BoxSize,
    QuaternionValues,
    Vector3Values,
)

ArmJoints = tuple[FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat]
NonnegativeGain = Annotated[FiniteFloat, Field(ge=0)]
DampingGain = Annotated[FiniteFloat, Field(ge=0, le=5.0)]
ControlGains = tuple[
    NonnegativeGain,
    NonnegativeGain,
    NonnegativeGain,
    NonnegativeGain,
    NonnegativeGain,
    NonnegativeGain,
    NonnegativeGain,
]
DampingGains = tuple[
    DampingGain,
    DampingGain,
    DampingGain,
    DampingGain,
    DampingGain,
    DampingGain,
    DampingGain,
]


class CameraPerceptionConfig(GraspPerceptionConfig):
    color_width: int = Field(default=1280, gt=0)
    color_height: int = Field(default=720, gt=0)
    fps: int = Field(default=6, gt=0)


class ArmControlConfig(BaseConfig):
    kp: ControlGains
    kd: DampingGains


class OpenYamWorkspaceConfig(BaseConfig):
    """Runtime inputs; extra top-level calibration records remain site metadata.

    Policy sections use the same typed configs as their consuming modules.
    Unknown fields inside those sections are rejected by DimOS BaseConfig.
    """

    model_config = {"extra": "allow"}

    camera_serial: str = Field(min_length=1)
    camera_translation_m: Vector3Values
    camera_quaternion_xyzw: QuaternionValues
    collision_model: Path
    gripper: SweepVolumeGripperConfig
    grasp_frame_to_tcp: RigidTransform
    home_joints: ArmJoints
    bench_center_m: Vector3Values
    bench_size_m: BoxSize
    bench_quaternion_xyzw: QuaternionValues
    bench_top_z_m: FiniteFloat
    camera_wall: BoxGeometryConfig
    empty_epsilon: float = Field(gt=0, lt=1)
    perception: CameraPerceptionConfig
    grasp_quality: GraspQualityConfig
    graspgenx: GraspGenXSamplingConfig
    grasp_execution: GraspExecutionConfig
    approach_planning: GraspMotionConfig
    arm_control: ArmControlConfig | None = None
    planning_joint_limits_rad: dict[str, tuple[FiniteFloat, FiniteFloat]] = Field(default_factory=dict)
    runtime_environment: Path | None = Field(default=None, exclude=True)

    def planning_model(self, model: RobotModel) -> RobotModel:
        """Restrict the planning model to this bench's usable joint travel."""
        joints = {
            joint.get("name"): joint
            for joint in ET.fromstring(model.load().xml).findall("joint")
        }
        for name, (lower, upper) in self.planning_joint_limits_rad.items():
            joint = joints.get(name)
            limit = None if joint is None else joint.find("limit")
            if (
                limit is None
                or not float(limit.get("lower", "nan")) <= lower < upper
                or not upper <= float(limit.get("upper", "nan"))
            ):
                raise ValueError(f"Planning limits for {name} must narrow existing model limits")
            model = model.with_joint_position_limits(name, lower=lower, upper=upper)
        return model

    @field_validator("camera_quaternion_xyzw", "bench_quaternion_xyzw")
    @classmethod
    def _unit_quaternion(cls, value: QuaternionValues) -> QuaternionValues:
        if abs(sum(component * component for component in value) - 1.0) > 1e-4:
            raise ValueError("Workspace rotations must be unit quaternions (xyzw)")
        return value

    def camera_mount(self) -> Transform:
        return Transform(
            translation=Vector3(*self.camera_translation_m),
            rotation=Quaternion(*self.camera_quaternion_xyzw),
            frame_id="world",
            child_frame_id="camera_link",
        )


def load_workspace_config(path: Path | None = None) -> OpenYamWorkspaceConfig:
    """Load an explicit profile, resolving paths relative to its own directory."""
    if path is None:
        value = os.environ.get("OPENYAM_WORKSPACE_CONFIG")
        if not value:
            raise ValueError("Set OPENYAM_WORKSPACE_CONFIG to a measured workspace JSON file")
        path = Path(value)
    path = path.expanduser().resolve()
    config = OpenYamWorkspaceConfig.model_validate(json.loads(path.read_text(encoding="utf-8")))
    config.collision_model = (path.parent / config.collision_model).resolve()
    environment = config.runtime_environment or Path("temp/graspgenx-venv")
    config.runtime_environment = (path.parent / environment).resolve()
    return config

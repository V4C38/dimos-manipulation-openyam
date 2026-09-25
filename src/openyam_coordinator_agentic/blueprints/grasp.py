"""Compose the OpenYAM grasp stack and its agent from an explicit workspace."""

from __future__ import annotations

from dataclasses import replace
import os

from dimos.agents.mcp.mcp_client import McpClient, McpClientConfig
from dimos.agents.mcp.mcp_server import McpServer
from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.perception.detection.detectors.yoloe import YoloePromptMode
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.openyam.config import OPENYAM_GRIPPER_JOINT, openyam_hardware
from dimos.web.cockpit import Chat, Row, Video, cockpit
from openyam_coordinator_agentic.agent_prompts import OPENYAM_GRASP_AGENT_SYSTEM_PROMPT
from openyam_coordinator_agentic.collision_model import model_config
from openyam_coordinator_agentic.config import OpenYamWorkspaceConfig, load_workspace_config
from openyam_coordinator_agentic.dense_scene_registration import DenseObjectSceneRegistrationModule
from openyam_coordinator_agentic.grasping.grasp_gen_x.module import ConfiguredGraspGenXModule
from openyam_coordinator_agentic.openyam_adapter import OpenYamDiagnosticCoordinator
from openyam_coordinator_agentic.pick_and_place_module import OpenYamPickAndPlaceModule
from openyam_coordinator_agentic.workspace_geometry import workspace_obstacles
from openyam_coordinator_agentic.workspace_module import (
    WorkspaceManipulationModule,
    WorkspaceMountModule,
)


def make_openyam_grasp_graspgenx(config: OpenYamWorkspaceConfig) -> Blueprint:
    """Assemble the hardware-backed stack without starting any module."""
    mount = config.camera_mount()
    model = model_config(config.collision_model).model_copy(
        update={
            "base_pose": PoseStamped(frame_id="world"),
            "home_joints": list(config.home_joints),
        }
    )
    model.model = config.planning_model(model.model)
    hardware = openyam_hardware()
    if hardware.adapter_type == "openyam_damiao":
        hardware = replace(hardware, adapter_type="openyam_traced_damiao")
    if config.arm_control is not None:
        hardware = replace(
            hardware,
            wb_config=replace(
                hardware.wb_config,
                kp=config.arm_control.kp,
                kd=config.arm_control.kd,
            ),
        )
    perception = config.perception.model_dump(exclude={"color_width", "color_height", "fps"})
    execution = config.grasp_execution.model_dump() | {
        "planning_frame": "world",
        "grasp_verification": config.grasp_execution.grasp_verification.model_dump()
        | {"empty_epsilon": config.empty_epsilon},
    }
    runtime_env = (
        {}
        if config.runtime_environment is None
        else {"UV_PROJECT_ENVIRONMENT": str(config.runtime_environment)}
    )
    return autoconnect(
        RealSenseCamera.blueprint(
            serial_number=config.camera_serial,
            width=config.perception.color_width,
            height=config.perception.color_height,
            fps=config.perception.fps,
            align_depth_to_color=True,
            enable_pointcloud=False,
            enable_imu=False,
        ),
        DenseObjectSceneRegistrationModule.blueprint(
            target_frame="world",
            detector_backend="yoloe",
            segmentation_backend="yolo",
            prompt_mode=YoloePromptMode.PROMPT,
            detect_on_request=True,
            min_detections_for_permanent=1,
            distance_threshold=0.05,
            use_aabb=True,
            max_obstacle_width=0.0,
            support_plane_z_m=config.bench_top_z_m,
            **perception,
        ),
        ConfiguredGraspGenXModule.blueprint(
            gripper=config.gripper,
            grasp_frame_to_tcp=config.grasp_frame_to_tcp,
            max_candidates=config.graspgenx.num_grasps,
            extra_env=runtime_env,
            **config.graspgenx.model_dump(),
        ),
        WorkspaceMountModule.blueprint(mount=mount),
        WorkspaceManipulationModule.blueprint(
            model=model,
            world_frame="world",
            static_transforms=[mount],
            visualization={"backend": "viser"},
            default_speed_scale=1.0,
            linear_speed_scale=1.0,
            obstacles=workspace_obstacles(config),
            **config.approach_planning.model_dump(),
        ),
        OpenYamPickAndPlaceModule.blueprint(
            collision_model=config.collision_model,
            gripper=config.gripper,
            grasp_frame_to_tcp=config.grasp_frame_to_tcp,
            support_plane_z_m=config.bench_top_z_m,
            max_scene_skew_s=config.perception.max_rgb_depth_skew_s,
            quality=config.grasp_quality,
            **execution,
        ),
        ManipulationSkills.blueprint(),
        coordinator(
            hardware=[hardware],
            cls=OpenYamDiagnosticCoordinator,
            instance_name="ControlCoordinator",
            tasks=[
                trajectory_task(hardware),
                TaskConfig(
                    name="openyam_gripper",
                    type="gripper",
                    joint_names=[OPENYAM_GRIPPER_JOINT],
                    priority=20,
                ),
            ],
        ),
    )


openyam_grasp_graspgenx = make_openyam_grasp_graspgenx(load_workspace_config())

openyam_grasp_graspgenx_agent = autoconnect(
    openyam_grasp_graspgenx,
    McpServer.blueprint(),
    McpClient.blueprint(
        system_prompt=OPENYAM_GRASP_AGENT_SYSTEM_PROMPT,
        model=os.environ.get("OPENYAM_LLM_MODEL", McpClientConfig().model),
    ),
    cockpit(layout=Row(Video("color_image", title="Workspace camera"), Chat(title="Agent chat"))),
).global_config(n_workers=8)

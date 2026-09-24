"""OpenYAM fixed-camera grasping configured by an explicit workspace profile."""

from __future__ import annotations

import json
import os
import time
from dataclasses import replace
from pathlib import Path
from typing import Any

from dimos.agents.mcp.mcp_client import McpClient, McpClientConfig
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.skill_result import SkillResult
from dimos.control.coordinator import TaskConfig
from dimos.core.core import rpc
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.detection.detectors.yoloe import YoloePromptMode
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.openyam.config import OPENYAM_GRIPPER_JOINT, openyam_hardware
from dimos.utils.logging_config import setup_logger
from dimos.web.cockpit import Chat, Row, Video, cockpit

from openyam_coordinator_agentic.collision_model import model_config
from openyam_coordinator_agentic.collision_safety import filter_static_bench_overlap, require_collision_coverage
from openyam_coordinator_agentic.dense_scene_registration import DenseObjectSceneRegistrationModule
from openyam_coordinator_agentic.grasp_quality import GraspQualityConfig, GraspQualityFilter
from openyam_coordinator_agentic.workspace_geometry import workspace_obstacles

MAX_GRASP_ATTEMPTS = 3
MAX_DETECTION_ATTEMPTS = 5
MAX_QUALITY_RESCANS = 1
# At 6 FPS, this waits for three post-retreat RGB-D frames before the next
# on-request scan selects its latest aligned frame.
RETRY_FRAME_SETTLE_S = 0.5
# Trajectory completion reports the last command sent, not physical settling.
MOTION_SETTLE_S = 0.5
logger = setup_logger()


def workspace_config() -> dict[str, Any]:
    """Load a measured workspace profile for the grasping stack."""
    profile = os.environ.get("OPENYAM_WORKSPACE_CONFIG")
    if not profile:
        raise ValueError("Set OPENYAM_WORKSPACE_CONFIG to a measured workspace JSON file")
    path = Path(profile).resolve()
    config = json.loads(path.read_text(encoding="utf-8"))
    required = (
        "camera_serial", "camera_translation_m", "camera_quaternion_xyzw", "gripper",
        "grasp_frame_to_tcp", "bench_center_m", "bench_size_m", "bench_quaternion_xyzw",
        "camera_wall", "empty_epsilon", "perception", "grasp_quality",
        "collision_model", "home_joints",
    )
    missing = [name for name in required if config.get(name) is None]
    gripper = config.get("gripper") or {}
    missing.extend(
        f"gripper.{name}" for name in (
            "extents_open", "offset_open", "extents_half_open", "offset_half_open", "fingertip_depth"
        ) if gripper.get(name) is None
    )
    if missing:
        raise ValueError("Complete measured OpenYAM workspace fields: " + ", ".join(missing))
    config["collision_model"] = str((path.parent / config["collision_model"]).resolve())
    return config


C = workspace_config()
CAMERA_WIDTH = C["perception"]["color_width"]
CAMERA_HEIGHT = C["perception"]["color_height"]
CAMERA_FPS = C["perception"]["fps"]
_mount = Transform(
    translation=Vector3(*C["camera_translation_m"]),
    rotation=Quaternion(*C["camera_quaternion_xyzw"]),
    frame_id="world", child_frame_id="camera_link",
)


class OpenYamWorkspaceMount(StaticTfPublisher):
    def transforms(self):
        return [_mount]


class CollisionAwareWorkspaceManipulation(ManipulationModule):
    """Planner containing the measured bench and camera-side obstacle."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        require_collision_coverage(self.config.model.model.load().xml)

    def _initialize_planning(self) -> None:
        super()._initialize_planning()
        assert self._world_monitor is not None
        for obstacle in workspace_obstacles(C):
            self._world_monitor.add_obstacle(obstacle)
        filter_static_bench_overlap(self._world_monitor.world)


class OpenYamPickAndPlace(PickAndPlaceModule):
    """Use the URDF tip's -Z approach axis for approach and retreat."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._last_grasp_leg: tuple[PoseStamped, PoseStamped, Any] | None = None
        quality = C["grasp_quality"]
        self._grasp_quality = GraspQualityFilter(
            C["gripper"], C["grasp_frame_to_tcp"], GraspQualityConfig(**quality)
        )
        self._last_grasp_quality: dict[str, Any] = {}

    @staticmethod
    def _offset_pose(pose: PoseStamped, offset: float) -> PoseStamped:
        # The fixed-camera grasp clearance is a vertical lift in the world
        # frame. Rotating this offset with a tilted grasp can shift the TCP
        # several centimetres sideways before it ever reaches the object.
        return PoseStamped(
            ts=pose.ts,
            frame_id=pose.frame_id,
            position=pose.position + Vector3(0, 0, offset),
            orientation=pose.orientation,
        )

    def _servo(self, start: PoseStamped, end: PoseStamped, planning_group: Any) -> SkillResult | None:
        """Remember the final approach so a failed close can reverse it."""
        self._last_grasp_leg = (start, end, planning_group)
        self._log_reached_pose("linear_start", start, planning_group)
        state = self._manipulation.get_state().groups.get(planning_group)
        reached = None if state is None else state.end_effector_pose
        if reached is None or reached.frame_id != end.frame_id:
            return SkillResult.fail("EXECUTION_FAILED", "Current TCP pose unavailable in the target frame")
        # move_linear is relative to measured FK. Subtract that same pose so
        # pregrasp tracking error is not carried through to the contact target.
        failure = super()._servo(reached, end, planning_group)
        if failure is None:
            time.sleep(MOTION_SETTLE_S)
        self._log_reached_pose("linear_end", end, planning_group)
        return failure

    def _move(self, pose: PoseStamped, planning_group: Any) -> SkillResult | None:
        failure = super()._move(pose, planning_group)
        if failure is None:
            time.sleep(MOTION_SETTLE_S)
        self._log_reached_pose("pregrasp_move_end", pose, planning_group)
        if failure is None:
            state = self._manipulation.get_state().groups.get(planning_group)
            reached = None if state is None else state.end_effector_pose
            if reached is None or reached.frame_id != pose.frame_id:
                return SkillResult.fail("EXECUTION_FAILED", "Reached TCP pose unavailable")
            error = reached.position.distance(pose.position)
            if error > 0.01:
                return SkillResult.fail(
                    "EXECUTION_FAILED", f"Pregrasp position was not reached ({error:.3f} m error)"
                )
        return failure

    @staticmethod
    def _pose_record(pose: PoseStamped) -> dict[str, Any]:
        return {
            "frame_id": pose.frame_id,
            "position_m": pose.position.to_list(),
            "quaternion_xyzw": pose.orientation.to_list(),
        }

    def _log_reached_pose(self, phase: str, target: PoseStamped, group: Any) -> None:
        """Record encoder-derived FK; this is not an independent physical measurement."""
        try:
            snapshot = self._manipulation.get_state()
            state = snapshot.groups.get(group)
            reached = None if state is None else state.end_effector_pose
            error = None
            if reached is not None and reached.frame_id == target.frame_id:
                error = (reached.position - target.position).to_list()
            logger.info(
                "OpenYAM grasp tracking", phase=phase, planning_group=str(group),
                snapshot_timestamp=snapshot.timestamp,
                target=self._pose_record(target),
                reached_fk=None if reached is None else self._pose_record(reached),
                reached_minus_target_m=error,
            )
        except Exception as exc:
            # Diagnostic availability must not change motion or retry behavior.
            logger.warning("OpenYAM grasp tracking unavailable", phase=phase, error=str(exc))

    def _back_off_after_failed_grasp(self) -> SkillResult | None:
        """Retreat, then clear the fixed camera's view before rescanning."""
        if self._last_grasp_leg is None:
            return SkillResult.fail("RETRY_BACKOFF_FAILED", "No completed grasp approach to reverse")
        pregrasp, grasp, group = self._last_grasp_leg
        reverse_failure = super()._servo(grasp, pregrasp, group)
        state = self._manipulation.get_state().groups.get(group)
        target = None if state is None else state.joint_presets.get("home")
        if target is None:
            return SkillResult.fail("RETRY_BACKOFF_FAILED", "Configured home pose is unavailable")
        plan = self._manipulation.plan_to_joints({group: target})
        if not plan.succeeded:
            detail = reverse_failure.message if reverse_failure else plan.message
            return SkillResult.fail("RETRY_BACKOFF_FAILED", f"Could not clear camera view: {detail}")
        execution = self._manipulation.execute(blocking=True)
        if not execution.succeeded:
            return SkillResult.fail("RETRY_BACKOFF_FAILED", f"Could not clear camera view: {execution.message}")
        return None

    @rpc
    def get_grasp_quality_report(self) -> dict[str, Any]:
        """Return diagnostics from the most recent proposal-quality evaluation."""
        return self._last_grasp_quality

    def _pick_once(self, object_id: str, planning_group: str | None) -> SkillResult:
        """Generate, gate, and execute one grasp without any perception retry policy."""
        self._clear_selection()
        if object_id not in self._objects:
            return SkillResult.fail("OBJECT_NOT_DETECTED", f"Unknown object_id: {object_id}")
        try:
            pointcloud = self._scene.get_object_pointcloud_by_object_id(object_id)
            if pointcloud is None:
                return SkillResult.fail("OBJECT_NOT_DETECTED", f"No pointcloud for object_id: {object_id}")
            candidates = self._grasp_generator.propose_grasps(pointcloud)
        except (RuntimeError, ValueError) as exc:
            return SkillResult.fail("GRASP_GENERATION_FAILED", str(exc))
        candidates, self._last_grasp_quality = self._grasp_quality.filter(
            candidates, pointcloud
        )
        points = pointcloud.points_f32()
        logger.info(
            "OpenYAM grasp perception", object_id=object_id,
            frame_id=pointcloud.frame_id, cloud_timestamp=pointcloud.ts,
            point_count=len(points),
            centroid_m=points.mean(axis=0).tolist() if len(points) else None,
            bounds_min_m=points.min(axis=0).tolist() if len(points) else None,
            bounds_max_m=points.max(axis=0).tolist() if len(points) else None,
            grasp_quality=self._last_grasp_quality,
        )
        self._grasp_candidates = candidates
        self._manipulation.show_grasp_proposals(candidates)
        if candidates.header.frame_id != self.config.planning_frame:
            return SkillResult.fail(
                "GRASP_FRAME_MISMATCH",
                f"Expected {self.config.planning_frame}, got {candidates.header.frame_id}",
            )
        if not candidates.candidates:
            failure = SkillResult.fail(
                "INSUFFICIENT_GRASP_QUALITY", "No proposal passed geometric quality gates"
            )
            failure.metadata["grasp_quality"] = self._last_grasp_quality
            return failure
        group = self._resolve_group(planning_group)
        if group is None:
            return SkillResult.fail(
                "ROBOT_NOT_FOUND", "Gripper-capable planning group is missing or ambiguous"
            )
        if failure := self._open_gripper(group, "pre-grasp open"):
            return failure
        unreachable: SkillResult | None = None
        for rank, candidate in enumerate(candidates.candidates[: self.config.max_grasp_attempts]):
            grasp = self._apply_yaw_policy(
                PoseStamped(
                    ts=candidates.header.timestamp,
                    frame_id=candidates.header.frame_id,
                    position=candidate.pose.position,
                    orientation=candidate.pose.orientation,
                ),
                group,
            )
            pregrasp = self._offset_pose(grasp, self.config.pregrasp_offset)
            logger.info(
                "OpenYAM grasp target", object_id=object_id, rank=rank,
                score=float(candidate.score), grasp=self._pose_record(grasp),
                pregrasp=self._pose_record(pregrasp),
            )
            failure = self._move(pregrasp, group) or self._servo(pregrasp, grasp, group)
            if failure is not None:
                if failure.error_code != "PLANNING_FAILED":
                    return failure
                unreachable = failure
                continue
            if failure := self._close_and_verify(group):
                return failure
            self._selected_object_id = object_id
            self._selected_grasp = grasp
            self._holding_object = True
            if failure := self._servo(grasp, pregrasp, group):
                return failure
            return SkillResult.ok(
                "Pick complete", object_id=object_id, rank=rank, score=candidate.score,
                candidates=len(candidates.candidates), grasp_quality=self._last_grasp_quality,
            )
        return unreachable or SkillResult.fail("PLANNING_FAILED", "No grasp candidate was reachable")

    @skill(uses=[CAP_MOVEMENT])
    def pick_object(self, object_id: str, planning_group: str | None = None) -> SkillResult:
        """Pick with two camera-clear, fresh-perception retries, then park."""
        attempted_ids = [object_id]
        self._last_grasp_leg = None
        quality_rescans = 0
        result = self._pick_once(object_id, planning_group)
        for _attempt in range(1, MAX_GRASP_ATTEMPTS):
            if result.success or result.error_code not in {
                "GRASP_VERIFICATION_FAILED", "INSUFFICIENT_GRASP_QUALITY"
            }:
                break
            if result.error_code == "GRASP_VERIFICATION_FAILED":
                back_off = self._back_off_after_failed_grasp()
                if back_off is not None:
                    result.metadata["retry_backoff"] = back_off.message
                    break
            else:
                quality_rescans += 1
                if quality_rescans > MAX_QUALITY_RESCANS:
                    break
            # scan_objects processes the latest aligned RGB-D frame. Wait for
            # frames acquired after the arm has cleared the object, rather than
            # reusing the pre-grasp image that drove the failed attempt.
            object_name = self._objects.get(object_id, {}).get("name")
            if not object_name:
                result.metadata["retry_scan"] = "object label unavailable"
                break
            matches: list[dict[str, Any]] = []
            scan: SkillResult | None = None
            for _scan_attempt in range(MAX_DETECTION_ATTEMPTS):
                time.sleep(RETRY_FRAME_SETTLE_S)
                scan = self.scan_objects([object_name])
                matches = [
                    item for item in scan.metadata.get("objects", [])
                    if item.get("name") == object_name and item.get("object_id")
                ] if scan.success else []
                if matches:
                    break
            if not matches:
                result.metadata["retry_scan"] = (
                    f"no {object_name} in five fresh scans"
                    if scan is not None and scan.success
                    else "fresh scan failed" if scan is None else scan.message
                )
                break
            # The scene module returns objects observed in this scan only, but
            # the detector can emit overlapping hypotheses. They all
            # originate from the new frame; use its first returned detection
            # rather than treating a duplicate hypothesis as a terminal error.
            object_id = str(matches[0]["object_id"])
            attempted_ids.append(object_id)
            self._last_grasp_leg = None
            result = self._pick_once(object_id, planning_group)

        if result.success:
            result.metadata["grasp_attempts"] = len(attempted_ids)
            return result

        result.metadata["grasp_attempts"] = len(attempted_ids)
        result.metadata["attempted_object_ids"] = attempted_ids

        if result.error_code == "INSUFFICIENT_GRASP_QUALITY":
            return result

        group = self._resolve_group(planning_group)
        if group is None:
            result.metadata["folded_return"] = "skipped: planning group unavailable"
            return result
        state = self._manipulation.get_state().groups.get(group)
        target = None if state is None else state.joint_presets.get("home")
        if target is None:
            result.metadata["folded_return"] = "skipped: configured home preset unavailable"
            return result
        plan = self._manipulation.plan_to_joints({group: target})
        if not plan.succeeded:
            result.metadata["folded_return"] = f"failed to plan: {plan.message}"
            return result
        execution = self._manipulation.execute(blocking=True)
        result.metadata["folded_return"] = (
            "completed" if execution.succeeded else f"failed to execute: {execution.message}"
        )
        return result


_sensing = (
    RealSenseCamera.blueprint(
        serial_number=C["camera_serial"], width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps=CAMERA_FPS,
        align_depth_to_color=True, enable_pointcloud=False, enable_imu=False,
    ),
    DenseObjectSceneRegistrationModule.blueprint(
        target_frame="world", detector_backend="yoloe", segmentation_backend="yolo",
        prompt_mode=YoloePromptMode.PROMPT, detect_on_request=True,
        min_detections_for_permanent=1, distance_threshold=0.05, use_aabb=True,
        max_obstacle_width=0.0,
        object_voxel_downsample_m=C["perception"]["object_voxel_downsample_m"],
        object_mask_erode_pixels=C["perception"]["object_mask_erode_pixels"],
        object_outlier_neighbors=C["perception"]["object_outlier_neighbors"],
        object_outlier_std_ratio=C["perception"]["object_outlier_std_ratio"],
    ),
    GraspGenXModule.blueprint(
        gripper=C["gripper"], grasp_frame_to_tcp=C["grasp_frame_to_tcp"],
        max_candidates=C["grasp_quality"]["max_candidates"],
    ),
)

_model = model_config(Path(C["collision_model"])).model_copy(
    update={
        "base_pose": PoseStamped(frame_id="world"),
        "home_joints": list(C["home_joints"]),
    }
)
_hardware = openyam_hardware()
if control := C.get("arm_control"):
    _hardware = replace(
        _hardware,
        wb_config=replace(_hardware.wb_config, kp=tuple(control["kp"]), kd=tuple(control["kd"])),
    )
_openyam_grasp_stack = autoconnect(
    *_sensing,
    OpenYamWorkspaceMount.blueprint(),
    CollisionAwareWorkspaceManipulation.blueprint(
        model=_model, world_frame="world", static_transforms=[_mount],
        visualization={"backend": "viser"}, default_speed_scale=0.6, linear_speed_scale=0.6,
    ),
    OpenYamPickAndPlace.blueprint(
        planning_frame="world", max_grasp_attempts=20, yaw_policy="generated",
        grasp_verification={"empty_epsilon": C["empty_epsilon"]},
    ),
    coordinator(
        hardware=[_hardware],
        tasks=[trajectory_task(_hardware), TaskConfig(
            name="openyam_gripper", type="gripper", joint_names=[OPENYAM_GRIPPER_JOINT], priority=20,
        )],
    ),
)

OPENYAM_GRASP_AGENT_PROMPT = """\
You control an OpenYAM arm with a calibrated fixed RGB-D camera and a gripper.

For an object pick, call scan_objects with the requested object description,
then pick_object with an exact object ID from that scan. The pick tool generates
and checks grasp proposals. It handles fresh-scan retries after a failed grasp.
Only call place_at after a successful pick and when the user supplied explicit
world-frame TCP coordinates. Do not infer a release coordinate from the image.
Keep the gripper closed while carrying an object. Do not issue your own pick
retries after a terminal failure. Report the failure and stop.
"""

openyam_grasp_graspgenx_agent = autoconnect(
    _openyam_grasp_stack,
    ManipulationSkills.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(
        system_prompt=OPENYAM_GRASP_AGENT_PROMPT,
        model=os.environ.get("OPENYAM_LLM_MODEL", McpClientConfig().model),
    ),
    cockpit(layout=Row(Video("color_image", title="Workspace camera"), Chat(title="Agent chat"))),
).global_config(n_workers=8)

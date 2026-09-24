"""OpenYAM fixed-camera grasping configured by an explicit workspace profile."""

from __future__ import annotations

import json
import os
import time
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
from pydantic import Field

from dimos.agents.mcp.mcp_client import McpClient, McpClientConfig
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.skill_result import SkillResult
from dimos.control.coordinator import TaskConfig
from dimos.core.core import rpc
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule, PickAndPlaceModuleConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header
from dimos.perception.detection.detectors.yoloe import YoloePromptMode
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.openyam.config import OPENYAM_GRIPPER_JOINT, openyam_hardware
from dimos.utils.logging_config import setup_logger
from dimos.web.cockpit import Chat, Row, Video, cockpit

from openyam_coordinator_agentic.collision_model import model_config
from openyam_coordinator_agentic.collision_safety import filter_static_bench_overlap, require_collision_coverage
from openyam_coordinator_agentic.dense_scene_registration import DenseObjectSceneRegistrationModule
from openyam_coordinator_agentic.grasp_quality import GraspQualityConfig, GraspQualityFilter, _pose_matrix
from openyam_coordinator_agentic.workspace_geometry import workspace_obstacles
from openyam_coordinator_agentic.graspgen_config import ConfiguredGraspGenXModule
from openyam_coordinator_agentic.gripper_geometry import GripperGeometry
from openyam_coordinator_agentic.checked_motion import CheckedManipulationModule, CheckedManipulationSpec, pose_error

MAX_GRASP_ATTEMPTS = 3
MAX_DETECTION_ATTEMPTS = 5
MAX_QUALITY_RESCANS = 1
# At 6 FPS, this waits for three post-retreat RGB-D frames before the next
# on-request scan selects its latest aligned frame.
RETRY_FRAME_SETTLE_S = 0.5
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
        "collision_model", "home_joints", "graspgenx", "grasp_execution",
        "approach_planning", "bench_top_z_m",
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


class CollisionAwareWorkspaceManipulation(CheckedManipulationModule):
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


class OpenYamPickConfig(PickAndPlaceModuleConfig):
    contact_position_tolerance_m: float = Field(default=0.005, gt=0)
    contact_orientation_tolerance_deg: float = Field(default=3.0, gt=0)
    settle_timeout_s: float = Field(default=3.0, gt=0)
    max_joint_state_age_s: float = Field(default=0.5, gt=0)
    max_object_age_s: float = Field(default=30.0, gt=0)
    lift_distance_m: float = Field(default=0.10, gt=0)
    hold_verification_s: float = Field(default=1.0, gt=0)
    max_hold_aperture_change: float = Field(default=0.05, gt=0)
    home_joint_tolerance_rad: float = Field(default=0.02, gt=0)
    supported_contact_tolerance_m: float = Field(default=0.015, gt=0, le=0.015)


class OpenYamPickAndPlace(PickAndPlaceModule):
    """Use the URDF tip's -Z approach axis for approach and retreat."""

    config: OpenYamPickConfig
    _manipulation: CheckedManipulationSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._last_grasp_leg: tuple[PoseStamped, PoseStamped, Any] | None = None
        quality = C["grasp_quality"]
        geometry = GripperGeometry(Path(C["collision_model"]))
        geometry.validate_capture(C["gripper"], C["grasp_frame_to_tcp"])
        self._grasp_quality = GraspQualityFilter(
            C["gripper"], C["grasp_frame_to_tcp"], GraspQualityConfig(**quality), geometry
        )
        self._last_grasp_quality: dict[str, Any] = {}
        self._failed_grasps: list[np.ndarray] = []
        self._minimum_observation_ts = 0.0
        self._last_object_center: np.ndarray | None = None

    @staticmethod
    def _offset_pose(pose: PoseStamped, offset: float) -> PoseStamped:
        return PoseStamped(
            ts=pose.ts,
            frame_id=pose.frame_id,
            position=pose.position + pose.orientation.rotate_vector(Vector3(0, 0, offset)),
            orientation=pose.orientation,
        )

    def _servo(self, start: PoseStamped, end: PoseStamped, planning_group: Any) -> SkillResult | None:
        """Reach an absolute pose with collision checking and encoder settling."""
        self._log_reached_pose("linear_start", start, planning_group)
        result = self._manipulation.move_to_contact(end, planning_group)
        failure = None if result.success else result
        if failure is None:
            failure = self._await_pose(end, planning_group)
        self._log_reached_pose("linear_end", end, planning_group)
        return failure

    def _move(self, pose: PoseStamped, planning_group: Any) -> SkillResult | None:
        failure = super()._move(pose, planning_group)
        if failure is None:
            failure = self._await_pose(pose, planning_group)
        self._log_reached_pose("pregrasp_move_end", pose, planning_group)
        return failure

    def _await_pose(self, target: PoseStamped, group: str, *, position_tolerance_m: float | None = None) -> SkillResult | None:
        """Require three distinct, fresh encoder observations at the target."""
        tolerance = (self.config.contact_position_tolerance_m
                     if position_tolerance_m is None else position_tolerance_m)
        deadline = time.monotonic() + self.config.settle_timeout_s
        started = time.time()
        previous_ts = 0.0
        stable = 0
        distance, angle = float("inf"), float("inf")
        while time.monotonic() < deadline:
            state = self._manipulation.get_state().groups.get(group)
            joints = None if state is None else state.joints
            reached = None if state is None else state.end_effector_pose
            fresh = (joints is not None and np.isfinite(joints.ts)
                     and joints.ts > max(started, previous_ts)
                     and -0.1 <= time.time() - joints.ts <= self.config.max_joint_state_age_s)
            if fresh and reached is not None:
                previous_ts = joints.ts
                distance, angle = pose_error(reached, target)
                slow = not joints.velocity or max(abs(v) for v in joints.velocity) < 0.04
                stable = stable + 1 if (slow and distance <= tolerance
                                       and angle <= self.config.contact_orientation_tolerance_deg) else 0
                if stable >= 3:
                    return None
            elif not fresh:
                stable = 0
            time.sleep(0.1)
        self._manipulation.cancel()
        return SkillResult.fail("EXECUTION_FAILED", f"TCP did not converge ({distance:.4f} m, {angle:.2f} deg)")

    def _approach_supported_contact(
        self, pregrasp: PoseStamped, target: PoseStamped, group: str,
        cloud: PointCloud2, scene: PointCloud2, raw_score: float,
    ) -> tuple[PoseStamped | None, SkillResult | None]:
        """Allow a bounded miss only when the attained pose still supports grasping."""
        self._log_reached_pose("linear_start", pregrasp, group)
        result = self._manipulation.move_to_contact(target, group)
        failure = None if result.success else result
        if failure is None:
            failure = self._await_pose(target, group,
                                       position_tolerance_m=self.config.supported_contact_tolerance_m)
        self._log_reached_pose("linear_end", target, group)
        if failure is not None:
            return None, failure
        if failure := self._observation_failure(cloud):
            return None, failure
        state = self._manipulation.get_state().groups.get(group)
        if (state is None or state.joints is None or state.end_effector_pose is None
                or not -0.1 <= time.time() - state.joints.ts <= self.config.max_joint_state_age_s):
            return None, SkillResult.fail("EXECUTION_FAILED", "Fresh contact feedback unavailable")
        reached = state.end_effector_pose
        distance, angle = pose_error(reached, target)
        if (distance > self.config.supported_contact_tolerance_m
                or angle > self.config.contact_orientation_tolerance_deg):
            return None, SkillResult.fail("EXECUTION_FAILED", "Contact pose moved outside the bounded settling tolerance")
        # Keep the generator's raw confidence for the single-pose geometry check;
        # the quality-weighted display score is not a new model confidence.
        actual = GraspCandidate(Pose(position=reached.position, orientation=reached.orientation), score=raw_score)
        supported, report = self._grasp_quality.filter(
            GraspCandidateArray(Header(cloud.ts, cloud.frame_id), [actual]), cloud, scene,
            support_z=C["bench_top_z_m"], approach_distance=0.0,
        )
        self._last_grasp_quality["attained_contact"] = report
        logger.info("OpenYAM attained contact", position_error_m=distance,
                    orientation_error_deg=angle, supported=bool(supported.candidates), quality=report)
        if not supported.candidates:
            failure = SkillResult.fail("CONTACT_NOT_SUPPORTED", "Reached pose does not retain object support/clearance; jaws kept open")
            failure.metadata["contact_quality"] = report
            return None, failure
        return reached, None

    def _close_and_verify(self, planning_group: str) -> SkillResult | None:
        logger.info("OpenYAM close requested", planning_group=planning_group,
                    readback_before=self._gripper_position(planning_group))
        failure = super()._close_and_verify(planning_group)
        logger.info("OpenYAM close completed", planning_group=planning_group,
                    readback_after=self._gripper_position(planning_group),
                    success=failure is None, failure=None if failure is None else failure.message)
        return failure

    def _verify_lift(self, group: str, closed_reading: float | None) -> SkillResult | None:
        """Check sustained jaw obstruction after lifting; do not command closure again."""
        cfg = self.config.grasp_verification
        deadline = time.monotonic() + self.config.hold_verification_s
        samples = 0
        previous_ts = 0.0
        while time.monotonic() < deadline:
            state = self._manipulation.get_state().groups.get(group)
            reading = None if state is None else state.gripper_position
            joints = None if state is None else state.joints
            fresh = (joints is not None and previous_ts < joints.ts
                     and -0.1 <= time.time() - joints.ts <= self.config.max_joint_state_age_s)
            if (not fresh or reading is None or not np.isfinite(reading)
                    or not cfg.held_low < reading < cfg.held_high
                    or closed_reading is None or abs(reading - closed_reading) > self.config.max_hold_aperture_change):
                return SkillResult.fail("HOLD_VERIFICATION_FAILED", "Jaw feedback did not confirm a sustained hold after lift")
            previous_ts = joints.ts
            samples += 1
            time.sleep(0.1)
        logger.info("OpenYAM grasp hold verified", planning_group=group, readback=reading, samples=samples)
        return None

    def _await_home(self, target: JointState, group: str) -> SkillResult | None:
        deadline = time.monotonic() + self.config.settle_timeout_s
        previous_ts = time.time()
        stable = 0
        while time.monotonic() < deadline:
            state = self._manipulation.get_state().groups.get(group)
            joints = None if state is None else state.joints
            if (joints is not None and joints.ts > previous_ts
                    and -0.1 <= time.time() - joints.ts <= self.config.max_joint_state_age_s):
                previous_ts = joints.ts
                actual = dict(zip(joints.name, joints.position))
                errors = [abs(actual.get(name, float("inf")) - value)
                          for name, value in zip(target.name, target.position)]
                slow = not joints.velocity or max(abs(v) for v in joints.velocity) < 0.04
                stable = stable + 1 if (errors and np.all(np.isfinite(errors)) and slow
                                       and max(errors) <= self.config.home_joint_tolerance_rad) else 0
                if stable >= 3:
                    return None
            else:
                stable = 0
            time.sleep(0.1)
        self._manipulation.cancel()
        return SkillResult.fail("EXECUTION_FAILED", "Home position did not settle before retry scan")

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
                joints=None if state is None or state.joints is None else {
                    "names": state.joints.name, "positions": state.joints.position,
                    "velocities": state.joints.velocity, "efforts": state.joints.effort,
                },
            )
        except Exception as exc:
            # Diagnostic availability must not change motion or retry behavior.
            logger.warning("OpenYAM grasp tracking unavailable", phase=phase, error=str(exc))

    def _back_off_after_failed_grasp(self) -> SkillResult | None:
        """Retreat, then clear the fixed camera's view before rescanning."""
        if self._last_grasp_leg is None:
            return SkillResult.fail("RETRY_BACKOFF_FAILED", "No completed grasp approach to reverse")
        pregrasp, grasp, group = self._last_grasp_leg
        reverse_failure = self._servo(grasp, pregrasp, group)
        if reverse_failure is not None:
            return reverse_failure
        state = self._manipulation.get_state().groups.get(group)
        target = None if state is None else state.joint_presets.get("home")
        if target is None:
            return SkillResult.fail("RETRY_BACKOFF_FAILED", "Configured home pose is unavailable")
        plan = self._manipulation.plan_to_joints({group: target})
        if not plan.succeeded:
            return SkillResult.fail("RETRY_BACKOFF_FAILED", f"Could not clear camera view: {plan.message}")
        execution = self._manipulation.execute(blocking=True)
        if not execution.succeeded:
            return SkillResult.fail("RETRY_BACKOFF_FAILED", f"Could not clear camera view: {execution.message}")
        if failure := self._await_home(target, group):
            return failure
        self._last_grasp_leg = None
        self._minimum_observation_ts = time.time()
        return None

    @rpc
    def get_grasp_quality_report(self) -> dict[str, Any]:
        """Return diagnostics from the most recent proposal-quality evaluation."""
        return self._last_grasp_quality

    def _observation_failure(self, cloud: PointCloud2) -> SkillResult | None:
        if (not np.isfinite(cloud.ts) or cloud.ts <= self._minimum_observation_ts
                or not -0.1 <= time.time() - cloud.ts <= self.config.max_object_age_s):
            return SkillResult.fail("STALE_OBSERVATION", "Object observation expired; scan again")
        return None

    def _retry_object(self, matches: list[dict[str, Any]]) -> str | None:
        """Associate a retry with nearby observed geometry, including duplicate labels."""
        if self._last_object_center is None:
            return None
        nearby = []
        for item in matches:
            cloud = self._scene.get_object_pointcloud_by_object_id(str(item["object_id"]))
            if cloud is None or self._observation_failure(cloud) is not None:
                continue
            points = cloud.points_f32()
            if len(points) and np.all(np.isfinite(points)):
                distance = float(np.linalg.norm(points.mean(axis=0) - self._last_object_center))
                nearby.append((distance, str(item["object_id"])))
        nearby.sort()
        if not nearby or nearby[0][0] > 0.12:
            return None
        if len(nearby) > 1 and nearby[1][0] - nearby[0][0] < 0.02:
            return None
        return nearby[0][1]

    def _pick_once(self, object_id: str, planning_group: str | None) -> SkillResult:
        """Generate, gate, and execute one grasp without any perception retry policy."""
        self._clear_selection()
        if object_id not in self._objects:
            return SkillResult.fail("OBJECT_NOT_DETECTED", f"Unknown object_id: {object_id}")
        try:
            pointcloud = self._scene.get_object_pointcloud_by_object_id(object_id)
            if pointcloud is None:
                return SkillResult.fail("OBJECT_NOT_DETECTED", f"No pointcloud for object_id: {object_id}")
            if failure := self._observation_failure(pointcloud):
                return failure
            scene = self._scene.get_full_scene_pointcloud(exclude_object_id=object_id, voxel_size=0.003)
            if (scene is None or not np.isfinite(scene.ts)
                    or abs(scene.ts - pointcloud.ts) > C["perception"]["max_rgb_depth_skew_s"]):
                return SkillResult.fail("PERCEPTION_FAILED", "Matching scene geometry is unavailable")
            candidates = self._grasp_generator.propose_grasps(pointcloud)
        except (RuntimeError, ValueError) as exc:
            return SkillResult.fail("GRASP_GENERATION_FAILED", str(exc))
        points = pointcloud.points_f32()
        self._last_object_center = points.mean(axis=0) if len(points) else None
        failed_world = []
        if self._last_object_center is not None:
            for relative in self._failed_grasps:
                world = relative.copy()
                world[:3, 3] += self._last_object_center
                failed_world.append(world)
        candidates, self._last_grasp_quality = self._grasp_quality.filter(
            candidates, pointcloud, scene, support_z=C["bench_top_z_m"],
            approach_distance=self.config.pregrasp_offset, failed_grasps=failed_world,
        )
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
            feasibility = self._manipulation.check_grasp_path(pregrasp, grasp, group)
            if not feasibility.success:
                logger.info("OpenYAM grasp path rejected", object_id=object_id,
                            rank=rank, reason=feasibility.message)
                unreachable = feasibility
                continue
            if failure := self._observation_failure(pointcloud):
                return failure
            failure = self._move(pregrasp, group)
            attained = None
            if failure is None:
                attained, failure = self._approach_supported_contact(
                    pregrasp, grasp, group, pointcloud, scene,
                    raw_score=self._last_grasp_quality["retained"][rank]["raw_score"],
                )
            if failure is not None:
                if failure.error_code != "PLANNING_FAILED":
                    return failure
                unreachable = failure
                continue
            assert attained is not None
            grasp = attained
            self._last_grasp_leg = (pregrasp, grasp, group)
            if failure := self._observation_failure(pointcloud):
                return failure
            if failure := self._close_and_verify(group):
                relative = _pose_matrix(GraspCandidate(
                    Pose(position=grasp.position, orientation=grasp.orientation), score=candidate.score,
                ))
                relative[:3, 3] -= self._last_object_center
                self._failed_grasps.append(relative)
                # Only a confirmed empty close may trigger another automatic pick.
                if failure.error_code != "GRASP_VERIFICATION_FAILED" or "nothing in the jaws" not in failure.message:
                    self._selected_object_id = object_id
                    self._selected_grasp = grasp
                    self._holding_object = True
                return failure
            self._selected_object_id = object_id
            self._selected_grasp = grasp
            self._holding_object = True
            closed_reading = self._gripper_position(group)
            lift = PoseStamped(frame_id=grasp.frame_id, position=grasp.position + Vector3(0, 0, self.config.lift_distance_m),
                               orientation=grasp.orientation)
            if failure := self._servo(grasp, lift, group):
                return failure
            if failure := self._verify_lift(group, closed_reading):
                return failure
            return SkillResult.ok(
                "Pick complete", object_id=object_id, rank=rank, score=candidate.score,
                candidates=len(candidates.candidates), grasp_quality=self._last_grasp_quality,
                verification="encoder_pose_and_sustained_jaw_obstruction",
            )
        return unreachable or SkillResult.fail("PLANNING_FAILED", "No grasp candidate was reachable")

    @skill(uses=[CAP_MOVEMENT])
    def pick_object(self, object_id: str, planning_group: str | None = None) -> SkillResult:
        """Pick with fresh, spatially associated retries after confirmed empty closes."""
        if self._holding_object:
            return SkillResult.fail("ALREADY_HOLDING_OBJECT", "Resolve the held or uncertain object before starting another pick")
        attempted_ids = [object_id]
        self._last_grasp_leg = None
        self._failed_grasps = []
        self._minimum_observation_ts = 0.0
        self._last_object_center = None
        quality_rescans = 0
        result = self._pick_once(object_id, planning_group)
        for _attempt in range(1, MAX_GRASP_ATTEMPTS):
            if self._holding_object or result.success or result.error_code not in {
                "GRASP_VERIFICATION_FAILED", "INSUFFICIENT_GRASP_QUALITY"
            }:
                break
            if result.error_code == "GRASP_VERIFICATION_FAILED":
                back_off = self._back_off_after_failed_grasp()
                if back_off is not None:
                    back_off.metadata["original_failure"] = result.message
                    result = back_off
                    break
            else:
                quality_rescans += 1
                if quality_rescans > MAX_QUALITY_RESCANS:
                    break
                self._minimum_observation_ts = time.time()
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
            retry_id = self._retry_object(matches)
            if retry_id is None:
                result.metadata["retry_scan"] = "No unambiguous nearby match for the original object"
                break
            object_id = retry_id
            attempted_ids.append(object_id)
            self._last_grasp_leg = None
            result = self._pick_once(object_id, planning_group)

        if result.success:
            result.metadata["grasp_attempts"] = len(attempted_ids)
            return result

        result.metadata["grasp_attempts"] = len(attempted_ids)
        result.metadata["attempted_object_ids"] = attempted_ids

        if self._holding_object or result.error_code != "GRASP_VERIFICATION_FAILED":
            return result

        # Exhausted empty closes still retreat along the completed insertion.
        if self._last_grasp_leg is not None:
            back_off = self._back_off_after_failed_grasp()
            result.metadata["folded_return"] = "completed" if back_off is None else back_off.message
            return result

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
        detector_image_size=C["perception"]["detector_image_size"],
        max_frame_age_s=C["perception"]["max_frame_age_s"],
        max_rgb_depth_skew_s=C["perception"]["max_rgb_depth_skew_s"],
        support_plane_z_m=C["bench_top_z_m"],
        object_surface_margin_m=C["perception"]["object_surface_margin_m"],
    ),
    ConfiguredGraspGenXModule.blueprint(
        gripper=C["gripper"], grasp_frame_to_tcp=C["grasp_frame_to_tcp"],
        max_candidates=C["graspgenx"]["num_grasps"], **C["graspgenx"],
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
        visualization={"backend": "viser"}, default_speed_scale=0.7, linear_speed_scale=0.7,
        **C["approach_planning"],
    ),
    OpenYamPickAndPlace.blueprint(
        planning_frame="world", max_grasp_attempts=20, yaw_policy="generated",
        grasp_verification={"empty_epsilon": C["empty_epsilon"]}, **C["grasp_execution"],
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

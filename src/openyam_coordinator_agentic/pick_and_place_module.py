"""OpenYAM pick/place execution with contact, clearance, and feedback checks."""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
import time
from typing import Any, Literal

import numpy as np
from pydantic import Field

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.manipulation.grasping.grasp_gen_x.module import RigidTransform, SweepVolumeGripperConfig
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule, PickAndPlaceModuleConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.utils.logging_config import setup_logger
from openyam_coordinator_agentic.checked_motion import CheckedManipulationSpec, pose_error
from openyam_coordinator_agentic.grasp_quality import (
    GraspQualityConfig,
    GraspQualityFilter,
    pose_matrix,
)
from openyam_coordinator_agentic.gripper_geometry import GripperGeometry

logger = setup_logger()


class GraspExecutionConfig(PickAndPlaceModuleConfig):
    """Execution policy, independent of a particular camera or bench profile."""

    max_grasp_attempts: int = Field(default=20, gt=0)
    yaw_policy: Literal["generated"] = "generated"
    max_pick_attempts: int = Field(default=3, ge=1)
    max_detection_attempts: int = Field(default=5, ge=1)
    max_quality_rescans: int = Field(default=1, ge=0)
    retry_frame_settle_s: float = Field(default=0.5, gt=0)
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
    pregrasp_fallback_offset_m: float = Field(default=0.06, gt=0, le=0.1)


class OpenYamPickAndPlaceConfig(GraspExecutionConfig):
    collision_model: Path
    gripper: SweepVolumeGripperConfig
    grasp_frame_to_tcp: RigidTransform
    support_plane_z_m: float
    max_scene_skew_s: float = Field(default=0.03, gt=0)
    quality: GraspQualityConfig = Field(default_factory=GraspQualityConfig)


class OpenYamPickAndPlaceModule(PickAndPlaceModule):
    """Use the URDF tip's -Z approach axis for approach and retreat."""

    config: OpenYamPickAndPlaceConfig
    _manipulation: CheckedManipulationSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._last_grasp_leg: tuple[PoseStamped, PoseStamped, str] | None = None
        geometry = GripperGeometry(self.config.collision_model)
        gripper = self.config.gripper.model_dump()
        geometry.validate_capture(gripper, self.config.grasp_frame_to_tcp)
        self._grasp_quality = GraspQualityFilter(
            gripper, self.config.grasp_frame_to_tcp, self.config.quality, geometry
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

    def _servo(
        self, start: PoseStamped, end: PoseStamped, planning_group: str
    ) -> SkillResult | None:
        """Reach an absolute pose with collision checking and encoder settling."""
        self._log_reached_pose("linear_start", start, planning_group)
        result = self._manipulation.move_to_contact(end, planning_group)
        failure = None if result.success else result
        if failure is None:
            failure = self._await_pose(end, planning_group)
        self._log_reached_pose("linear_end", end, planning_group)
        return failure

    def _move(self, pose: PoseStamped, planning_group: str) -> SkillResult | None:
        failure = super()._move(pose, planning_group)
        if failure is None:
            failure = self._await_pose(pose, planning_group)
        self._log_reached_pose("pregrasp_move_end", pose, planning_group)
        return failure

    def _move_checked_pregrasp(
        self, pose: PoseStamped, group: str, plan_id: str
    ) -> SkillResult | None:
        """Execute the transit whose IK branch passed the insertion check."""
        execution = self._manipulation.execute_grasp_transit(plan_id, group)
        failure = self._await_pose(pose, group) if execution.success else execution
        self._log_reached_pose("pregrasp_move_end", pose, group)
        if failure is not None:
            failure.metadata["phase"] = "pregrasp"
        return failure

    def _await_pose(
        self, target: PoseStamped, group: str, *, position_tolerance_m: float | None = None
    ) -> SkillResult | None:
        """Require three distinct, fresh encoder observations at the target."""
        tolerance = (
            self.config.contact_position_tolerance_m
            if position_tolerance_m is None
            else position_tolerance_m
        )
        deadline = time.monotonic() + self.config.settle_timeout_s
        started = time.time()
        previous_ts = 0.0
        stable = 0
        distance, angle = float("inf"), float("inf")
        while time.monotonic() < deadline:
            state = self._manipulation.get_state().groups.get(group)
            joints = None if state is None else state.joints
            reached = None if state is None else state.end_effector_pose
            fresh = (
                joints is not None
                and np.isfinite(joints.ts)
                and joints.ts > max(started, previous_ts)
                and -0.1 <= time.time() - joints.ts <= self.config.max_joint_state_age_s
            )
            if fresh and reached is not None:
                previous_ts = joints.ts
                distance, angle = pose_error(reached, target)
                slow = not joints.velocity or max(abs(v) for v in joints.velocity) < 0.04
                stable = (
                    stable + 1
                    if (
                        slow
                        and distance <= tolerance
                        and angle <= self.config.contact_orientation_tolerance_deg
                    )
                    else 0
                )
                if stable >= 3:
                    return None
            elif (
                joints is None
                or not -0.1 <= time.time() - joints.ts <= self.config.max_joint_state_age_s
            ):
                # Re-reading the same fresh snapshot is not evidence of motion.
                stable = 0
            time.sleep(0.1)
        self._manipulation.cancel()
        return SkillResult.fail(
            "EXECUTION_FAILED", f"TCP did not converge ({distance:.4f} m, {angle:.2f} deg)"
        )

    def _approach_supported_contact(
        self,
        pregrasp: PoseStamped,
        target: PoseStamped,
        group: str,
        cloud: PointCloud2,
        scene: PointCloud2,
        raw_score: float,
    ) -> tuple[PoseStamped | None, SkillResult | None]:
        """Allow a bounded miss only when the attained pose still supports grasping."""
        self._log_reached_pose("linear_start", pregrasp, group)
        result = self._manipulation.move_to_contact(target, group)
        failure = None if result.success else result
        if failure is None:
            failure = self._await_pose(
                target, group, position_tolerance_m=self.config.supported_contact_tolerance_m
            )
        self._log_reached_pose("linear_end", target, group)
        if failure is not None:
            failure.metadata["phase"] = "contact"
            return None, failure
        if failure := self._observation_failure(cloud):
            return None, failure
        state = self._manipulation.get_state().groups.get(group)
        if (
            state is None
            or state.joints is None
            or state.end_effector_pose is None
            or not -0.1 <= time.time() - state.joints.ts <= self.config.max_joint_state_age_s
        ):
            return None, SkillResult.fail("EXECUTION_FAILED", "Fresh contact feedback unavailable")
        reached = state.end_effector_pose
        distance, angle = pose_error(reached, target)
        if (
            distance > self.config.supported_contact_tolerance_m
            or angle > self.config.contact_orientation_tolerance_deg
        ):
            return None, SkillResult.fail(
                "EXECUTION_FAILED", "Contact pose moved outside the bounded settling tolerance"
            )
        # Keep the generator's raw confidence for the single-pose geometry check;
        # the quality-weighted display score is not a new model confidence.
        actual = GraspCandidate(
            Pose(position=reached.position, orientation=reached.orientation), score=raw_score
        )
        supported, report = self._grasp_quality.filter(
            GraspCandidateArray(Header(cloud.ts, cloud.frame_id), [actual]),
            cloud,
            scene,
            support_z=self.config.support_plane_z_m,
            approach_distance=0.0,
        )
        self._last_grasp_quality["attained_contact"] = report
        logger.info(
            "OpenYAM attained contact",
            position_error_m=distance,
            orientation_error_deg=angle,
            supported=bool(supported.candidates),
            quality=report,
        )
        if not supported.candidates:
            failure = SkillResult.fail(
                "CONTACT_NOT_SUPPORTED",
                "Reached pose does not retain object support/clearance; jaws kept open",
            )
            failure.metadata["contact_quality"] = report
            return None, failure
        return reached, None

    def _close_and_verify(self, planning_group: str) -> SkillResult | None:
        logger.info(
            "OpenYAM close requested",
            planning_group=planning_group,
            readback_before=self._gripper_position(planning_group),
        )
        failure = super()._close_and_verify(planning_group)
        logger.info(
            "OpenYAM close completed",
            planning_group=planning_group,
            readback_after=self._gripper_position(planning_group),
            success=failure is None,
            failure=None if failure is None else failure.message,
        )
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
            fresh = (
                joints is not None
                and np.isfinite(joints.ts)
                and -0.1 <= time.time() - joints.ts <= self.config.max_joint_state_age_s
            )
            if (
                not fresh
                or reading is None
                or not np.isfinite(reading)
                or not cfg.held_low < reading < cfg.held_high
                or closed_reading is None
                or abs(reading - closed_reading) > self.config.max_hold_aperture_change
            ):
                return SkillResult.fail(
                    "HOLD_VERIFICATION_FAILED",
                    "Jaw feedback did not confirm a sustained hold after lift",
                )
            if joints.ts <= previous_ts:
                time.sleep(0.1)
                continue
            previous_ts = joints.ts
            samples += 1
            time.sleep(0.1)
        if samples < 3:
            return SkillResult.fail(
                "HOLD_VERIFICATION_FAILED", "Insufficient distinct jaw feedback samples after lift"
            )
        logger.info(
            "OpenYAM grasp hold verified", planning_group=group, readback=reading, samples=samples
        )
        return None

    def _await_home(self, target: JointState, group: str) -> SkillResult | None:
        deadline = time.monotonic() + self.config.settle_timeout_s
        previous_ts = time.time()
        stable = 0
        errors: list[float] = []
        while time.monotonic() < deadline:
            state = self._manipulation.get_state().groups.get(group)
            joints = None if state is None else state.joints
            if (
                joints is not None
                and joints.ts > previous_ts
                and -0.1 <= time.time() - joints.ts <= self.config.max_joint_state_age_s
            ):
                previous_ts = joints.ts
                actual = dict(zip(joints.name, joints.position))
                errors = [
                    abs(actual.get(name, float("inf")) - value)
                    for name, value in zip(target.name, target.position)
                ]
                slow = not joints.velocity or max(abs(v) for v in joints.velocity) < 0.04
                stable = (
                    stable + 1
                    if (
                        errors
                        and np.all(np.isfinite(errors))
                        and slow
                        and max(errors) <= self.config.home_joint_tolerance_rad
                    )
                    else 0
                )
                if stable >= 3:
                    return None
            elif (
                joints is None
                or not -0.1 <= time.time() - joints.ts <= self.config.max_joint_state_age_s
            ):
                stable = 0
            time.sleep(0.1)
        self._manipulation.cancel()
        failure = SkillResult.fail(
            "EXECUTION_FAILED", "Home position did not settle before retry scan"
        )
        failure.metadata.update(
            phase="retry_home", joint_names=list(target.name), absolute_joint_errors_rad=errors
        )
        logger.info("OpenYAM home settling failed", **failure.metadata)
        return failure

    @staticmethod
    def _pose_record(pose: PoseStamped) -> dict[str, Any]:
        return {
            "frame_id": pose.frame_id,
            "position_m": pose.position.to_list(),
            "quaternion_xyzw": pose.orientation.to_list(),
        }

    def _log_reached_pose(self, phase: str, target: PoseStamped, group: str) -> None:
        """Record encoder-derived FK; this is not an independent physical measurement."""
        try:
            snapshot = self._manipulation.get_state()
            state = snapshot.groups.get(group)
            reached = None if state is None else state.end_effector_pose
            error = None
            if reached is not None and reached.frame_id == target.frame_id:
                error = (reached.position - target.position).to_list()
            logger.info(
                "OpenYAM grasp tracking",
                phase=phase,
                planning_group=str(group),
                snapshot_timestamp=snapshot.timestamp,
                target=self._pose_record(target),
                reached_fk=None if reached is None else self._pose_record(reached),
                reached_minus_target_m=error,
                joints=None
                if state is None or state.joints is None
                else {
                    "names": state.joints.name,
                    "positions": state.joints.position,
                    "velocities": state.joints.velocity,
                    "efforts": state.joints.effort,
                },
            )
        except Exception as exc:
            # Diagnostic availability must not change motion or retry behavior.
            logger.warning("OpenYAM grasp tracking unavailable", phase=phase, error=str(exc))

    def _back_off_after_failed_grasp(self) -> SkillResult | None:
        """Retreat, then clear the fixed camera's view before rescanning."""
        if self._last_grasp_leg is None:
            return SkillResult.fail(
                "RETRY_BACKOFF_FAILED", "No completed grasp approach to reverse"
            )
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
            return SkillResult.fail(
                "RETRY_BACKOFF_FAILED", f"Could not clear camera view: {plan.message}"
            )
        execution = self._manipulation.execute(blocking=True)
        if not execution.succeeded:
            return SkillResult.fail(
                "RETRY_BACKOFF_FAILED", f"Could not clear camera view: {execution.message}"
            )
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
        if (
            not np.isfinite(cloud.ts)
            or cloud.ts <= self._minimum_observation_ts
            or not -0.1 <= time.time() - cloud.ts <= self.config.max_object_age_s
        ):
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

    def _planning_options(
        self,
        candidates: GraspCandidateArray,
        cloud: PointCloud2,
        scene: PointCloud2,
        group: str,
    ) -> Iterator[tuple[int, GraspCandidate, PoseStamped, float, str]]:
        """Try both jaw orientations and a shorter standoff at the same contact."""
        distances = [self.config.pregrasp_offset]
        if self.config.pregrasp_fallback_offset_m < self.config.pregrasp_offset:
            distances.append(self.config.pregrasp_fallback_offset_m)
        for rank, candidate in enumerate(candidates.candidates):
            generated = self._apply_yaw_policy(
                PoseStamped(
                    ts=candidates.header.timestamp,
                    frame_id=candidates.header.frame_id,
                    position=candidate.pose.position,
                    orientation=candidate.pose.orientation,
                ),
                group,
            )
            raw_score = self._last_grasp_quality["retained"][rank]["raw_score"]
            half_turn = PoseStamped(
                ts=generated.ts,
                frame_id=generated.frame_id,
                position=generated.position,
                orientation=Quaternion.from_rotation_matrix(
                    generated.orientation.to_rotation_matrix() @ np.diag([-1.0, -1.0, 1.0])
                ),
            )
            for distance in distances:
                yield rank, candidate, generated, distance, "generated"
                # Jaw contact is symmetric, but palm meshes and robot reachability
                # need not be. Recheck geometry before giving IK the half-turn.
                alternative = GraspCandidate(
                    Pose(position=half_turn.position, orientation=half_turn.orientation),
                    score=raw_score,
                )
                supported, report = self._grasp_quality.filter(
                    GraspCandidateArray(candidates.header, [alternative]),
                    cloud,
                    scene,
                    support_z=self.config.support_plane_z_m,
                    approach_distance=distance,
                )
                logger.info(
                    "OpenYAM grasp half-turn checked",
                    rank=rank,
                    standoff_m=distance,
                    supported=bool(supported.candidates),
                    rejected=report["rejected"],
                )
                if supported.candidates:
                    yield rank, candidate, half_turn, distance, "half_turn"

    def _pick_once(self, object_id: str, planning_group: str | None) -> SkillResult:
        """Generate, gate, and execute one grasp without any perception retry policy."""
        self._clear_selection()
        if object_id not in self._objects:
            return SkillResult.fail("OBJECT_NOT_DETECTED", f"Unknown object_id: {object_id}")
        try:
            pointcloud = self._scene.get_object_pointcloud_by_object_id(object_id)
            if pointcloud is None:
                return SkillResult.fail(
                    "OBJECT_NOT_DETECTED", f"No pointcloud for object_id: {object_id}"
                )
            if failure := self._observation_failure(pointcloud):
                return failure
            scene = self._scene.get_full_scene_pointcloud(
                exclude_object_id=object_id, voxel_size=0.003
            )
            if (
                scene is None
                or not np.isfinite(scene.ts)
                or abs(scene.ts - pointcloud.ts) > self.config.max_scene_skew_s
            ):
                return SkillResult.fail(
                    "PERCEPTION_FAILED", "Matching scene geometry is unavailable"
                )
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
            candidates,
            pointcloud,
            scene,
            support_z=self.config.support_plane_z_m,
            approach_distance=self.config.pregrasp_offset,
            failed_grasps=failed_world,
        )
        logger.info(
            "OpenYAM grasp perception",
            object_id=object_id,
            frame_id=pointcloud.frame_id,
            cloud_timestamp=pointcloud.ts,
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
        self._last_grasp_quality["planning"] = []
        options = self._planning_options(candidates, pointcloud, scene, group)
        for attempt, (rank, candidate, grasp, standoff, variant) in enumerate(options):
            if attempt >= self.config.max_grasp_attempts:
                break
            if failure := self._observation_failure(pointcloud):
                return failure
            pregrasp = self._offset_pose(grasp, standoff)
            logger.info(
                "OpenYAM grasp target",
                object_id=object_id,
                rank=rank,
                score=float(candidate.score),
                grasp=self._pose_record(grasp),
                pregrasp=self._pose_record(pregrasp),
                variant=variant,
                standoff_m=standoff,
                planning_attempt=attempt,
            )
            feasibility = self._manipulation.check_grasp_path(pregrasp, grasp, group)
            self._last_grasp_quality["planning"].append(
                {
                    "rank": rank,
                    "variant": variant,
                    "standoff_m": standoff,
                    "success": feasibility.success,
                    "reason": feasibility.message,
                }
            )
            if not feasibility.success:
                logger.info(
                    "OpenYAM grasp path rejected",
                    object_id=object_id,
                    rank=rank,
                    reason=feasibility.message,
                )
                unreachable = feasibility
                continue
            if failure := self._observation_failure(pointcloud):
                self._manipulation.clear_planned_path()
                return failure
            failure = self._move_checked_pregrasp(pregrasp, group, feasibility.metadata["plan_id"])
            attained = None
            if failure is None:
                attained, failure = self._approach_supported_contact(
                    pregrasp,
                    grasp,
                    group,
                    pointcloud,
                    scene,
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
                relative = pose_matrix(
                    GraspCandidate(
                        Pose(position=grasp.position, orientation=grasp.orientation),
                        score=candidate.score,
                    )
                )
                relative[:3, 3] -= self._last_object_center
                self._failed_grasps.append(relative)
                # Only a confirmed empty close may trigger another automatic pick.
                if (
                    failure.error_code != "GRASP_VERIFICATION_FAILED"
                    or "nothing in the jaws" not in failure.message
                ):
                    self._selected_object_id = object_id
                    self._selected_grasp = grasp
                    self._holding_object = True
                return failure
            self._selected_object_id = object_id
            self._selected_grasp = grasp
            self._holding_object = True
            closed_reading = self._gripper_position(group)
            lift = PoseStamped(
                frame_id=grasp.frame_id,
                position=grasp.position + Vector3(0, 0, self.config.lift_distance_m),
                orientation=grasp.orientation,
            )
            if failure := self._servo(grasp, lift, group):
                return failure
            if failure := self._verify_lift(group, closed_reading):
                return failure
            return SkillResult.ok(
                "Pick complete",
                object_id=object_id,
                rank=rank,
                score=candidate.score,
                candidates=len(candidates.candidates),
                grasp_quality=self._last_grasp_quality,
                verification="encoder_pose_and_sustained_jaw_obstruction",
                variant=variant,
                standoff_m=standoff,
            )
        failure = unreachable or SkillResult.fail(
            "PLANNING_FAILED", "No grasp candidate was reachable"
        )
        failure.metadata["planning"] = self._last_grasp_quality["planning"]
        return failure

    @skill(uses=[CAP_MOVEMENT])
    def pick_object(self, object_id: str, planning_group: str | None = None) -> SkillResult:
        """Pick with fresh, spatially associated retries after confirmed empty closes."""
        if self._holding_object:
            return SkillResult.fail(
                "ALREADY_HOLDING_OBJECT",
                "Resolve the held or uncertain object before starting another pick",
            )
        attempted_ids = [object_id]
        self._last_grasp_leg = None
        self._failed_grasps = []
        self._minimum_observation_ts = 0.0
        self._last_object_center = None
        quality_rescans = 0
        result = self._pick_once(object_id, planning_group)
        for _attempt in range(1, self.config.max_pick_attempts):
            if (
                self._holding_object
                or result.success
                or result.error_code
                not in {"GRASP_VERIFICATION_FAILED", "INSUFFICIENT_GRASP_QUALITY"}
            ):
                break
            if result.error_code == "GRASP_VERIFICATION_FAILED":
                back_off = self._back_off_after_failed_grasp()
                if back_off is not None:
                    back_off.metadata["original_failure"] = result.message
                    result = back_off
                    break
            else:
                quality_rescans += 1
                if quality_rescans > self.config.max_quality_rescans:
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
            for _scan_attempt in range(self.config.max_detection_attempts):
                time.sleep(self.config.retry_frame_settle_s)
                scan = self.scan_objects([object_name])
                matches = (
                    [
                        item
                        for item in scan.metadata.get("objects", [])
                        if item.get("name") == object_name and item.get("object_id")
                    ]
                    if scan.success
                    else []
                )
                if matches:
                    break
            if not matches:
                result.metadata["retry_scan"] = (
                    f"no {object_name} in {self.config.max_detection_attempts} fresh scans"
                    if scan is not None and scan.success
                    else "fresh scan failed"
                    if scan is None
                    else scan.message
                )
                break
            retry_id = self._retry_object(matches)
            if retry_id is None:
                result.metadata["retry_scan"] = (
                    "No unambiguous nearby match for the original object"
                )
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

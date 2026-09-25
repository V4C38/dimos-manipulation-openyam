"""Absolute Cartesian approach planning and endpoint checks for the local stack."""

from __future__ import annotations

from collections import deque
from collections.abc import Mapping, Sequence
import threading
import time
from typing import Protocol

import numpy as np
from pydantic import Field

from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.manipulation_spec import ManipulationSpec
from dimos.manipulation.planning.kinematics.config import (
    ManipulationKinematicsConfig,
    PinkKinematicsConfig,
)
from dimos.manipulation.planning.planners.config import CartesianPathConfig
from dimos.manipulation.planning.spec.models import GeneratedPlan, PlanningResult
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def pose_error(actual: PoseStamped, target: PoseStamped) -> tuple[float, float]:
    if actual.frame_id != target.frame_id:
        return float("inf"), float("inf")
    rotation = actual.orientation.to_rotation_matrix().T @ target.orientation.to_rotation_matrix()
    angle = float(np.degrees(np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1, 1))))
    distance = actual.position.distance(target.position)
    if not np.isfinite(distance) or not np.isfinite(angle):
        return float("inf"), float("inf")
    return distance, angle


class CheckedManipulationSpec(ManipulationSpec, Protocol):
    def check_grasp_path(
        self, pregrasp: PoseStamped, grasp: PoseStamped, planning_group: str
    ) -> SkillResult: ...
    def execute_grasp_transit(self, plan_id: str, planning_group: str) -> SkillResult: ...
    def move_to_contact(self, target: PoseStamped, planning_group: str) -> SkillResult: ...
    def measured_group_samples(
        self, planning_group: str, after_ts: float
    ) -> list[tuple[JointState, PoseStamped]]: ...
    def report_tracking_failure(self, reason: str) -> None: ...


class GraspMotionConfig(BaseConfig):
    """Planning policy shared by workspace profiles and the motion module."""

    kinematics: ManipulationKinematicsConfig = Field(default_factory=PinkKinematicsConfig)
    endpoint_position_tolerance_m: float = Field(default=0.003, gt=0)
    endpoint_orientation_tolerance_deg: float = Field(default=2.0, gt=0)
    approach_speed_m_s: float = Field(default=0.08, gt=0)
    approach_acceleration_m_s2: float = Field(default=0.3, gt=0)
    approach_blend_deviation_rad: float = Field(default=0.005, gt=0, le=0.01)
    max_approach_duration_s: float = Field(default=8.0, gt=0)
    ik_position_tolerance_m: float = Field(default=0.002, gt=0, le=0.003)
    ik_orientation_tolerance_deg: float = Field(default=1.0, gt=0, le=2.0)
    ik_max_attempts: int = Field(default=10, ge=1, le=20)
    grasp_joint_limit_margin_rad: float = Field(default=0.05, ge=0, le=0.15)
    checked_transit_start_tolerance_rad: float = Field(default=0.03, gt=0, le=0.05)


class CheckedManipulationConfig(ManipulationModuleConfig, GraspMotionConfig):
    """DimOS manipulation configuration with checked grasp motion."""


class CheckedManipulationModule(ManipulationModule):
    config: CheckedManipulationConfig

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._contact_trace_lock = threading.Lock()
        self._contact_trace_joints: tuple[str, ...] = ()
        self._contact_trace_feedback: deque[
            tuple[float, list[float], list[float | None], list[float | None]]
        ] = deque(maxlen=1200)
        self._measured_feedback: deque[JointState] = deque(maxlen=600)

    def _on_joint_state(self, msg: JointState) -> None:
        aliases = self.config.joint_state_aliases
        indices = {aliases.get(name, name): index for index, name in enumerate(msg.name)}
        names = self.config.model.joint_names
        if len(msg.position) == len(msg.name) and all(name in indices for name in names):
            ordered = [indices[name] for name in names]
            sample = JointState(
                ts=msg.ts,
                frame_id=msg.frame_id,
                name=list(names),
                position=[msg.position[index] for index in ordered],
                velocity=[msg.velocity[index] for index in ordered]
                if len(msg.velocity) == len(msg.name)
                else [],
                effort=[msg.effort[index] for index in ordered]
                if len(msg.effort) == len(msg.name)
                else [],
            )
            with self._contact_trace_lock:
                self._measured_feedback.append(sample)
        super()._on_joint_state(msg)
        # Buffer the raw coordinator stream; write one log after the motion so
        # diagnostics do not add logging work to the 100 Hz feedback callback.
        with self._contact_trace_lock:
            joint_names = self._contact_trace_joints
            if not joint_names:
                return
            positions = dict(zip(msg.name, msg.position))
            velocities = dict(zip(msg.name, msg.velocity))
            efforts = dict(zip(msg.name, msg.effort))
            if all(name in positions for name in joint_names):
                self._contact_trace_feedback.append(
                    (
                        msg.ts,
                        [positions[name] for name in joint_names],
                        [velocities.get(name) for name in joint_names],
                        [efforts.get(name) for name in joint_names],
                    )
                )

    @rpc
    def measured_group_samples(
        self, planning_group: str, after_ts: float
    ) -> list[tuple[JointState, PoseStamped]]:
        """Return original feedback and FK from each corresponding joint sample."""
        if self._world_monitor is None:
            return []
        with self._contact_trace_lock:
            samples = [sample for sample in self._measured_feedback if sample.ts > after_ts]
        result = []
        for sample in samples:
            try:
                pose = self._world_monitor.get_group_ee_pose(planning_group, sample)
            except (KeyError, ValueError, RuntimeError):
                continue
            result.append((sample, pose))
        return result

    @rpc
    def report_tracking_failure(self, reason: str) -> None:
        """Flush the adapter's motor trace after a measured settling failure."""
        self._control_coordinator.emit_motor_trace(reason)

    def _discard_plan(self, plan_id: str) -> None:
        """Discard only this request's plan, preserving a concurrent replacement."""
        with self._lock:
            if self._last_plan is None or self._last_plan.plan_id != plan_id:
                return
            self._last_plan = None
        self._dismiss_preview()

    def _cartesian_config(self) -> CartesianPathConfig:
        return CartesianPathConfig(
            speed_mode="bounded",
            max_linear_speed=self.config.approach_speed_m_s,
            max_linear_acceleration=self.config.approach_acceleration_m_s2,
            max_position_error=self.config.endpoint_position_tolerance_m,
            max_orientation_error=np.radians(self.config.endpoint_orientation_tolerance_deg),
            toppra_blend_deviation=self.config.approach_blend_deviation_rad,
        )

    def _duration_failure(self, result: PlanningResult) -> str | None:
        duration = (
            None if not result.timestamps else float(result.timestamps[-1] - result.timestamps[0])
        )
        logger.info(
            "OpenYAM Cartesian timing",
            duration_s=duration,
            speed_limit_m_s=self.config.approach_speed_m_s,
            acceleration_limit_m_s2=self.config.approach_acceleration_m_s2,
            blend_deviation_rad=self.config.approach_blend_deviation_rad,
        )
        if (
            duration is None
            or not np.isfinite(duration)
            or duration <= 0
            or duration > self.config.max_approach_duration_s
        ):
            return f"Cartesian approach duration {duration} exceeds {self.config.max_approach_duration_s:.1f} s budget"
        return None

    def _endpoint_failure(self, final: JointState, target: PoseStamped, group: str) -> str | None:
        if self._world_monitor is None:
            return "Planning world unavailable"
        try:
            actual = self._world_monitor.get_group_ee_pose(group, final)
            distance, angle = pose_error(actual, target)
        except (RuntimeError, ValueError, KeyError) as exc:
            return f"Endpoint FK unavailable: {exc}"
        logger.info(
            "OpenYAM planned endpoint",
            planning_group=group,
            joint_names=final.name,
            joint_positions=final.position,
            position_error_m=distance,
            orientation_error_deg=angle,
        )
        if (
            distance > self.config.endpoint_position_tolerance_m
            or angle > self.config.endpoint_orientation_tolerance_deg
        ):
            return f"Planned endpoint misses target by {distance:.4f} m / {angle:.2f} deg"
        return None

    @staticmethod
    def _final_state(plan: GeneratedPlan) -> JointState:
        return JointState(
            name=list(plan.trajectory.joint_names),
            position=list(plan.trajectory.points[-1].positions),
        )

    def _grasp_limit_failure(self, path: Sequence[JointState]) -> str | None:
        """Avoid grasps at joint stops; this does not constrain the home pose."""
        if self._world_monitor is None:
            return "Planning world unavailable"
        space = self._world_monitor.world.get_prepared_model().joint_space
        lower, upper = space.position_limits()
        limits = dict(zip(space.names, zip(lower, upper)))
        for state in path:
            for name, value in zip(state.name, state.position):
                low, high = limits[name]
                margin = min(value - low, high - value)
                if not np.isfinite(value) or margin < self.config.grasp_joint_limit_margin_rad:
                    return f"Grasp path puts {name} within {margin:.4f} rad of a joint limit"
        return None

    def generate_plan_to_pose_targets(
        self,
        pose_targets: Mapping[str, Pose],
        auxiliary_groups: Sequence[str] = (),
        speed_scale: float | None = None,
    ) -> GeneratedPlan | None:
        if self._world_monitor is None or self._kinematics is None:
            self._record_error("Planning world unavailable")
            return None
        if not pose_targets:
            self._record_error("At least one pose target is required")
            return None
        planning = self._begin_group_planning(speed_scale)
        if planning is None:
            return None
        epoch, speed = planning
        group_ids = tuple(dict.fromkeys((*pose_targets, *auxiliary_groups)))
        resolved = self._resolve_group_plan_start(group_ids, epoch)
        if resolved is None:
            return None
        _, start = resolved
        try:
            groups = self._world_monitor.planning_groups
            targets = {
                groups.get(group): PoseStamped(
                    frame_id=self.config.world_frame,
                    position=pose.position,
                    orientation=pose.orientation,
                )
                for group, pose in pose_targets.items()
            }
            # The upstream call hard-codes 1 mm / 0.57 degrees. Keep IK tighter
            # than our endpoint gate, without requiring unnecessary precision.
            ik = self._kinematics.solve_pose_targets(
                world=self._world_monitor.world,
                pose_targets=targets,
                auxiliary_groups=tuple(groups.get(group) for group in auxiliary_groups),
                seed=start,
                check_collision=True,
                max_attempts=self.config.ik_max_attempts,
                position_tolerance=min(
                    self.config.ik_position_tolerance_m, self.config.endpoint_position_tolerance_m
                ),
                orientation_tolerance=np.radians(
                    min(
                        self.config.ik_orientation_tolerance_deg,
                        self.config.endpoint_orientation_tolerance_deg,
                    )
                ),
            )
            logger.info(
                "OpenYAM IK result",
                status=ik.status.name,
                iterations=ik.iterations,
                position_error_m=ik.position_error,
                orientation_error_deg=float(np.degrees(ik.orientation_error)),
                reason=ik.message,
            )
            if not ik.is_success() or ik.joint_state is None:
                self._fail_planning_epoch(epoch, f"IK failed: {ik.status.name}: {ik.message}")
                return None
            plan = self._plan_selected_path(group_ids, start, ik.joint_state, epoch, speed)
        except (KeyError, RuntimeError, ValueError) as exc:
            self._fail_planning_epoch(epoch, f"Pose planning failed: {exc}")
            return None
        if plan is not None:
            for group, pose in pose_targets.items():
                target = PoseStamped(
                    frame_id=self.config.world_frame,
                    position=pose.position,
                    orientation=pose.orientation,
                )
                if failure := self._endpoint_failure(self._final_state(plan), target, group):
                    self._discard_plan(plan.plan_id)
                    self._record_error(failure)
                    return None
        return plan

    @rpc
    def check_grasp_path(
        self, pregrasp: PoseStamped, grasp: PoseStamped, planning_group: str
    ) -> SkillResult:
        """Check full insertion and retain the exact transit for execution by ID."""
        keep_transit = False
        transit: GeneratedPlan | None = None
        try:
            transit = self.generate_plan_to_pose_targets({planning_group: pregrasp})
            if transit is None:
                return SkillResult.fail("PLANNING_FAILED", self.get_error())
            if self._world_monitor is None or self._planner is None:
                return SkillResult.fail("PLANNING_FAILED", "Planning world unavailable")
            selection = self._world_monitor.planning_groups.select((planning_group,))
            start = self._final_state(transit)
            if failure := self._grasp_limit_failure([start]):
                return SkillResult.fail("PLANNING_FAILED", failure)
            # IK is approximate. RoboPlan requires waypoint zero to match the
            # FK of its supplied start joints to 1e-6, not our endpoint tolerance.
            start_pose = self._world_monitor.get_group_ee_pose(planning_group, start)
            result = self._planner.plan_cartesian_path(
                world=self._world_monitor.world,
                selection=selection,
                start=start,
                targets={planning_group: (start_pose, grasp)},
                config=self._cartesian_config(),
                check_collision=True,
            )
            if not result.is_success() or not result.path:
                return SkillResult.fail("PLANNING_FAILED", result.message)
            if failure := self._grasp_limit_failure(result.path):
                return SkillResult.fail("PLANNING_FAILED", failure)
            if failure := self._duration_failure(result):
                return SkillResult.fail("PLANNING_FAILED", failure)
            if failure := self._endpoint_failure(result.path[-1], grasp, planning_group):
                return SkillResult.fail("PLANNING_FAILED", failure)
            # Do not solve transit IK again: another solution can put the arm on
            # a different branch from the one whose insertion we just checked.
            with self._lock:
                if self._last_plan is None or self._last_plan.plan_id != transit.plan_id:
                    return SkillResult.fail(
                        "PLANNING_FAILED", "Checked transit was cancelled or replaced"
                    )
                keep_transit = True
            return SkillResult.ok(
                "Transit and insertion are reachable",
                plan_id=transit.plan_id,
                approach_duration_s=float(result.timestamps[-1] - result.timestamps[0]),
            )
        except (KeyError, RuntimeError, ValueError) as exc:
            return SkillResult.fail("PLANNING_FAILED", f"Grasp path check failed: {exc}")
        finally:
            if not keep_transit and transit is not None:
                self._discard_plan(transit.plan_id)

    @rpc
    def execute_grasp_transit(self, plan_id: str, planning_group: str) -> SkillResult:
        """Use fresh feedback to reject drift since the retained transit was planned."""
        with self._lock:
            plan = self._last_plan
            if plan is None or plan.plan_id != plan_id or plan.group_ids != (planning_group,):
                return SkillResult.fail(
                    "EXECUTION_FAILED", "Checked transit was cancelled or replaced"
                )
        samples = self.measured_group_samples(planning_group, time.time() - 0.2)
        joints = samples[-1][0] if samples else None
        if joints is None or not -0.1 <= time.time() - joints.ts <= 0.5:
            self._discard_plan(plan_id)
            return SkillResult.fail("EXECUTION_FAILED", "Fresh transit-start feedback unavailable")
        actual = dict(zip(joints.name, joints.position))
        errors = [
            abs(actual.get(name, float("inf")) - value)
            for name, value in zip(plan.trajectory.joint_names, plan.trajectory.points[0].positions)
        ]
        if (
            not errors
            or not np.all(np.isfinite(errors))
            or max(errors) > self.config.checked_transit_start_tolerance_rad
        ):
            self._discard_plan(plan_id)
            return SkillResult.fail(
                "EXECUTION_FAILED", "Arm moved away from the checked transit start"
            )
        execution = self.execute(blocking=True, plan_id=plan_id)
        if not execution.succeeded:
            return SkillResult.fail("EXECUTION_FAILED", execution.message)
        return SkillResult.ok("Checked transit commands completed", plan_id=plan_id)

    def _generate_contact_plan(
        self, target: PoseStamped, planning_group: str
    ) -> GeneratedPlan | None:
        """Build absolute waypoints from the same joint snapshot as the planner."""
        if self._world_monitor is None or self._planner is None:
            self._record_error("Planning world unavailable")
            return None
        planning = self._begin_group_planning(speed_scale=1.0)
        if planning is None:
            return None
        epoch, speed = planning
        group_ids = (planning_group,)
        resolved = self._resolve_group_plan_start(group_ids, epoch)
        if resolved is None:
            return None
        selection, start = resolved
        try:
            if not -0.1 <= time.time() - start.ts <= 0.5:
                raise ValueError("Current joint feedback is stale")
            start_pose = self._world_monitor.get_group_ee_pose(planning_group, start)
            # generate_cartesian_plan() resolves a newer joint snapshot internally.
            # Resolve once here so feedback arriving between RPCs cannot invalidate
            # waypoint zero. The destination remains the requested absolute pose.
            result = self._planner.plan_cartesian_path(
                world=self._world_monitor.world,
                selection=selection,
                start=start,
                targets={planning_group: (start_pose, target)},
                config=self._cartesian_config(),
                check_collision=True,
            )
        except (KeyError, RuntimeError, ValueError) as exc:
            self._fail_planning_epoch(epoch, f"Cartesian planning failed: {exc}")
            return None
        if not result.is_success() or not result.path:
            self._fail_planning_epoch(
                epoch, f"Cartesian planning failed: {result.status.name}: {result.message}"
            )
            return None
        if failure := self._duration_failure(result):
            self._fail_planning_epoch(epoch, failure)
            return None
        # Retain the base module's cancellation epoch and trajectory validation.
        return self._store_generated_plan(group_ids, result, epoch, speed)

    @rpc
    def move_to_contact(self, target: PoseStamped, planning_group: str) -> SkillResult:
        """Plan an absolute pose, including orientation, and check its endpoint."""
        samples = self.measured_group_samples(planning_group, time.time() - 0.2)
        if not samples:
            return SkillResult.fail("EXECUTION_FAILED", "Fresh current TCP/joints unavailable")
        if not -0.1 <= time.time() - samples[-1][0].ts <= 0.5:
            return SkillResult.fail("EXECUTION_FAILED", "Current joint feedback is stale")
        plan = self._generate_contact_plan(target, planning_group)
        if plan is None:
            return SkillResult.fail("PLANNING_FAILED", self.get_error())
        if failure := self._endpoint_failure(self._final_state(plan), target, planning_group):
            self._discard_plan(plan.plan_id)
            return SkillResult.fail("PLANNING_FAILED", failure)
        joint_names = tuple(plan.trajectory.joint_names)
        with self._contact_trace_lock:
            self._contact_trace_joints = joint_names
            self._contact_trace_feedback.clear()
        try:
            execution = self.execute(blocking=True, plan_id=plan.plan_id)
        except Exception:
            with self._contact_trace_lock:
                self._contact_trace_joints = ()
                self._contact_trace_feedback.clear()
            raise
        with self._contact_trace_lock:
            feedback = list(self._contact_trace_feedback)
            self._contact_trace_joints = ()
            self._contact_trace_feedback.clear()
        if not execution.succeeded:
            logger.warning(
                "OpenYAM failed Cartesian feedback trace",
                plan_id=plan.plan_id,
                joint_names=joint_names,
                feedback=feedback[-300:],
            )
            try:
                self.report_tracking_failure("Cartesian execution failed")
            except Exception as exc:
                logger.warning("OpenYAM motor trace unavailable", error=str(exc))
        if not execution.succeeded:
            return SkillResult.fail("EXECUTION_FAILED", execution.message)
        return SkillResult.ok("Contact trajectory commands completed", plan_id=plan.plan_id)

"""Absolute Cartesian approach planning and endpoint checks for the local stack."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Protocol
import time

import numpy as np
from pydantic import Field

from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.manipulation_spec import ManipulationSpec
from dimos.manipulation.planning.planners.config import CartesianPathConfig
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def pose_error(actual: PoseStamped, target: PoseStamped) -> tuple[float, float]:
    if actual.frame_id != target.frame_id:
        return float('inf'), float('inf')
    rotation = actual.orientation.to_rotation_matrix().T @ target.orientation.to_rotation_matrix()
    angle = float(np.degrees(np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1, 1))))
    distance = actual.position.distance(target.position)
    if not np.isfinite(distance) or not np.isfinite(angle):
        return float('inf'), float('inf')
    return distance, angle


class CheckedManipulationSpec(ManipulationSpec, Protocol):
    def check_grasp_path(self, pregrasp: PoseStamped, grasp: PoseStamped, planning_group: str) -> SkillResult: ...
    def move_to_contact(self, target: PoseStamped, planning_group: str) -> SkillResult: ...


class CheckedManipulationConfig(ManipulationModuleConfig):
    endpoint_position_tolerance_m: float = Field(default=0.003, gt=0)
    endpoint_orientation_tolerance_deg: float = Field(default=2.0, gt=0)
    approach_speed_m_s: float = Field(default=0.08, gt=0)
    approach_acceleration_m_s2: float = Field(default=0.3, gt=0)
    approach_blend_deviation_rad: float = Field(default=0.005, gt=0, le=0.01)
    max_approach_duration_s: float = Field(default=8.0, gt=0)


class CheckedManipulationModule(ManipulationModule):
    config: CheckedManipulationConfig

    def _cartesian_config(self) -> CartesianPathConfig:
        return CartesianPathConfig(
            speed_mode='bounded', max_linear_speed=self.config.approach_speed_m_s,
            max_linear_acceleration=self.config.approach_acceleration_m_s2,
            max_position_error=self.config.endpoint_position_tolerance_m,
            max_orientation_error=np.radians(self.config.endpoint_orientation_tolerance_deg),
            toppra_blend_deviation=self.config.approach_blend_deviation_rad,
        )

    def _duration_failure(self, result) -> str | None:
        duration = None if not result.timestamps else float(result.timestamps[-1] - result.timestamps[0])
        logger.info('OpenYAM Cartesian timing', duration_s=duration,
                    speed_limit_m_s=self.config.approach_speed_m_s,
                    acceleration_limit_m_s2=self.config.approach_acceleration_m_s2,
                    blend_deviation_rad=self.config.approach_blend_deviation_rad)
        if duration is None or not np.isfinite(duration) or duration > self.config.max_approach_duration_s:
            return f'Cartesian approach duration {duration} exceeds {self.config.max_approach_duration_s:.1f} s budget'
        return None

    def _endpoint_failure(self, final: JointState, target: PoseStamped, group: str) -> str | None:
        if self._world_monitor is None:
            return 'Planning world unavailable'
        try:
            actual = self._world_monitor.get_group_ee_pose(group, final)
            distance, angle = pose_error(actual, target)
        except (RuntimeError, ValueError, KeyError) as exc:
            return f'Endpoint FK unavailable: {exc}'
        logger.info('OpenYAM planned endpoint', planning_group=group,
                    joint_names=final.name, joint_positions=final.position,
                    position_error_m=distance, orientation_error_deg=angle)
        if (distance > self.config.endpoint_position_tolerance_m
                or angle > self.config.endpoint_orientation_tolerance_deg):
            return f'Planned endpoint misses target by {distance:.4f} m / {angle:.2f} deg'
        return None

    @staticmethod
    def _final_state(plan: GeneratedPlan) -> JointState:
        return JointState(name=list(plan.trajectory.joint_names),
                          position=list(plan.trajectory.points[-1].positions))

    def generate_plan_to_pose_targets(
        self, pose_targets: Mapping[str, Pose], auxiliary_groups: Sequence[str] = (),
        speed_scale: float | None = None,
    ) -> GeneratedPlan | None:
        plan = super().generate_plan_to_pose_targets(pose_targets, auxiliary_groups=auxiliary_groups,
                                                     speed_scale=speed_scale)
        if plan is not None:
            for group, pose in pose_targets.items():
                target = PoseStamped(frame_id=self.config.world_frame,
                                     position=pose.position, orientation=pose.orientation)
                if failure := self._endpoint_failure(self._final_state(plan), target, group):
                    self.clear_planned_path()
                    self._fail(failure)
                    return None
        return plan

    @rpc
    def check_grasp_path(self, pregrasp: PoseStamped, grasp: PoseStamped, planning_group: str) -> SkillResult:
        """Check transit and full insertion without sending hardware commands."""
        try:
            transit = self.generate_plan_to_pose_targets({planning_group: pregrasp})
            if transit is None:
                return SkillResult.fail('PLANNING_FAILED', self.get_error())
            if self._world_monitor is None or self._planner is None:
                return SkillResult.fail('PLANNING_FAILED', 'Planning world unavailable')
            selection = self._world_monitor.planning_groups.select((planning_group,))
            start = self._final_state(transit)
            # IK is approximate. RoboPlan requires waypoint zero to match the
            # FK of its supplied start joints to 1e-6, not our endpoint tolerance.
            start_pose = self._world_monitor.get_group_ee_pose(planning_group, start)
            result = self._planner.plan_cartesian_path(
                world=self._world_monitor.world, selection=selection,
                start=start, targets={planning_group: (start_pose, pregrasp, grasp)},
                config=self._cartesian_config(), check_collision=True,
            )
            if not result.is_success() or not result.path:
                return SkillResult.fail('PLANNING_FAILED', result.message)
            if failure := self._duration_failure(result):
                return SkillResult.fail('PLANNING_FAILED', failure)
            if failure := self._endpoint_failure(result.path[-1], grasp, planning_group):
                return SkillResult.fail('PLANNING_FAILED', failure)
            return SkillResult.ok('Transit and insertion are reachable')
        finally:
            self.clear_planned_path()

    def _generate_contact_plan(self, target: PoseStamped, planning_group: str) -> GeneratedPlan | None:
        """Build absolute waypoints from the same joint snapshot as the planner."""
        if self._world_monitor is None or self._planner is None:
            self._record_error('Planning world unavailable')
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
                raise ValueError('Current joint feedback is stale')
            start_pose = self._world_monitor.get_group_ee_pose(planning_group, start)
            # generate_cartesian_plan() resolves a newer joint snapshot internally.
            # Resolve once here so feedback arriving between RPCs cannot invalidate
            # waypoint zero. The destination remains the requested absolute pose.
            result = self._planner.plan_cartesian_path(
                world=self._world_monitor.world, selection=selection, start=start,
                targets={planning_group: (start_pose, target)},
                config=self._cartesian_config(), check_collision=True,
            )
        except (KeyError, RuntimeError, ValueError) as exc:
            self._fail_planning_epoch(epoch, f'Cartesian planning failed: {exc}')
            return None
        if not result.is_success() or not result.path:
            self._fail_planning_epoch(epoch, f'Cartesian planning failed: {result.status.name}: {result.message}')
            return None
        if failure := self._duration_failure(result):
            self._fail_planning_epoch(epoch, failure)
            return None
        # Retain the base module's cancellation epoch and trajectory validation.
        return self._store_generated_plan(group_ids, result, epoch, speed)

    @rpc
    def move_to_contact(self, target: PoseStamped, planning_group: str) -> SkillResult:
        """Plan an absolute pose, including orientation, and check its endpoint."""
        state = self.get_state().groups.get(planning_group)
        if state is None or state.joints is None or state.end_effector_pose is None:
            return SkillResult.fail('EXECUTION_FAILED', 'Fresh current TCP/joints unavailable')
        if not -0.1 <= time.time() - state.joints.ts <= 0.5:
            return SkillResult.fail('EXECUTION_FAILED', 'Current joint feedback is stale')
        plan = self._generate_contact_plan(target, planning_group)
        if plan is None:
            return SkillResult.fail('PLANNING_FAILED', self.get_error())
        if failure := self._endpoint_failure(self._final_state(plan), target, planning_group):
            self.clear_planned_path()
            return SkillResult.fail('PLANNING_FAILED', failure)
        execution = self.execute(blocking=True, plan_id=plan.plan_id)
        if not execution.succeeded:
            return SkillResult.fail('EXECUTION_FAILED', execution.message)
        return SkillResult.ok('Contact trajectory commands completed', plan_id=plan.plan_id)

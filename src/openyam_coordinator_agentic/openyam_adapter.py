"""Repository-local motor trace for the physical OpenYAM adapter."""

from __future__ import annotations

from collections import deque
import math
import threading
import time

from dimos.control.components import HardwareComponent
from dimos.control.coordinator import ControlCoordinator
from dimos.control.task import BaseControlTask, CoordinatorState, JointCommandOutput, ResourceClaim
from dimos.core.core import rpc
from dimos.hardware.manipulators.spec import ControlMode
from dimos.hardware.whole_body.openyam_damiao.adapter import OpenYamDamiaoAdapter
from dimos.hardware.whole_body.spec import MotorCommand, MotorState, WholeBodyAdapter
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class TracedOpenYamAdapter(OpenYamDamiaoAdapter):
    """Keep the values passed to MIT control alongside the latest motor readback."""

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self._trace: deque[dict[str, object]] = deque(maxlen=200)
        self._trace_lock = threading.Lock()
        self._feedback: list[MotorState] | None = None
        self._feedback_time = 0.0
        self._gravity: list[float] = []
        self._previous_feedback: list[MotorState] | None = None
        self._offset_since: float | None = None
        self._last_trace_time = 0.0
        self.last_sent_arm_positions: tuple[float, ...] | None = None

    def _gravity_torques(self):  # type: ignore[no-untyped-def]
        gravity = super()._gravity_torques()
        self._gravity = gravity.tolist()
        return gravity

    def read_motor_states(self) -> list[MotorState]:
        states = super().read_motor_states()
        self._previous_feedback = self._feedback
        self._feedback = states
        self._feedback_time = time.time()
        return states

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        # The base adapter adds gravity torque immediately before mit_control.
        sent = super().write_motor_commands(commands)
        now = time.time()
        arm_count = len(self.kinematic_joint_names)
        if sent:
            self.last_sent_arm_positions = tuple(command.q for command in commands[:arm_count])
        feedback = self._feedback
        gravity = self._gravity[:arm_count]
        with self._trace_lock:
            self._trace.append(
                {
                    "ts": now,
                    "sent": sent,
                    "feedback_ts": self._feedback_time,
                    "joint_names": self.kinematic_joint_names,
                    "command_q": [command.q for command in commands[:arm_count]],
                    "command_dq": [command.dq for command in commands[:arm_count]],
                    "kp": [command.kp for command in commands[:arm_count]],
                    "kd": [command.kd for command in commands[:arm_count]],
                    "feedforward_tau": [command.tau for command in commands[:arm_count]],
                    "gravity_tau": gravity,
                    "motor_tau": [command.tau + value for command, value in zip(commands, gravity)],
                    "feedback_q": []
                    if feedback is None
                    else [state.q for state in feedback[:arm_count]],
                    "feedback_dq": []
                    if feedback is None
                    else [state.dq for state in feedback[:arm_count]],
                    "feedback_tau": []
                    if feedback is None
                    else [state.tau for state in feedback[:arm_count]],
                }
            )
        if not sent:
            self._emit_trace("motor_command_rejected", now)
        elif feedback is not None and now - self._feedback_time < 0.1:
            step = (
                max(
                    abs(current.q - previous.q)
                    for current, previous in zip(
                        feedback[:arm_count], self._previous_feedback[:arm_count]
                    )
                )
                if self._previous_feedback is not None
                else 0.0
            )
            if step >= 0.015:
                self._emit_trace("rapid_joint_motion", now)
            error = max(
                abs(command.q - state.q)
                for command, state in zip(commands[:arm_count], feedback[:arm_count])
            )
            if error >= 0.01:
                if self._offset_since is None:
                    self._offset_since = now
                elif now - self._offset_since >= 0.5:
                    self._emit_trace("persistent_tracking_error", now)
            else:
                self._offset_since = None
        return sent

    def _emit_trace(self, reason: str, now: float) -> None:
        if now - self._last_trace_time < 2.0:
            return
        self._last_trace_time = now
        with self._trace_lock:
            samples = list(self._trace)
        threading.Thread(
            target=logger.warning,
            args=("OpenYAM motor command and feedback trace",),
            kwargs={"reason": reason, "hardware_id": self._hardware_id, "samples": samples},
            daemon=True,
        ).start()

    def emit_trace(self, reason: str) -> None:
        """Flush the bounded recent trace for an upstream tracking failure."""
        self._last_trace_time = 0.0
        self._emit_trace(reason, time.time())


class OpenYamHoldTask(BaseControlTask):
    """Refresh the last accepted arm target between planned trajectories."""

    def __init__(self, adapter: TracedOpenYamAdapter) -> None:
        self._adapter = adapter
        self._joints = list(adapter.kinematic_joint_names)
        self._initial_target: tuple[float, ...] | None = None

    @property
    def name(self) -> str:
        return "openyam_hold"

    def claim(self) -> ResourceClaim:
        return ResourceClaim(frozenset(self._joints), priority=1, mode=ControlMode.SERVO_POSITION)

    def is_active(self) -> bool:
        return True

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        target = self._adapter.last_sent_arm_positions
        if target is None:
            if self._initial_target is None:
                positions = [state.joints.get_position(name) for name in self._joints]
                if any(value is None or not math.isfinite(value) for value in positions):
                    return None
                self._initial_target = tuple(float(value) for value in positions)
            target = self._initial_target
        return JointCommandOutput(
            joint_names=self._joints, positions=list(target), mode=ControlMode.SERVO_POSITION
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        pass


class OpenYamDiagnosticCoordinator(ControlCoordinator):
    """Select the local diagnostic wrapper for the physical OpenYAM only."""

    def _create_whole_body_adapter(self, component: HardwareComponent) -> WholeBodyAdapter:
        if component.adapter_type == "openyam_traced_damiao":
            return TracedOpenYamAdapter(
                dof=len(component.joints),
                hardware_id=component.hardware_id,
                address=component.address,
                domain_id=component.domain_id,
                **component.adapter_kwargs,
            )
        return super()._create_whole_body_adapter(component)

    def _setup_from_config(self) -> None:
        super()._setup_from_config()
        hardware = self._hardware.get("openyam")
        adapter = None if hardware is None else hardware.adapter
        if isinstance(adapter, TracedOpenYamAdapter):
            self.add_task(OpenYamHoldTask(adapter))

    @rpc
    def emit_motor_trace(self, reason: str) -> bool:
        with self._hardware_lock:
            hardware = self._hardware.get("openyam")
            adapter = None if hardware is None else hardware.adapter
        if not isinstance(adapter, TracedOpenYamAdapter):
            return False
        adapter.emit_trace(reason)
        return True

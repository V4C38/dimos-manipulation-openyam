"""The contact gate must reject rapid motion even at the requested TCP pose."""

from types import SimpleNamespace

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from openyam_coordinator_agentic import pick_and_place_module as pick_module


class Clock:
    def __init__(self) -> None:
        self.now = 100.0

    def time(self) -> float:
        return self.now

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += duration


class MeasuredArm:
    def __init__(self, samples: list[tuple[JointState, PoseStamped]], clock: Clock) -> None:
        self.samples = samples
        self.clock = clock
        self.cancelled = False
        self.reported = False

    def measured_group_samples(
        self, group: str, after_ts: float
    ) -> list[tuple[JointState, PoseStamped]]:
        assert group == "manipulator"
        return [
            observation
            for observation in self.samples
            if after_ts < observation[0].ts <= self.clock.now
        ]

    def cancel(self) -> None:
        self.cancelled = True

    def report_tracking_failure(self, reason: str) -> None:
        self.reported = True


def _settle(monkeypatch, positions: list[float]) -> tuple[object, MeasuredArm]:
    clock = Clock()
    monkeypatch.setattr(pick_module, "time", clock)
    target = PoseStamped(frame_id="world")
    samples = [
        (
            JointState(
                ts=100.0 + index * 0.01,
                name=["joint"],
                position=[position],
                velocity=[0.0],
                effort=[0.0],
            ),
            target,
        )
        for index, position in enumerate(positions, start=1)
    ]
    arm = MeasuredArm(samples, clock)
    config = SimpleNamespace(
        contact_position_tolerance_m=0.005,
        contact_orientation_tolerance_deg=3.0,
        settle_timeout_s=0.25,
        max_joint_state_age_s=0.5,
        max_feedback_gap_s=0.05,
        max_stationary_step_rad=0.004,
        max_stationary_velocity_rad_s=0.04,
        stationary_interval_s=0.1,
    )
    module = SimpleNamespace(config=config, _manipulation=arm)
    failure = pick_module.OpenYamPickAndPlaceModule._await_pose(module, target, "manipulator")
    return failure, arm


def test_continuous_stationary_feedback_allows_contact(monkeypatch) -> None:
    failure, arm = _settle(monkeypatch, [0.0] * 30)
    assert failure is None
    assert not arm.cancelled


def test_rapid_joint_steps_keep_jaws_open(monkeypatch) -> None:
    failure, arm = _settle(monkeypatch, [0.0, 0.042] * 15)
    assert failure is not None
    assert arm.cancelled
    assert arm.reported

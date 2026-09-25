"""The local arm hold continues to supply targets after a trajectory ends."""

from dimos.control.task import ControlTask, CoordinatorState, JointStateSnapshot
from openyam_coordinator_agentic.openyam_adapter import OpenYamHoldTask, TracedOpenYamAdapter


def test_hold_starts_at_feedback_then_repeats_last_sent_target() -> None:
    adapter = TracedOpenYamAdapter(dof=7)
    task = OpenYamHoldTask(adapter)
    assert isinstance(task, ControlTask)
    names = adapter.kinematic_joint_names
    state = CoordinatorState(joints=JointStateSnapshot(joint_positions=dict(zip(names, [0.1] * 6))))

    initial = task.compute(state)
    assert initial is not None
    assert initial.positions == [0.1] * 6

    adapter.last_sent_arm_positions = (0.2,) * 6
    state.joints.joint_positions = dict(zip(names, [0.15] * 6))
    held = task.compute(state)
    assert held is not None
    assert held.positions == [0.2] * 6
    assert task.claim().priority < 10  # The trajectory task wins while moving.

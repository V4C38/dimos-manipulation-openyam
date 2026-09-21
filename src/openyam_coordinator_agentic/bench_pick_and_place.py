"""Guide-aligned external OpenYAM apple pick-and-place blueprints."""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any

from dimos.agents.mcp.mcp_client import McpClient, McpClientConfig
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.skill_result import SkillResult
from dimos.control.coordinator import TaskConfig
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
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.openyam.config import OPENYAM_GRIPPER_JOINT, openyam_hardware

from openyam_coordinator_agentic.collision_model import model_config
from openyam_coordinator_agentic.collision_safety import filter_static_bench_overlap, require_collision_coverage
from openyam_coordinator_agentic.local_bench_geometry import bench_obstacles

# The guide specifies 848x480@15 for USB 3.  This D435i has been observed to
# run reliably over the current USB 2.1 cable at this lower supported profile.
USB21_WIDTH = 640
USB21_HEIGHT = 480
USB21_FPS = 6

# Unlike the upstream observation pose ([0, 1.047, 1.047, 0, 0, 0]), zero is
# the folded, power-off-safe configuration for this OpenYAM: joints 2 and 3
# are at their lower limits.  Keep this local to the calibrated bench setup.
FOLDED_POWER_OFF_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
MAX_GRASP_ATTEMPTS = 3
MAX_DETECTION_ATTEMPTS = 5
# At 6 FPS, this waits for three post-retreat RGB-D frames before the next
# on-request scan selects its latest aligned frame.
RETRY_FRAME_SETTLE_S = 0.5


def bench_config() -> dict[str, Any]:
    """Load measured local values, rejecting every guide-required null."""
    config = json.loads(Path(os.environ["OPENYAM_BENCH_CONFIG"]).read_text(encoding="utf-8"))
    required = (
        "camera_serial", "camera_translation_m", "camera_quaternion_xyzw", "gripper",
        "grasp_frame_to_tcp", "bench_center_m", "bench_size_m", "bench_quaternion_xyzw",
        "camera_wall", "empty_epsilon", "place_tcp_m",
    )
    missing = [name for name in required if config.get(name) is None]
    gripper = config.get("gripper") or {}
    missing.extend(
        f"gripper.{name}" for name in (
            "extents_open", "offset_open", "extents_half_open", "offset_half_open", "fingertip_depth"
        ) if gripper.get(name) is None
    )
    if missing:
        raise ValueError("Complete measured OpenYAM bench fields: " + ", ".join(missing))
    return config


C = bench_config()
_mount = Transform(
    translation=Vector3(*C["camera_translation_m"]),
    rotation=Quaternion(*C["camera_quaternion_xyzw"]),
    frame_id="world", child_frame_id="camera_link",
)


class OpenYamBenchMount(StaticTfPublisher):
    def transforms(self):
        return [_mount]


class CollisionAwareBenchManipulation(ManipulationModule):
    """Planner containing the measured bench and camera-side obstacle."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        require_collision_coverage(self.config.model.model.load().xml)

    def _initialize_planning(self) -> None:
        super()._initialize_planning()
        assert self._world_monitor is not None
        for obstacle in bench_obstacles(C):
            self._world_monitor.add_obstacle(obstacle)
        filter_static_bench_overlap(self._world_monitor.world)


class OpenYamPickAndPlace(PickAndPlaceModule):
    """Use the URDF tip's -Z approach axis for approach and retreat."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._last_grasp_leg: tuple[PoseStamped, PoseStamped, Any] | None = None

    @staticmethod
    def _offset_pose(pose: PoseStamped, offset: float) -> PoseStamped:
        # GraspGenX approaches along +Z; OpenYAM's gripper_tip approaches
        # along -Z. Retreat is therefore +Z in the planning tip frame.
        return PoseStamped(
            ts=pose.ts,
            frame_id=pose.frame_id,
            position=pose.position + pose.orientation.rotate_vector(Vector3(0, 0, offset)),
            orientation=pose.orientation,
        )

    def _servo(self, start: PoseStamped, end: PoseStamped, planning_group: Any) -> SkillResult | None:
        """Remember the final approach so a failed close can reverse it."""
        self._last_grasp_leg = (start, end, planning_group)
        return super()._servo(start, end, planning_group)

    def _back_off_after_failed_grasp(self) -> SkillResult | None:
        """Reverse the just-completed straight approach to clear the camera view."""
        if self._last_grasp_leg is None:
            return SkillResult.fail("RETRY_BACKOFF_FAILED", "No completed grasp approach to reverse")
        pregrasp, grasp, group = self._last_grasp_leg
        return super()._servo(grasp, pregrasp, group)

    @skill(uses=[CAP_MOVEMENT])
    def pick_object(self, object_id: str, planning_group: str | None = None) -> SkillResult:
        """Pick with two camera-clear, fresh-perception retries, then park."""
        attempted_ids = [object_id]
        self._last_grasp_leg = None
        result = super().pick_object(object_id, planning_group)
        for _attempt in range(1, MAX_GRASP_ATTEMPTS):
            if result.success or result.error_code != "GRASP_VERIFICATION_FAILED":
                break
            back_off = self._back_off_after_failed_grasp()
            if back_off is not None:
                result.metadata["retry_backoff"] = back_off.message
                break
            # scan_objects processes the latest aligned RGB-D frame. Wait for
            # frames acquired after the arm has cleared the object, rather than
            # reusing the pre-grasp image that drove the failed attempt.
            apples: list[dict[str, Any]] = []
            scan: SkillResult | None = None
            for _scan_attempt in range(MAX_DETECTION_ATTEMPTS):
                time.sleep(RETRY_FRAME_SETTLE_S)
                scan = self.scan_objects(["apple"])
                apples = [
                    item for item in scan.metadata.get("objects", [])
                    if item.get("name") == "apple" and item.get("object_id")
                ] if scan.success else []
                if apples:
                    break
            if not apples:
                result.metadata["retry_scan"] = (
                    "no apple in five fresh scans"
                    if scan is not None and scan.success
                    else "fresh scan failed" if scan is None else scan.message
                )
                break
            # The scene module returns objects observed in this scan only, but
            # the detector can emit overlapping apple hypotheses. They all
            # originate from the new frame; use its first returned detection
            # rather than treating a duplicate hypothesis as a terminal error.
            object_id = str(apples[0]["object_id"])
            attempted_ids.append(object_id)
            self._last_grasp_leg = None
            result = super().pick_object(object_id, planning_group)

        if result.success:
            result.metadata["grasp_attempts"] = len(attempted_ids)
            return result

        result.metadata["grasp_attempts"] = len(attempted_ids)
        result.metadata["attempted_object_ids"] = attempted_ids

        group = self._resolve_group(planning_group)
        if group is None:
            result.metadata["folded_return"] = "skipped: planning group unavailable"
            return result
        state = self._manipulation.get_state().groups.get(group)
        target = None if state is None else state.joint_presets.get("home")
        if target is None:
            result.metadata["folded_return"] = "skipped: folded home preset unavailable"
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
        serial_number=C["camera_serial"], width=USB21_WIDTH, height=USB21_HEIGHT, fps=USB21_FPS,
        align_depth_to_color=True, enable_pointcloud=False, enable_imu=False,
    ),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world", detector_backend="yoloe", segmentation_backend="yolo",
        prompt_mode=YoloePromptMode.PROMPT, detect_on_request=True,
        min_detections_for_permanent=1, distance_threshold=0.05, use_aabb=True,
        max_obstacle_width=0.0,
    ),
    GraspGenXModule.blueprint(
        gripper=C["gripper"], grasp_frame_to_tcp=C["grasp_frame_to_tcp"], max_candidates=20,
    ),
)

openyam_bench_proposals = autoconnect(
    *_sensing, OpenYamBenchMount.blueprint(),
).global_config(n_workers=4)

_model = model_config(Path(os.environ["OPENYAM_COLLISION_MODEL"])).model_copy(
    update={
        "base_pose": PoseStamped(frame_id="world"),
        "home_joints": list(FOLDED_POWER_OFF_JOINTS),
    }
)
_hardware = openyam_hardware()
openyam_bench_grasp = autoconnect(
    *_sensing,
    OpenYamBenchMount.blueprint(),
    CollisionAwareBenchManipulation.blueprint(
        model=_model, world_frame="world", static_transforms=[_mount],
        visualization={"backend": "viser"}, default_speed_scale=0.1, linear_speed_scale=0.1,
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
).global_config(n_workers=6)

OPENYAM_BENCH_AGENT_PROMPT = f"""\
You control one fixed-camera OpenYAM manipulation test.

The only requested episode is: pick up the apple, place it 10 cm further away
on the table, then return home.  For this calibrated bench, the supplied
release TCP is {C["place_tcp_m"]!r}; it is the measured position 10 cm from
the test apple and is the only release coordinate you may use.

Call scan_objects(["apple"]) and retry a no-detection up to five times. Use
an exact apple object ID from that scan. Then call pick_object with that ID. If it succeeds, call place_at with the supplied
release TCP, then call go_home. Here, go_home means the folded, power-off-safe
pose. A failed grasp verification automatically rescans the apple and generates
a fresh grasp up to two more times, then returns to that same folded pose if
all three attempts fail. Do not issue your own retries, reset faults, choose a
different object, infer coordinates, or open the gripper except through the
successful place_at call. Report the final failure and stop.
"""

openyam_bench_agentic = autoconnect(
    openyam_bench_grasp,
    ManipulationSkills.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(
        system_prompt=OPENYAM_BENCH_AGENT_PROMPT,
        model=os.environ.get("OPENYAM_LLM_MODEL", McpClientConfig().model),
    ),
).global_config(n_workers=6)

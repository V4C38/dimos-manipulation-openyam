# Workspace configuration

`openyam_bench.json` is this bench's measured profile. The grasp blueprint reads it
through `OPENYAM_WORKSPACE_CONFIG` at startup. `yam_collision.urdf` and its
`yam_collision.manifest.json` are the verified collision model;
`collision-meshes/` holds its finger-sweep meshes. Keep these files together.
The existing finger collision sweeps include a 20 mm distal extension. The grasp
capture band now ends at their approximately 166.84 mm distal extent, measured
from the gripper body. Its length remains 96.002 mm. The TCP stays at 120 mm;
it is a datum inside the fingertip envelope, not the physical fingertip.
Generated grasp +Z is gripper −Z and generated X is gripper Y, with a shared
body origin. `grasp_frame_to_tcp` encodes this convention. The grasp module
checks capture depth against the loaded collision geometry during construction.
These changes retain the existing collision model; the contact-band width and
thickness and the physical meaning of the extension have not been remeasured.

`calibrate_workspace.py` finds the AprilTag in the fixed RGB-D camera stream and
uses its measured position relative to the arm axis. In arm coordinates +X is
forward, +Y is left, and +Z is up. The active calibration places the tag center
120 mm right of the arm axis (Y=-0.120 m), measured center-to-center. The tag and arm
datum are 20 mm above the bench; the bench top is Z=-0.020 m. No wrist camera,
arm motion, or plate-edge selection is involved.
When the RGB-D stream is already running, use:

```bash
../dimos/.venv/bin/python workspace-config/calibrate_workspace.py
../dimos/.venv/bin/python workspace-config/calibrate_workspace.py --apply
```

Run this while the RGB-D camera is streaming. The first command prints the
measured alignment; `--apply` updates the JSON. Restart the blueprint to load it.

The tag is 70 mm square overall; pose fitting uses its **56 mm black square**.
The bench footprint, camera wall, and placement point are separate measured site
data. Grasp targets use the generated contact pose without a height adjustment;
the pregrasp is offset along TCP +Z, with insertion along TCP −Z. A successful
close is followed by a separate vertical lift.
The collision model uses Git HEAD's manufacturer joint geometry. Earlier
joint-zero offsets proved to create a false collision at the folded arm pose;
their historical values remain in the calibration records, not the active model.
The `arm_control` gains improve tracking of commanded poses.
Grasp runs log `OpenYAM grasp perception`, `OpenYAM grasp target`, and
`OpenYAM grasp tracking` records in the DimOS run's `main.jsonl`. These include
the object cloud bounds, chosen TCP target, and encoder-derived reached pose
before and after the absolute Cartesian approach. `reached_minus_target_m` is in the
target frame (normally world). It exposes tracking error, but cannot measure
errors in joint zeros, the physical TCP, or camera-to-base calibration. A small
AprilTag fit residual alone does not establish absolute arm accuracy.
Object tracking retains stable IDs, but replaces each object's grasp point cloud
with its latest observation so retry scans cannot retain an object's old position.
The folded zero-joint `home_joints` match Git HEAD's power-off-safe resting pose.

## Grasp settings and execution

- RGB-D remains aligned at 1280×720, 6 FPS. YOLOE now uses inference size 1280
  and native image masks (`retina_masks=True`) to undo letterboxing correctly.
  Object clouds retain 2 mm voxels. Frames older than 2 seconds or RGB/depth
  skew above 30 ms are rejected. Points within 3 mm of/below the bench are removed
  from object geometry; no depth or calibration offset is applied.
- `graspgenx` requests 800 samples with confidence at least 0.7, one inference
  pass, and no early top-K limit. The additional GraspGenX outlier removal is off;
  scene registration still performs its configured outlier removal. The pinned
  upstream model/checkpoints and their diffusion settings remain in use.
- `grasp_quality` accepts a continuous 0–60° cone from top-down and smoothly
  favors vertical approaches. It requires fit before clipping to the jaw opening,
  capture coverage, opposing normal support, and spatial/orientation diversity.
  Collision meshes are checked against the support plane, scene points along
  insertion, and object/palm intersections. Full-stroke finger hulls are used
  conservatively against the scene, while intended finger/object contact is allowed.
- Up to 20 planning options undergo transit and complete insertion planning.
  For each ranked contact, try the generated orientation and its 180° rotation
  about TCP Z, first with 60 mm axial standoff, then with 40 mm if needed.
  The alternative rotation must pass the full contact/scene geometry filter;
  wrist, arm, and bench collisions remain checked by the motion planner.
  Both standoffs lead to the same contact position. The checked transit is
  retained and executed by plan ID, avoiding a second random IK solve that
  could select a different branch. Reuse requires fresh encoder feedback within
  0.03 rad per joint of the checked transit start. During selection, pregrasp and
  insertion states must retain 0.05 rad clearance from joint limits; the folded home
  waypoint is exempt from this grasp-selection preference.
  Pink IK uses up to ten attempts of 300 iterations, a 0.05 rad inward posture
  target near joint limits, and 2 mm / 1° convergence tolerances (also bounded
  by the endpoint limits). These are planning settings, not controller gains.
  Final approaches retain robot/self/bench collision checks and use absolute
  Cartesian targets at 40 mm/s with 0.15 m/s² acceleration. A 0.005 rad joint-path
  blend allowance avoids the severe corner slowdown seen with zero blending.
  The planner collision-checks the resulting trajectory; approaches exceeding
  8 seconds are rejected before execution. Planned endpoints must be within
  3 mm / 2°. Ordinary moves require three distinct fresh encoder observations
  within 5 mm / 3°. Tracking failure cancels motion.
  Cartesian waypoint zero is computed from the exact joint state passed to the
  planner. Using the requested pregrasp instead caused false start-pose errors
  even when IK was within 1 mm. Contact execution uses the same single snapshot
  to avoid a mismatch when new feedback arrives during planning setup.
- Final contact must settle within 5 mm / 3° and only proceeds if the attained pose passes
  the object-fit, capture, opposing-contact, approach-direction, palm, scene,
  and table-clearance gates again. This is checked at the measured pose, and
  the measured pose becomes the lift origin. Larger misses still fail; there
  is no closure based on distance alone. `CONTACT_NOT_SUPPORTED` means this
  recheck failed and the jaws were kept open. This does not correct the underlying
  encoder tracking error, whose joint positions, velocities and efforts are now
  included in tracking logs. Close-command/readback events distinguish a skipped
  close from a gripper actuation failure.
- Confirmed empty closes can retry after retreating to the absolute pregrasp and
  clearing the camera view. New frames and an unambiguous nearby object match are
  required. Failed poses are excluded relative to the latest object centroid.
  Object observations expire after 30 seconds, including planning/execution time.
  Repeated fresh feedback samples are skipped when counting settling/hold
  observations; stale feedback still fails. Home failures include joint errors.
- After closing, lift vertically 100 mm and require one second of sustained jaw
  obstruction without another close command. Uncertain holds prevent a new pick.
  Encoder FK and jaw obstruction are the reported evidence; neither is independent
  visual proof that the intended object was lifted.

`get_grasp_quality_report` exposes rejection counts and retained grasp metrics.
It also records each attempted orientation/standoff and its planning result.
The runtime project and lockfile are packaged under
`src/openyam_coordinator_agentic/grasping/grasp_gen_x/project/` at the repository
root. The profile loader selects `temp/graspgenx-venv` beside this workspace
profile as its environment on startup. The adjacent DimOS installation
is used as a read-only dependency. Restart the grasp blueprint to load changes.

The stricter contact gates are initial tuning values, not a measured optimum.
Partial depth views can legitimately fail the opposing-contact requirement.
The 0–60° cone constrains the contact/insertion direction, not the entire joint
transit from home. Collision checking uses observed scene geometry for insertion;
it does not reconstruct hidden surfaces or attach the held object's full geometry
to the carrying planner. Placement inherits absolute motion and convergence checks.

### Target offsets

No extra world-Z translation is added to a generated contact pose. The 120 mm
translation in `grasp_frame_to_tcp` converts the generator's gripper-body origin
to the URDF TCP, along the generated local approach axis. It matches
`gripper_tip_joint` and must stay paired with that tool definition. It is not a
120 mm world-height bias. The 60/40 mm pregrasp standoff and post-close 100 mm
lift are separate waypoints. There is no 200 mm grasp-target height adjustment.
The existing 20 **mm** collision-mesh extension is unrelated and is retained.

Earlier September 25 changes were made offline from source and September 24 logs.
No tests, blueprint restart, or hardware commands were run. Their effect on
physical success rate remains unmeasured; the residual encoder/depth errors
have not been compensated with an arbitrary target offset.
Physical gripper dimensions, depth bias, force tuning, and actual pick success
rates still require dedicated measurements/trials. No tests or hardware trials
were run for this implementation.

`wrist_camera_stream.py` serves the live wrist image at `http://localhost:8765`.
It uses the previously saved wrist calibration when available; `/raw` remains
available for comparison. Workspace calibration no longer fits or accesses this
camera. The grasp blueprint uses the fixed RGB-D camera for object perception.

## Run tuning (September 25, 18:00–18:10 local)

The run `20260925-175953-openyam-coordinator-agentic-openyam-grasp-graspgenx-agent`
reported 14 failed pick calls: 11 execution failures, two quality rejections, and
one planning failure. Five calls reached a close; all five were reported empty.
Their attained contact errors were 8.9–13.3 mm despite passing the geometry recheck.
This shows that the previous 15 mm acceptance band did not establish reliable
physical contact on this run. The profile now requires 5 mm, preserving the
attained-pose geometry recheck and empty-close detection.

The last three pregrasp failures requested joint 4 at -1.637, -1.530, and
-1.626 rad, but measured approximately -1.457 rad each time. TCP errors were
37.9, 18.2, and 39.2 mm. This is evidence of a repeatable travel restriction;
the logs do not establish whether it is a physical stop, interference, or a
controller restriction. `planning_joint_limits_rad` restricts joint 4 to
[-1.45, 1.5708] rad in the planning model, including IK and transit planning.
The existing 0.05 rad grasp margin further keeps selected grasp paths above
-1.40 rad. These overrides can only narrow the collision model's limits.

Shorter 60/40 mm standoffs reduce wrist travel and reach demands. Cartesian
speed/acceleration are reduced to 40 mm/s and 0.15 m/s² to reduce tracking lag;
settling gets five seconds. These are provisional tuning choices, not measured
cures for static tracking error. Two retry-home failures also showed 54 mrad
joint-3 and 41 mrad joint-4 residuals; additional time may not resolve these.
The existing controller gains, camera calibration, home pose, and collision
geometry were retained. Empty closes may also involve geometry or calibration
error that these logs cannot identify independently.

This update was based on log/source inspection. No tests, restart, or hardware
commands were run. Restart the grasp blueprint to load it; improved pick success
has not yet been demonstrated.

## Follow-up tuning (September 25, 18:22 run)

Run `20260925-182244-openyam-coordinator-agentic-openyam-grasp-graspgenx-agent`
reached all three executed pregrasps, then failed contact settling with TCP
errors of 23.4, 10.4, and 27.6 mm after the five-second wait. Planned contact
endpoints were within 0.01 mm of their targets. Joint 4 was well above its
restricted lower limit in these attempts. The earlier planning restriction
therefore does not address these contact failures.

The two larger misses retained shoulder errors of -0.032 and -0.035 rad,
joint-4 errors of +0.025 and +0.028 rad, and joint-5 errors of -0.047 and
-0.027 rad. These are measured execution errors; the logs cannot distinguish
load/model error from physical interference. Increasing the settling timeout
again is not supported by this run.

The bench profile now raises joint-2/3/4/5 position gains from
120/160/40/25 to 240/240/80/50, with joint-4/5 damping gains raised from
2/2 to 3/3. The first attempt also raised joint-2/3 damping to 7/6; the
motor codec rejected every command with `value out of range: field kd`, so
those two gains were restored to 5/5. Configuration validation now rejects
damping gains above 5 before hardware startup. Position-gain tuning remains
provisional and is not a demonstrated fix. Settling failures now record the first, best,
and final position errors and fresh sample count to distinguish persistent
offset from continuing convergence. Contact acceptance remains 5 mm with
the attained-pose geometry check.

The 18:34 run reached the selected contact pose within 3.0 mm, and the gripper
closed with an obstruction reading of 0.614. The operator reported a loud,
rapid, few-millimeter arm vibration beginning on the move from pregrasp to
contact, continuing through closure and lift, and unplugged the arm to stop it.
The arm gains above are retained because the operator reported that pose and
closure were good. The first CAN send-buffer error was at 16:35:39.139 UTC,
near the end of the 4.9-second lift; no such error appears during approach or
closure. The transport errors therefore occurred after the vibration began.
The contact trajectory ended at 16:35:32.666 UTC, and the next trajectory did
not start until 16:35:34.234 UTC. The operator reports vibration throughout,
including this fixed-target interval. This points to oscillation in the arm's
position-hold loop under the increased gains, rather than a rapidly changing
planned path as the sole cause.
Joint-2/3 position gains were raised from 120/160 to 240/240 while their
damping gains remained at 5/5, the motor codec's accepted maximum. This lowers
their relative damping and is the strongest configuration-based explanation
for a fast, small-amplitude position-hold oscillation. The log does not prove
which joint vibrated.
The current logs contain only occasional reached-pose snapshots, so they
cannot identify the vibrating joint or quantify its oscillation. The workspace
motion module now records Cartesian trajectory joint targets and 100 Hz joint
feedback during execution to distinguish command jitter from servo response
on a future run. It does not change the commands. DimOS was stopped after the
report.
Do not treat the successful contact pose as evidence that the vibration is
resolved.

## Tracking diagnosis baseline (September 25)

The higher arm gains from the 18:22 tuning are reverted to Git HEAD's
120/160/40/25 position gains on joints 2–5 and 5/5/2/2 damping gains. The
measured camera calibration and narrowed joint-4 planning limit remain. This
is a diagnostic baseline; it does not establish a grasp fix.

The grasp module now evaluates original 100 Hz position, velocity, effort,
and timestamps. TCP poses come from those same joint samples. Closure requires
0.4 seconds of continuous, fresh, stationary feedback at the target and a
second stationary interval immediately before commanding the jaws. Missing
velocity, stale samples, gaps over 50 ms, or rapid joint steps reject closure.

The physical adapter buffers the motor positions, velocity targets, gains,
feedforward torque, computed gravity torque, and motor feedback. A bounded
trace is emitted for rapid joint movement, persistent tracking error, rejected
motor commands, or failed settling. Compare outgoing position and gravity
torque against feedback to distinguish command jumps from motor response.

No hardware motion or tests were run for this change. The next requested
hardware session should use staged unloaded motion away from the table at
the configured approach speed and a slower speed, including the previously
failed contact postures. Require fresh stationary feedback without rapid wrist
motion before allowing another autonomous pick. A persistent unloaded offset
or vibration still needs measured controller or gravity-model work.

## Unloaded hardware diagnostic (September 25, 21:01–21:04 local)

Run `20260925-210126-openyam-coordinator-agentic-openyam-grasp-graspgenx-agent`
started with the restored baseline gains. At the unloaded starting pose, the
TCP was about 231 mm above world zero (bench top: -20 mm). Fifty-five 100 Hz
feedback samples showed no joint step and at most 0.0073 rad/s reported speed.

A 10 mm upward Cartesian move at the configured 40 mm/s limit completed. Its
commanded endpoint FK was z=241.50 mm; feedback at the end of the trace was
z=239.97 mm. The following stationary samples reached about z=241 mm, with a
maximum joint step of 0.00038 rad over 59 samples. No gripper close occurred.

The unloaded return requested 10 mm downward with `move_linear`, collision
checking, and `speed_scale=0.2`. Its planned duration was 0.574 s; this speed
scale is not an independently measured 20 mm/s Cartesian speed. The trajectory
reported complete, but commanded endpoint FK was z=232.03 mm while measured
endpoint FK remained z=241.20 mm. At the final sample, joint 3 lagged its
command by 0.0193 rad. During this return the largest consecutive outgoing
joint-target step was 0.00115 rad and the largest feedback step was 0.00191 rad.
Gravity feedforward on joint 3 remained near 8.2 Nm. A 0.0166 rad difference
between two trace rows spans 72 seconds of idle time; it is not a 10 ms
command jump.

This unloaded miss with smooth outgoing targets points to motor tracking or
gravity/model error, but does not separate those causes. The previously failed
contact postures were not tested after the unloaded return missed by about
9 mm. The run was stopped without another motion or grasp. The first shutdown
disconnected the adapter; a repeated coordinator stop logged a failed
deactivation after it was already disconnected. Autonomous picking has not met
the stationary unloaded approach criterion.

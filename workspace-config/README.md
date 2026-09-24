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
- `graspgenx` requests 400 samples with confidence at least 0.7, one inference
  pass, and no early top-K limit. The additional GraspGenX outlier removal is off;
  scene registration still performs its configured outlier removal. The pinned
  upstream model/checkpoints and their diffusion settings remain in use.
- `grasp_quality` accepts a continuous 0–60° cone from top-down and smoothly
  favors vertical approaches. It requires fit before clipping to the jaw opening,
  capture coverage, opposing normal support, and spatial/orientation diversity.
  Collision meshes are checked against the support plane, scene points along
  insertion, and object/palm intersections. Full-stroke finger hulls are used
  conservatively against the scene, while intended finger/object contact is allowed.
- Up to 20 ranked candidates undergo transit and complete insertion planning.
  Final approaches retain robot/self/bench collision checks and use absolute
  Cartesian targets at 80 mm/s with 0.3 m/s² acceleration. A 0.005 rad joint-path
  blend allowance avoids the severe corner slowdown seen with zero blending.
  The planner collision-checks the resulting trajectory; approaches exceeding
  8 seconds are rejected before execution. Planned endpoints must be within
  3 mm / 2°. Ordinary moves require three distinct fresh encoder observations
  within 5 mm / 3°. Tracking failure cancels motion.
  Cartesian waypoint zero is computed from the exact joint state passed to the
  planner. Using the requested pregrasp instead caused false start-pose errors
  even when IK was within 1 mm. Contact execution uses the same single snapshot
  to avoid a mismatch when new feedback arrives during planning setup.
- Final contact can settle within 15 mm / 3° only if the attained pose passes
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
- After closing, lift vertically 100 mm and require one second of sustained jaw
  obstruction without another close command. Uncertain holds prevent a new pick.
  Encoder FK and jaw obstruction are the reported evidence; neither is independent
  visual proof that the intended object was lifted.

`get_grasp_quality_report` exposes rejection counts and retained grasp metrics.
The runtime project and lockfile live in `../runtime/graspgenx`; its environment
is created under `temp/graspgenx-venv` on startup. The adjacent DimOS installation
is used as a read-only dependency. Restart the grasp blueprint to load changes.

The stricter contact gates are initial tuning values, not a measured optimum.
Partial depth views can legitimately fail the opposing-contact requirement.
The 0–60° cone constrains the contact/insertion direction, not the entire joint
transit from home. Collision checking uses observed scene geometry for insertion;
it does not reconstruct hidden surfaces or attach the held object's full geometry
to the carrying planner. Placement inherits absolute motion and convergence checks.
Physical gripper dimensions, depth bias, force tuning, and actual pick success
rates still require dedicated measurements/trials. No tests or hardware trials
were run for this implementation.

`wrist_camera_stream.py` serves the live wrist image at `http://localhost:8765`.
It uses the previously saved wrist calibration when available; `/raw` remains
available for comparison. Workspace calibration no longer fits or accesses this
camera. The grasp blueprint uses the fixed RGB-D camera for object perception.

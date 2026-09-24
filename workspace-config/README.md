# Workspace configuration

`openyam_bench.json` is this bench's measured profile. The grasp blueprint reads it
through `OPENYAM_WORKSPACE_CONFIG` at startup. `yam_collision.urdf` and its
`yam_collision.manifest.json` are the verified collision model;
`collision-meshes/` holds its finger-sweep meshes. Keep these files together.

`calibrate_workspace.py` finds the AprilTag in the fixed RGB-D camera stream and
uses its measured position relative to the arm axis. In arm coordinates +X is
forward, +Y is left, and +Z is up. The tag center is 70 mm right of the arm axis
(Y=-0.070 m), so the arm axis is 70 mm left of the tag (Y=0). The tag and arm
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
the approach clearance is a separate vertical lift before contact.
The collision model uses Git HEAD's manufacturer joint geometry. Earlier
joint-zero offsets proved to create a false collision at the folded arm pose;
their historical values remain in the calibration records, not the active model.
The `arm_control` gains improve tracking of commanded poses.
Grasp runs log `OpenYAM grasp perception`, `OpenYAM grasp target`, and
`OpenYAM grasp tracking` records in the DimOS run's `main.jsonl`. These include
the object cloud bounds, chosen TCP target, and encoder-derived reached pose
before and after the relative approach. `reached_minus_target_m` is in the
target frame (normally world). It exposes tracking error, but cannot measure
errors in joint zeros, the physical TCP, or camera-to-base calibration. A small
AprilTag fit residual alone does not establish absolute arm accuracy.
Object tracking retains stable IDs, but replaces each object's grasp point cloud
with its latest observation so retry scans cannot retain an object's old position.
The folded zero-joint `home_joints` match Git HEAD's power-off-safe resting pose, and
`side_grasp_score_scale` favors top approaches while retaining side grasps.

`wrist_camera_stream.py` serves the live wrist image at `http://localhost:8765`.
It uses the previously saved wrist calibration when available; `/raw` remains
available for comparison. Workspace calibration no longer fits or accesses this
camera. The grasp blueprint uses the fixed RGB-D camera for object perception.

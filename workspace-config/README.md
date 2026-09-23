# Workspace configuration

`openyam_bench.json` is this bench's measured profile. The grasp blueprint reads it
through `OPENYAM_WORKSPACE_CONFIG` at startup. `yam_collision.urdf` and its
`yam_collision.manifest.json` are the verified collision model;
`collision-meshes/` holds its finger-sweep meshes. Keep these files together.

`calibrate_workspace.py` calibrates the fixed RGB-D camera and the wrist camera
from the base-mounted AprilTag. With the blueprint and MCP server running, use:

```bash
../dimos/.venv/bin/python workspace-config/calibrate_workspace.py --source live
../dimos/.venv/bin/python workspace-config/calibrate_workspace.py --source live --apply
```

The wrist sequence first moves to the saved tag-visible pose, captures several
views to fit the 150-degree fisheye lens and camera-to-tool transform, then
returns the arm home. The first command writes a candidate under ignored `temp/`;
`--apply` writes a permanent record under ignored `calibration-records/` and
updates the JSON only after validation. Restart the blueprint to load it.
`--mode measure` checks the fixed camera pose without applying; `--replay PATH`
reuses an RGB-D capture. `temp/` is disposable, whereas the record named by
`workspace_calibration.record_path` documents the active pose.
`calibration-records/history/` retains earlier measurement reports.

The tag's black square is 56 mm, its center is 70 mm from the arm's base axis,
and the tag and arm-base datum are 20 mm above the bench. The saved tag rotation
comes from the mechanically aligned mounting plate; routine camera calibration
assumes the tag has not moved relative to the arm. The bench footprint, camera
wall and placement point are separately measured and are **not** rediscovered
by this script. The 12 mm grasp-height offset is separate from calibration.
The collision model uses Git HEAD's manufacturer joint geometry. Earlier
joint-zero offsets proved to create a false collision at the folded arm pose;
their historical values remain in the calibration records, not the active model.
The `arm_control` gains improve tracking of commanded poses.
The folded zero-joint `home_joints` match Git HEAD's power-off-safe resting pose, and
`side_grasp_score_scale` favors top approaches while retaining side grasps.

`wrist_camera_stream.py` serves the live wrist image at `http://localhost:8765`.
After calibration it displays the fisheye-corrected stream; `/raw` remains
available for comparison. The grasp blueprint currently uses the fixed RGB-D
camera for object perception.

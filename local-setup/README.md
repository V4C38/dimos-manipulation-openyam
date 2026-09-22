# Local Setup

This directory is intentionally outside the installable Python package. It holds
machine- and bench-specific calibration, collision geometry, and evidence.

`openyam_bench.json` is the current personal workbench configuration. The measured
apple runner supplies it through `OPENYAM_BENCH_CONFIG` to the local agentic grasp
stack. Configuration is read at startup; editing it does not update a running
coordinator.

## Dense cloud and grasp-quality gate

The local bench stack captures aligned RGB-D at 1280×720 @ 6 FPS and generates
object clouds with 2 mm voxels, a one-pixel mask edge trim, and statistical
outlier removal. `perception` in `openyam_bench.json` records those settings.

Before any pick approach, `grasp_quality` applies the same class-independent
checks to every generated proposal: minimum cloud coverage, support within the
open gripper sweep, plausible jaw-axis span and centering. It ranks surviving proposals
using the generator score and those geometric measures. If none pass, the robot
makes no approach or close command, rescans once, then returns
`INSUFFICIENT_GRASP_QUALITY` with diagnostics. The MCP RPC
`get_grasp_quality_report` exposes the latest raw count, rejection reasons and
retained metrics. These checks apply to any segmented rigid object, not only an
apple label.

For this bench, the AprilTag center is 7 cm from the arm axis at
`[0.0, -0.07, 0.0]` metres in the arm-base world frame. That displacement is
already encoded in the current camera transform; applying it to the resulting
translation a second time double-counts it. A future calibration must also use
the tag's measured orientation rather than assuming that its printed axes are
perfectly aligned with the arm axes.

The historical restored camera pose mapped the captured tabletop to the configured collision
surface within 0.7 mm beside the tag and 2.5 mm at the apple. This table-plane
check predates the September 22 translation correction; it is not a current
whole-workspace calibration result.

## September 22 tag-center correction

The operator confirmed that the tag center is 70 mm from the base-joint rotation
axis and that its surface shares the arm-base datum, 13 mm above the tabletop.
The existing `[0, -0.07, 0]` tag coordinate is therefore retained. The detected
black square measures 56 mm; the 70 mm paper square is not the PnP tag size.

Using 36 synchronized live RGB-D frames, the old transform placed the tag at
`[-6.420, -70.953, -0.919]` mm. Camera translation was corrected by
`[+6.420, +0.953, +0.919]` mm, retaining the camera quaternion. A separate
36-frame capture evaluated with the saved correction placed it at
`[-0.019, -70.015, +0.041]` mm. The independent depth estimate was
`[+0.899, -69.516, -1.775]` mm. These are tag-anchor residuals, not proof of
submillimetre arm or workspace accuracy.

The printed tag initially had an apparent world yaw of approximately -2.86
degrees. The operator subsequently confirmed that the black mounting plate is
mechanically aligned with robot X/Y, allowing small physical mounting mismatch.
This independently establishes a yaw reference without assuming perfect tag
alignment or matching the plate to the manufacturer's base outline.

On 36 fresh frames, the exposed bottom and right plate edges had axis errors of
-2.505 and -2.448 degrees. Their mean yielded a +2.476654-degree world-Z camera
rotation, with translation recomputed to keep the tag center at `[0, -0.07, 0]`.
Roll and pitch were retained. A further 36-frame capture evaluated with that
saved transform gave bottom/right errors of -0.029/+0.029 degrees, tag yaw
-0.375 degrees, and tag-center error of approximately 0.12 mm by PnP or 1.7 mm
by depth. These are same-reference repeatability measurements, not independent
proof of the arm's absolute positioning accuracy.

The shorter visible top plate edge disagrees by approximately 1.03 degrees
after correction; it was not used in the yaw fit. Edge localization, physical
plate shape and remaining tilt errors are not distinguished by this check.
The residual tag yaw is consistent with a small mounting mismatch, but does
not prove one. Multi-position fingertip/workspace validation is still outstanding.

Evidence is in `evidence/tag-offset-before-20260922/` and
`evidence/tag-offset-after-20260922/` (Git-ignored). Each contains the evaluated
configuration, intrinsics, per-frame measurements, a scene image and depth data.
Yaw evidence is in `evidence/yaw-before-20260922/` and
`evidence/yaw-after-20260922/`, including per-frame plate-edge fits.
The running coordinator was not restarted and no arm movement was commanded.
The saved correction takes effect on the next normal stack launch.

## Reusable calibration

`calibrate_workspace.py` replaces the former calibration, live-tag measurement,
and camera-capture scripts in `tools/`. The bench pickup runner is now
`local-setup/test_apple_pick_and_place.py`. There is one calibration entry point;
no wrappers or old fixed-pixel calibration scripts are retained.

From the repository root, evaluate the saved calibration against the running
Zenoh camera stream:

```bash
../dimos/.venv/bin/python local-setup/calibrate_workspace.py --source live --mode measure
```

Solve a fresh full six-degree-of-freedom camera pose, saving a candidate without
changing the active configuration; add `--apply` to install it:

```bash
../dimos/.venv/bin/python local-setup/calibrate_workspace.py --source live
../dimos/.venv/bin/python local-setup/calibrate_workspace.py --source live --apply
```

The script uses the detected tag corners anywhere in the image, current camera
intrinsics and factory color/depth extrinsics. It does not reuse pixel ROIs or
accumulate old offsets. Frames are split into a fit set and held-out validation
set. Reports include old/new tag residuals, depth cross-checks and reprojection
errors. These measure local consistency, not physical TCP accuracy across the
workspace. Unstable fits are not applied. The script never commands the arm,
stops a coordinator, or changes the adjacent DimOS installation.

The current tag orientation in `openyam_bench.json` is an **empirical reference**
from the September 22 corrected frame, including its unverified roll/pitch.
Reusing it preserves that reference, including its uncertainty. It is not an
independent measurement of tag tilt. To establish/refine orientation from the
mechanically aligned plate, select a long, unobstructed edge on a fresh image:

```bash
../dimos/.venv/bin/python local-setup/calibrate_workspace.py \
  --source live --select-plate-axis --plate-axis x --apply
```

Click start then end in the robot's **positive X direction**, then press Enter
(Escape cancels). For a positive Y edge, use `--plate-axis y`. This is a desktop
OpenCV window. The line is fitted to nearby image edges; `plate-reference.png`
and the report retain the reference frame and fitted endpoints. Headless use can
provide `--plate-axis-pixels U1 V1 U2 V2` on the selected capture frame, preferably
through replay. Endpoints must span at least 40 pixels. The selected edge must
be coplanar with the tag; their plane is assumed parallel to robot XY. Tag tilt
or an incorrectly directed axis invalidates this reference. Once established,
the tag-to-base orientation is stored, so routine camera/base recalibration
needs no new clicks while the tag mounting stays rigid.

For an independently measured tag orientation use `--tag-rpy-deg R P Y` (degrees,
extrinsic XYZ). `--tag-in-world X Y Z` supplies its measured base-relative center
in metres; `--tag-size` is the black-square edge, currently 0.056 m; `--tag-id`
defaults to 0. If the tag is moved relative to the arm, remeasure those inputs.
Moving the arm without moving the reference tag with it cannot be inferred from
the tag image alone.

### Camera or base movement

- Camera-only movement: rerun calibration. The full camera pose is solved again;
  static world geometry is retained. Moving the camera stand can also invalidate
  the manually measured camera-wall obstacle, which needs separate remeasurement.
- Arm base movement with the tag rigidly attached: use `--base-moved`. Camera
  pose is recalculated in the new base frame. Without additional measurements,
  the candidate sets `bench_center_m`, `camera_wall`, and `place_tcp_m` to null,
  so the existing runtime checks reject stale scene coordinates.
- If the rigid mapping of the old base frame into the new one is measured,
  supply `--base-moved --scene-transform path/to/matrix.json`. The JSON is a 4x4
  homogeneous matrix with `p_new = T @ p_old` (metres). It relocates bench and
  camera-wall centers/orientations and the release TCP, preserving box sizes.
  This assumes those physical scene objects did not move independently. Camera
  motion alone cannot establish this mapping when base motion is also possible.

### Direct camera, capture, and replay

When the camera is free, `--source direct` (the default) opens only the camera.
It does not stop a process that already owns the camera. The default profile is
640x480 at 6 FPS; `--width 1280 --height 720 --fps 6` requests the advertised
higher-resolution profile. It does not change the robot blueprint's profile.

```bash
../dimos/.venv/bin/python local-setup/calibrate_workspace.py --mode capture
../dimos/.venv/bin/python local-setup/calibrate_workspace.py \
  --replay local-setup/evidence/calibration-EXISTING --apply
```

Live capture requires the adjacent runtime and one connected camera matching the
configured serial. Direct/replay mode can use an isolated environment without
installing anything into DimOS:

```bash
uv run --no-project --with numpy --with scipy --with opencv-contrib-python \
  --with pyrealsense2 python local-setup/calibrate_workspace.py --help
```

Each run creates a new timestamped evidence directory (or `--evidence PATH`):
RGB-D frames in metres, camera metadata/extrinsics, image, original configuration,
and, for calibration, candidate configuration and report. Replay accepts this
new format, not the old metadata-incomplete `capture.npz` files. `--apply` keeps
the original in that evidence directory and atomically replaces the JSON.
Concurrent configuration edits are not overwritten. Applying does not reload
the running stack; its next normal launch loads the new calibration. Historical
September 22 correction records remain provenance; `workspace_calibration`
identifies subsequent script-generated updates.

The collision boxes are incomplete site geometry. They do not model people,
loose objects, the full bench perimeter, or unmeasured fixtures. Review and
measure them before any physical run.

Place capture directories below `local-setup/evidence/`; they are ignored by
Git because they may contain large RGB-D recordings.

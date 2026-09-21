# Local Setup

This directory is intentionally outside the installable Python package. It holds
machine- and bench-specific calibration, collision geometry, and evidence.

`openyam_bench.json` is the current personal workbench configuration. It is used
only by the optional `bench-planner-coordinator` entry point. The generic
`coordinator-agentic` blueprint does not load it.

For this bench, the AprilTag center is 7 cm from the arm axis at
`[0.0, -0.07, 0.0]` metres in the arm-base world frame. That displacement is
already encoded in the current camera transform; applying it to the resulting
translation a second time double-counts it. A future calibration must also use
the tag's measured orientation rather than assuming that its printed axes are
perfectly aligned with the arm axes.

The restored camera pose maps the captured tabletop to the configured collision
surface within 0.7 mm beside the tag and 2.5 mm at the apple. This table-plane
check is the local acceptance reference for future calibration changes.

The collision boxes are incomplete site geometry. They do not model people,
loose objects, the full bench perimeter, or unmeasured fixtures. Review and
measure them before any physical run.

Place capture directories below `local-setup/evidence/`; they are ignored by
Git because they may contain large RGB-D recordings.

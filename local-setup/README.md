# Local Setup

This directory is intentionally outside the installable Python package. It holds
machine- and bench-specific calibration, collision geometry, and evidence.

`openyam_bench.json` is the current personal workbench configuration. It is used
only by the optional `bench-planner-coordinator` entry point. The generic
`coordinator-agentic` blueprint does not load it.

The collision boxes are incomplete site geometry. They do not model people,
loose objects, the full bench perimeter, or unmeasured fixtures. Review and
measure them before any physical run.

Place capture directories below `local-setup/evidence/`; they are ignored by
Git because they may contain large RGB-D recordings.

# OpenYAM agentic manipulation

External DimOS package for RGB-D, GraspGenX pick and place with checked motion
and feedback. Personal calibration and collision assets stay in `workspace-config/`.

## Blueprints

| Name | Function |
| --- | --- |
| `openyam-coordinator-agentic.openyam-planner-coordinator-agent` | OpenYAM planner and MCP agent |
| `openyam-coordinator-agentic.openyam-grasp-graspgenx-agent` | Camera, GraspGenX pick/place, MCP agent and cockpit |

Start the grasp blueprint with `$run-grasp-blueprint` or run
`bash .agents/skills/run-grasp-blueprint/scripts/run.sh`. It stops registered
DimOS runs, sets up `can0` if needed, and loads `workspace-config/openyam_bench.json`.
The planner blueprint can be started with:

```bash
../dimos/.venv/bin/dimos --can-port can0 run openyam-coordinator-agentic.openyam-planner-coordinator-agent --daemon
```

## Configuration and development

`OPENYAM_WORKSPACE_CONFIG` selects the grasp profile. The configured 120 mm
gripper-body-to-TCP transform is part of the frame definition, not a contact
height offset. DimOS in `../dimos` is a read-only host dependency; make changes
in this repository. Grasp policies are separated for review and possible future
upstream contribution; physical reliability still requires hardware evidence.

Run `../dimos/.venv/bin/ruff check src` and
`../dimos/.venv/bin/ruff format --check src` for style checks. Start hardware
or run tests only when requested.

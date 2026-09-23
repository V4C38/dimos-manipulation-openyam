# OpenYAM Coordinator Agentic

An external DimOS distribution with OpenYAM planner and grasping agents. It does
not modify the DimOS checkout.

The installed distribution is named `openyam-coordinator-agentic`. DimOS
namespaces external blueprint entry points by distribution name:

```bash
dimos run openyam-coordinator-agentic.openyam-planner-coordinator-agent
```

This is deliberately not a built-in unqualified blueprint name. External
blueprints are discovered from the `dimos.blueprints` entry-point group and must
remain namespaced to avoid collisions with DimOS or other installed projects.

## Blueprints

- `openyam-coordinator-agentic.openyam-planner-coordinator-agent`: OpenYAM planner,
  control coordinator, manipulation skills, MCP server, and MCP client agent.
  It provides motion and gripper tools, without object scanning or pick/place.
- `openyam-coordinator-agentic.openyam-grasp-graspgenx-agent`: fixed-camera
  RGB-D registration, GraspGenX, quality gating, pick/place, planner, MCP agent,
  and a cockpit with camera video and agent chat. It requires an explicit
  `OPENYAM_WORKSPACE_CONFIG` profile.

The two names follow the shipped xArm planner-agent and grasp-agent split. The
grasp agent scans for object descriptions supplied in the interactive request.

## Install With uv

Use the DimOS environment. Do not use `pip`.

```bash
cd /home/johannes/dimensional-applications/dimos
uv pip install -e ../dimos-openyam-coordinator-agentic
uv run dimos list
```

For a fully isolated development environment, `uv sync` in the DimOS checkout
first creates or updates its `.venv`; then run the same `uv pip install -e`
command. The package has no independent `dimos` PyPI requirement because it is
designed to run against the adjacent checkout. Its `tool.uv.sources` records
that editable local DimOS source relationship for project tooling.

Verify package metadata without loading hardware:

```bash
cd /home/johannes/dimensional-applications/dimos
uv run python -c "from dimos.robot.external_blueprints import list_external_blueprint_names; print(*list_external_blueprint_names(), sep='\n')"
```

## Run

Prepare CAN according to the DimOS OpenYAM documentation, clear the workspace,
and keep the emergency stop reachable. Then run one blueprint at a time:

```bash
cd /home/johannes/dimensional-applications/dimos
uv run dimos --can-port can0 run openyam-coordinator-agentic.openyam-planner-coordinator-agent --daemon
```

The MCP server exposes DimOS manipulation skills. Agent motion is still real
robot motion; inspect state, confirm the workspace, and use the normal DimOS
status and stop commands.

To use the calibrated workbench for grasping, set its profile explicitly:

```bash
cd /home/johannes/dimensional-applications/dimos
export OPENYAM_WORKSPACE_CONFIG=/home/johannes/dimensional-applications/dimos-openyam-coordinator-agentic/workspace-config/openyam_bench.json
uv run dimos --can-port can0 run openyam-coordinator-agentic.openyam-grasp-graspgenx-agent --daemon
```

Open `http://127.0.0.1:7780/` on this machine for the cockpit camera and Chat
panel. You can also send your own prompt from another terminal with
`../dimos/.venv/bin/dimos agent-send "..."` or run `../dimos/.venv/bin/humancli`.
The grasp blueprint starts a local cockpit relay; `--relay-url` selects an
existing relay instead. A browser on another machine requires a hosted relay.

`OPENYAM_WORKSPACE_CONFIG` is the explicit path to this stack's profile. DimOS
`--config` changes module fields after blueprint composition and cannot add the
camera, grasp generator, or site-specific model. The workspace profile gives
the fixed-camera transform, capture and quality settings, obstacles, gripper
geometry, home pose, and a relative path to the collision model. The measured
placement coordinate is retained as site data, but is not injected into the
agent's instructions or used by the current pick flow.

## Workspace Configuration

`workspace-config/` isolates personal camera calibration and collision boxes from
generic package source. Only the grasp blueprint reads
`OPENYAM_WORKSPACE_CONFIG`; the planner agent never reads it.

The single calibration/camera-capture entry point is
`workspace-config/calibrate_workspace.py`:

```bash
../dimos/.venv/bin/python workspace-config/calibrate_workspace.py --source live
# Inspect the generated report/candidate, then apply a fresh calibration:
../dimos/.venv/bin/python workspace-config/calibrate_workspace.py --source live --apply
```

Use `--mode capture` for temporary frames only or `--mode measure` to evaluate the saved
calibration. See [workspace configuration](workspace-config/README.md) for the
measured assumptions. Unapplied runs go to ignored `workspace-config/temp/`;
applied calibration records go to ignored `workspace-config/calibration-records/`.

## Safety

The current shipped OpenYAM URDF contains visual meshes but **no collision
geometry**. `auto_convert_meshes` changes file formats only. The grasp planner
rejects that unmodified model during construction, before module startup.
The measured workspace profile points to the manufacturer-CAD collision model.
The planner-only agent uses upstream's canonical model and has none of the
workbench collision boxes.

The grasp planner corrects one RoboPlan modeling issue: the fixed `bench` and
`camera_tripod_wall` boxes overlap, and RoboPlan otherwise treats their mutual
overlap as a collision at every robot configuration. Only that exact static/static
pair is filtered. This filtering does not alter either box; robot self and
robot/environment checks remain enabled. A complete validated robot model and
site preflight are still required. Replacing either static obstacle later requires
reapplying the static-pair correction; until then the overlap fails closed.

The user separately authorized the camera wall to be **0.30 m wide in world Y**,
centered at camera Y=-0.2540255 m. The current local JSON therefore spans
Y=[-0.4040255, -0.1040255]. Wall depth/height/center and all bench geometry are
unchanged. The earlier 2 m wall is historical, not the current configuration.

The optional local collision boxes are incomplete. They do not model the full
bench, people, loose objects, unmeasured fixtures, or unverified calibration
error. They are not authorization for autonomous motion. Review physical setup,
hardware state, and collision geometry before each run.

## Upstream PR scope

The proposed DimOS PR would add the two arm blueprints under
`dimos/robot/manipulators/openyam/blueprints/agentic.py`, with registry entries
matching the xArm planner and GraspGenX agent names. Reusable grasping modules
would live in DimOS package code if accepted. This repository's
`workspace-config/openyam_bench.json`, collision meshes, and calibration records
are site data and stay outside that PR. The generic grasp
blueprint needs a documented workspace-profile schema or equivalent module
configuration; the current external implementation loads the profile through
`OPENYAM_WORKSPACE_CONFIG` before composing its modules. No upstream files have
been changed here.

# OpenYAM Coordinator Agentic

An external DimOS distribution for the generic OpenYAM planner coordinator with
MCP agent tools. It does not modify the DimOS checkout.

The installed distribution is named `openyam-coordinator-agentic`. DimOS
namespaces external blueprint entry points by distribution name, so the generic
agentic blueprint is run as:

```bash
dimos run openyam-coordinator-agentic.coordinator-agentic
```

This is deliberately not a built-in unqualified blueprint name. External
blueprints are discovered from the `dimos.blueprints` entry-point group and must
remain namespaced to avoid collisions with DimOS or other installed projects.

## Blueprints

- `openyam-coordinator-agentic.coordinator-agentic`: generic OpenYAM planner,
  control coordinator, manipulation skills, MCP server, and MCP client agent.
  It contains no camera calibration, scene geometry, or personal collision data.
- `openyam-coordinator-agentic.bench-planner-coordinator`: optional planner with
  the collision boxes from `local-setup/openyam_bench.json`. This is personal
  configuration, not generic OpenYAM behavior.

The generic agentic blueprint follows the shipped xArm coordinator-agentic
pattern: planner/coordinator plus `ManipulationSkills`, `McpServer`, and
`McpClient`. It uses standard OpenYAM hardware and model helpers provided by
DimOS. There is intentionally no custom automatic pick/place episode workflow.

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
uv run dimos --can-port can0 run openyam-coordinator-agentic.coordinator-agentic --daemon
```

The MCP server exposes DimOS manipulation skills. Agent motion is still real
robot motion; inspect state, confirm the workspace, and use the normal DimOS
status and stop commands.

To use the personal workbench collision boxes, opt in explicitly:

```bash
cd /home/johannes/dimensional-applications/dimos
export OPENYAM_LOCAL_SETUP=/home/johannes/dimensional-applications/dimos-openyam-coordinator-agentic/local-setup/openyam_bench.json
export OPENYAM_COLLISION_MODEL=/home/johannes/dimensional-applications/dimos-openyam-coordinator-agentic/local-setup/collision-model/yam_collision.urdf
uv run dimos --can-port can0 run openyam-coordinator-agentic.bench-planner-coordinator --daemon
```

## Local Setup

`local-setup/` isolates personal camera calibration and collision boxes from
generic package source. The optional bench blueprint reads
`OPENYAM_LOCAL_SETUP`; the generic blueprint never reads it.

Camera-only utility scripts are under `tools/`:

```bash
uv run --no-project --with numpy --with pyrealsense2 \
  python tools/capture_bench_evidence.py --serial <SERIAL> \
  --output local-setup/evidence/capture-YYYYMMDD
```

The calibration utility also requires `opencv-contrib-python`; see its module
docstring for its required physical AprilTag inputs. Captured evidence is ignored
by Git under `local-setup/evidence/`.

## Safety

The current shipped OpenYAM URDF contains visual meshes but **no collision
geometry**. `auto_convert_meshes` changes file formats only. The optional bench
planner rejects that unmodified model during construction, before module startup.
An external manufacturer-CAD model is now available through
`OPENYAM_COLLISION_MODEL`; it passes the model coverage gate. Do
not use the generic agentic blueprint as a workaround: it uses the same missing
collision model and does not contain the bench obstacles.

The bench planner corrects one RoboPlan modeling issue: the fixed `bench` and
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

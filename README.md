# OpenYAM agentic manipulation

An external DimOS package for OpenYAM motion and RGB-D pick/place. It adds
GraspGenX sampling, contact/clearance filtering, checked motion, feedback-based
verification, and bounded retries. Measured workspace data stays outside the
reusable modules.

## Blueprints

| Distribution-qualified name | Capabilities |
| --- | --- |
| `openyam-coordinator-agentic.openyam-planner-coordinator-agent` | Upstream OpenYAM planner/coordinator, manipulation tools, MCP agent |
| `openyam-coordinator-agentic.openyam-grasp-graspgenx-agent` | Fixed RGB-D camera, object registration, GraspGenX, checked pick/place, MCP agent, cockpit |

The grasp blueprint requires `OPENYAM_WORKSPACE_CONFIG`. The planner blueprint
does not load a workspace profile. The `dimos.blueprints` entry-point names stay
namespaced to avoid collisions with built-in DimOS blueprints.

## Package structure

```text
src/openyam_coordinator_agentic/
  blueprints/agentic.py          planner-agent composition
  blueprints/grasp.py            grasp stack and agent composition
  agent_prompts.py               agent instructions
  config.py                     typed workspace schema and explicit loader
  pick_and_place_module.py       OpenYamPickAndPlaceModule and execution policy
  checked_motion.py             motion RPC contract, planner and endpoint checks
  grasp_quality.py               contact, direction, clearance and diversity gates
  gripper_geometry.py            gripper collision geometry in TCP coordinates
  dense_scene_registration.py    fresh RGB-D observations and native masks
  workspace_module.py            configured obstacles and fixed-camera TF
  workspace_geometry.py          serializable workspace boxes
  collision_model.py             collision asset generation/loading
  collision_safety.py            coverage and static-obstacle collision handling
  grasping/grasp_gen_x/
    module.py                   host facade and sampling configuration
    runtime.py                  isolated inference implementation
    project/                    packaged pyproject.toml and uv.lock
workspace-config/               personal calibration, model and bench profile
audits/                         historical observations and implementation reviews
```

Execution modules use typed `ModuleConfig` subclasses and DimOS RPC/spec wiring.
They do not read environment variables or a global bench dictionary. Blueprint
composition loads the profile and passes each module its inputs. `grasp.py` and
the `blueprints` package export the previous entry-point targets for existing
editable installations; there is only one implementation of each blueprint.

## Dependencies and installation

DimOS is a separately provisioned host dependency. This source integration uses
the compatible adjacent checkout and its `native/python/graspgenx` runtime as
read-only dependencies. It does not install or update DimOS automatically.
Direct scientific dependencies are declared in this package's `pyproject.toml`.
The optional GraspGenX/Torch environment has its own packaged lockfile, pinned
source revision, and upstream checkpoint revision; inference imports occur only
inside that environment. The current recipe requires Linux x86-64 and Python 3.12.

This workspace already has an editable installation in the adjacent runtime.
No reinstall is needed for source changes. For a new deployment, provision a
compatible DimOS source environment that you own, then register this extension:

```bash
uv pip install --python /path/to/your/dimos-venv/bin/python -e .
```

Do not run installation or dependency-update commands against this workspace's
read-only `../dimos` environment. The isolated inference environment defaults to
`temp/graspgenx-venv` beside the workspace profile; `runtime_environment` can
select another profile-relative location.

## Run

From this repository, with the hardware available and a configured CAN interface:

```bash
export OPENYAM_WORKSPACE_CONFIG="$PWD/workspace-config/openyam_bench.json"
../dimos/.venv/bin/dimos --can-port can0 run openyam-coordinator-agentic.openyam-grasp-graspgenx-agent --daemon
```

Run only one hardware blueprint at a time. The cockpit is at
`http://127.0.0.1:7780/`; it includes camera video and agent chat. Use the normal
DimOS status/stop commands to manage the process. The planner-only command is:

```bash
../dimos/.venv/bin/dimos --can-port can0 run openyam-coordinator-agentic.openyam-planner-coordinator-agent --daemon
```

For a pick, the agent calls `scan_objects` with object descriptions, then
`pick_object` with a returned object ID. The pick tool owns retries and reports
terminal failures. Placement requires a successful pick and explicit world-frame
TCP coordinates; the agent does not infer release coordinates from an image.

## Workspace configuration

`config.py` defines the runtime schema. Camera and gripper transforms, collision
model, home joints, bench/wall geometry, and perception/planning/execution policy
are explicit inputs. Relative paths resolve against the profile directory.
Top-level calibration records are retained as metadata; runtime policy sections
reject unknown fields. `dimos --config` configures modules after composition and
does not replace the workspace profile.

See [workspace settings](workspace-config/README.md) for measured values,
frame conventions, grasp thresholds, and calibration commands. A generated
contact pose has no additional world-height bias: its 120 mm local transform
converts the configured gripper-body origin to the URDF TCP. Standoff and lift
are separate waypoints.

## Development and validation

Keep changes in this repository. Start hardware and run tests only when requested.
Formatting/import ordering follows DimOS's 100-column Ruff style:

```bash
../dimos/.venv/bin/ruff check src
../dimos/.venv/bin/ruff format --check src
```

Offline parsing, blueprint composition, config validation and package inspection
do not establish physical grasp reliability. See the
[offline grasp audit](audits/2026-09-25-offline-grasp-followup.md) and
[structure review](audits/2026-09-25-structure-review.md) for work actually done.
Residual tracking errors and empty closes remain measurement issues.

The grasp stack requires the configured collision model and retains robot,
bench, observation-age, endpoint and hold checks. Contact filtering uses a single
observed point cloud; hidden surfaces and a held object's full collision geometry
are not reconstructed. Encoder FK and jaw obstruction are the verification
signals, not independent visual confirmation of acquisition. Placement inherits
the explicit-target workflow with checked motion and release verification.

## Possible future DimOS contribution

No upstream PR is being created. Blueprint composition mirrors DimOS's OpenYAM
and xArm conventions. Reusable perception, sampling, grasp-quality and motion
policies are separated from the workspace so they can be reviewed independently.
The remaining upstream extension points and validation gaps are recorded in the
[structure review](audits/2026-09-25-structure-review.md). Personal calibration,
bench geometry and generated collision assets are excluded from that proposed
scope. Physical success-rate evidence and upstream integration tests are still
needed before claiming high reliability across object classes.

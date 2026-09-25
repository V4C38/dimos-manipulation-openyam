# OpenYAM module structure review — September 25, 2026

Scope: the external `openyam-grasp-graspgenx-agent` implementation and its
packaging, configuration, blueprint composition, and upstream references.
This was an offline source review. No hardware was started, no inference was
run, no unit or integration test suite was run, and no PR was created.

## Changes

- Split blueprint composition into `blueprints/agentic.py` and
  `blueprints/grasp.py`. Execution lives in `pick_and_place_module.py`; prompts,
  workspace loading, obstacle configuration, and TF publication have separate
  modules. Compatibility exports retain the previous editable-install entry
  points without duplicating their implementation.
- Replace the global workspace dictionary with `OpenYamWorkspaceConfig` and
  explicit module configurations. Perception, sampling, quality, execution,
  and motion policies have shared typed schemas used by both the profile and
  their consuming modules. Policy sections reject unknown fields. Historical
  top-level calibration metadata is still accepted.
- Keep measured poses, geometry, gains, and paths in `workspace-config/`.
  Resolve relative paths beside the selected profile. Importing execution or
  configuration modules no longer loads the bench profile.
- Use serializable box configurations at the blueprint boundary. Construct
  upstream world-frame obstacles when initializing the planning world. This
  avoids an upstream obstacle dataclass's unresolved forward annotation during
  nested configuration validation.
- Put the GraspGenX facade, isolated implementation, and runtime project under
  `grasping/grasp_gen_x/`, matching the upstream organization. Ship the runtime
  manifest and lockfile as package data. Resolve Python imports from the
  installed package and the read-only DimOS source root, without assuming the
  extension's checkout depth. Keep the inference environment beside the profile.
- Remove copied runtime development settings that referenced absent source and
  test directories. Declare the extension's direct scientific dependencies and
  document DimOS as a separately provisioned source-runtime dependency.
- Apply DimOS-style module/config naming, type annotations, import ordering,
  and 100-column Ruff formatting. Keep distribution-qualified blueprint names.
- Limit checked-plan cleanup to the matching plan ID, so a rejected request
  does not clear another request's replacement plan.

The previous motion, grasp filtering, feedback, retry, and collision guards
remain. This pass does not retune the bench profile or add a contact-height
offset. The local 120 mm gripper-body-to-TCP transform remains necessary;
pregrasp and lift are distinct waypoints, not offsets to the contact target.

## Offline validation performed

- Imported reusable modules without a workspace environment variable and
  parsed the package's Python source.
- Composed the grasp-agent blueprint, pickled/unpickled it, and validated all
  **11 module configurations** using their declared config types, without
  constructing or starting modules.
- Resolved all **five module references** with the host's reference resolver:
  motion to coordinator; pick/place to scene, generator, and checked motion;
  manipulation skills to checked motion. No missing or ambiguous provider was
  found. This does not exercise live stream transport or RPC execution.
- Confirmed the legacy grasp entry-point export resolves to the same blueprint.
- Ran `ruff check src` and `ruff format --check src`: both passed (20 files).
- Resolved the packaged runtime lockfile offline with `uv lock --offline`.
  Compared every non-root dependency against the original lockfile: all
  **146 dependency versions and sources** are unchanged.
- Built a wheel using `uv build --wheel --no-build-isolation` and inspected its
  entry points and packaged runtime files. No package was installed. Build
  output stays in ignored `workspace-config/temp/package-review/`.

These checks establish composition and packaging consistency, not physical
grasp success, inference latency, solver success rate, or controller tracking.

## Remaining upstream coupling

The host reviewed here is DimOS commit
`f2945fad060082dae0023f4f00cade270b097c13`. It remains a source dependency:
the isolated adapter imports `native/python/graspgenx/graspgenx_runtime` from
that checkout. A standalone wheel containing this extension does not provision
DimOS, its native dependencies, or the workspace assets.

Several overrides deliberately use upstream protected extension points:

| Local module | Upstream coupling |
| --- | --- |
| `checked_motion.py` | Planning epochs, selected-path generation, retained-plan state, endpoint FK, and planner/world interfaces |
| `pick_and_place_module.py` | Pick/place helpers, gripper readback, manipulation state, and inherited placement workflow |
| `dense_scene_registration.py` | Detector setup, image/3D processing, latest scene snapshot, and object database updates |
| `grasping/grasp_gen_x/runtime.py` | Runtime slot and sampler access while retaining upstream pose conversion/validation |
| `workspace_module.py`, `collision_safety.py` | Planning initialization and the narrowly scoped static bench/wall collision-pair handling |

For an eventual DimOS contribution, these policies should become supported
configuration/hooks within the corresponding upstream modules. That can reduce
the override surface; this external package cannot promise compatibility with
arbitrary future DimOS revisions. Personal calibration and collision assets
should stay outside the reusable contribution.

Physical evidence is still needed for the previously recorded tracking error
and empty closes, and for success rates across object shapes. A single observed
cloud does not reconstruct hidden surfaces or complete held-object geometry.
The current acquisition checks use encoders and jaw obstruction; they are not
independent visual confirmation. See the
[offline grasp follow-up](2026-09-25-offline-grasp-followup.md).

## Repository boundary

All edits and build artifacts are in this repository. No installation or
dependency changes were made to `../dimos`. Its two preexisting modified files
(`dimos/hardware/sensors/camera/realsense/blueprints.py` and `scripts/install.sh`)
were preserved. Its binary diff hash before and after the work is unchanged:
`e417959914de96d203b981b84c5df9d734c03fe99871012afeb18d16814726d6`.

# Repository Guidelines

## Repository Boundary & Agent Instructions

All changes belong in `/home/johannes/dimensional-applications/dimos-openyam-coordinator-agentic`. Do **not** change the base DimOS checkout or installation in `../dimos`, including its source, dependencies, and environment. Treat it as a read-only runtime dependency; implement extensions, configuration, calibration, collision models, and tests here.

Perform explicitly requested actions directly. Do not add or run unsolicited safety checks, preflight routines, or tests before acting unless the user explicitly requests them. Avoid excessive testing and repeated confirmation requests. This instruction does not authorize removing existing runtime safeguards.

## Project Structure & Module Organization

- `src/openyam_coordinator_agentic/`: generic agentic blueprint, optional bench planner, collision model generation, and geometry helpers.
- `tools/`: camera capture, calibration, diagnostics, model audits, and `test_collision_safety.py`.
- `local-setup/`: personal bench configuration, collision assets, calibration, and evidence. Keep these separate from generic package behavior; capture directories under `local-setup/evidence/` are Git-ignored.
- `pyproject.toml`: setuptools packaging and `dimos.blueprints` entry points. Preserve distribution-qualified blueprint names.

## Setup Reference

Consult [OpenYAM_dimOS_Setup_Guide.pdf](OpenYAM_dimOS_Setup_Guide.pdf) for applicable setup, camera, calibration, and agent integration details. Use `README.md` for repository commands and `local-setup/CONTINUATION_30CM.md` for recorded local state. Treat documented historical outcomes as history, not current hardware observations. Adapt upstream installation instructions to respect the repository boundary above.

## Development & Test Commands

Run from this repository using the existing adjacent runtime:

```bash
../dimos/.venv/bin/dimos list
../dimos/.venv/bin/dimos --can-port can0 run openyam-coordinator-agentic.coordinator-agentic --daemon
../dimos/.venv/bin/python -m pytest tools/test_collision_safety.py -q
```

These list blueprints, start the hardware-backed agentic blueprint, and run offline collision regressions, respectively. Start hardware only when requested. Run tests when requested, selecting relevant cases with `-k`; no coverage threshold is configured. Use `uv` rather than plain `pip` for dependency tooling, without modifying the base installation.

## Coding Style & Naming

Target Python 3.10–3.12. Follow existing four-space indentation, `snake_case` functions/modules, `PascalCase` classes, and uppercase constants. Use descriptive names, type annotations, and concise docstrings. No formatter or linter is configured here. Name pytest files `test_*.py` and functions `test_*`.

## Commits & Pull Requests

There is no commit history yet. Use concise, imperative commit subjects. PRs should explain the change, affected configuration, and validation actually performed; explicitly state when tests were not run. Link relevant issues or evidence when available.

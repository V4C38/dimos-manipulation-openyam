# Repository Guidelines

## Repository Boundary & Agent Instructions

All changes belong in `/home/johannes/dimensional-applications/dimos-openyam-coordinator-agentic`. Do **not** change the base DimOS checkout or installation in `../dimos`, including its source, dependencies, and environment. Treat it as a read-only runtime dependency; implement extensions, configuration, calibration, collision models, and tests here.

This package composes OpenYAM, a fixed RGB-D camera, GraspGenX, checked motion, and agentic pick/place. Measured bench settings live in `workspace-config/`; reusable policies live in `src/`. The upstream review separated those policies because reliable grasping needs explicit perception, filtering, planning, and feedback checks, while some DimOS integration still uses protected hooks. Keep this boundary clean for a possible future DimOS PR; do not make that PR unless requested.

Perform explicitly requested actions directly. Do not add or run unsolicited safety checks, preflight routines, or tests before acting unless the user explicitly requests them. Avoid excessive testing and repeated confirmation requests. This instruction does not authorize removing existing runtime safeguards.

## Project Structure & Module Organization

- `src/openyam_coordinator_agentic/`: planner and grasp blueprints, execution modules, collision model generation, and geometry helpers.
- `workspace-config/`: personal bench configuration, collision assets, and calibration. Keep these separate from generic package behavior; `temp/` and `calibration-records/` are Git-ignored.
- `pyproject.toml`: setuptools packaging and `dimos.blueprints` entry points. Preserve distribution-qualified blueprint names.

## Setup Reference

Use `README.md` for repository commands and `workspace-config/README.md` for measured local assumptions. Treat documented historical outcomes as history, not current hardware observations.

## Development & Test Commands

Run from this repository using the existing adjacent runtime:

```bash
../dimos/.venv/bin/dimos list
../dimos/.venv/bin/dimos --can-port can0 run openyam-coordinator-agentic.openyam-planner-coordinator-agent --daemon
```

These list blueprints and start the hardware-backed planner blueprint, respectively. Use `.agents/skills/run-grasp-blueprint/` for a requested grasp-blueprint start. Start hardware only when requested. Run tests only when requested. Use `uv` rather than plain `pip` for dependency tooling, without modifying the base installation.

## Coding Style & Naming

Target Python 3.10–3.12. Follow existing four-space indentation, `snake_case` functions/modules, `PascalCase` classes, and uppercase constants. Use descriptive names, type annotations, and concise docstrings. Ruff is configured in `pyproject.toml` with DimOS's 100-column formatting and import ordering. Name pytest files `test_*.py` and functions `test_*`.

## Commits & Pull Requests

Use concise, imperative commit subjects. PRs should explain the change, affected configuration, and validation actually performed; explicitly state when tests were not run. Link relevant issues or evidence when available.

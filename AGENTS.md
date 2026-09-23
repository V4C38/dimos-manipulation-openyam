# Repository Guidelines

## Repository Boundary & Agent Instructions

All changes belong in `/home/johannes/dimensional-applications/dimos-openyam-coordinator-agentic`. Do **not** change the base DimOS checkout or installation in `../dimos`, including its source, dependencies, and environment. Treat it as a read-only runtime dependency; implement extensions, configuration, calibration, collision models, and tests here.

Perform explicitly requested actions directly. Do not add or run unsolicited safety checks, preflight routines, or tests before acting unless the user explicitly requests them. Avoid excessive testing and repeated confirmation requests. This instruction does not authorize removing existing runtime safeguards.

## Project Structure & Module Organization

- `src/openyam_coordinator_agentic/`: generic agentic blueprint, optional bench planner, collision model generation, and geometry helpers.
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

These list blueprints and start the hardware-backed agentic blueprint, respectively. Start hardware only when requested. Run tests only when requested. Use `uv` rather than plain `pip` for dependency tooling, without modifying the base installation.

## Coding Style & Naming

Target Python 3.10–3.12. Follow existing four-space indentation, `snake_case` functions/modules, `PascalCase` classes, and uppercase constants. Use descriptive names, type annotations, and concise docstrings. No formatter or linter is configured here. Name pytest files `test_*.py` and functions `test_*`.

## Commits & Pull Requests

There is no commit history yet. Use concise, imperative commit subjects. PRs should explain the change, affected configuration, and validation actually performed; explicitly state when tests were not run. Link relevant issues or evidence when available.

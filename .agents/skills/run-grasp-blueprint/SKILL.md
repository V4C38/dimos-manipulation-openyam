---
name: run-grasp-blueprint
description: Start this repository's OpenYAM GraspGenX agent when the user asks to run the grasp blueprint, configuring can0 if needed.
---

# Run the OpenYAM grasp blueprint

Run `bash .agents/skills/run-grasp-blueprint/scripts/run.sh` from this repository.
The script stops all registered DimOS runs, checks `can0`, configures it at
1 Mbit/s if needed, then starts the grasp blueprint with the bench profile.
CAN setup uses `uv run --no-sync` in `../dimos` to preserve its installation.

Use only when the user requests a hardware-backed start. Report setup or
startup errors; do not retry or change workspace calibration.

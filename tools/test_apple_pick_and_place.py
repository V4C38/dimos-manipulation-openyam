#!/usr/bin/env python3
"""Start one agentic, measured OpenYAM apple pick-and-place test.

The runner stops the existing coordinator, starts only the local grasp stack,
then delivers one fixed prompt to its MCP agent. The agent runs the prompted
YOLOE/GraspGenX pick-and-place workflow and returns home only after a
successful release at the measured ``place_tcp_m``.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import time
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT.parent / "dimos" / ".venv" / "bin" / "dimos"
BLUEPRINT = "openyam-coordinator-agentic.coordinator-agentic"
INITIAL_PROMPT = "Pick up the apple, then place it 10 cm further away on the table, then return to home."
ENV_FILE = ROOT / ".openyam.env"
AGENT_READY_TIMEOUT_S = 180
AGENT_READY_POLL_S = 2


def load_local_environment() -> dict[str, str]:
    """Load this workstation's ignored key file without exporting it globally."""
    if not ENV_FILE.is_file():
        return {}
    values: dict[str, str] = {}
    for line in ENV_FILE.read_text(encoding="utf-8").splitlines():
        if line and not line.startswith("#"):
            name, value = line.split("=", maxsplit=1)
            values[name] = value
    return values


def load_config(path: Path) -> dict[str, Any]:
    """Reject missing physical measurements before affecting runtime state."""
    config = json.loads(path.read_text(encoding="utf-8"))
    required = ("gripper", "grasp_frame_to_tcp", "empty_epsilon", "place_tcp_m")
    absent = [name for name in required if config.get(name) is None]
    gripper = config.get("gripper") or {}
    absent.extend(
        f"gripper.{name}" for name in (
            "extents_open", "offset_open", "extents_half_open", "offset_half_open", "fingertip_depth"
        ) if gripper.get(name) is None
    )
    if absent:
        raise RuntimeError("Measured configuration is incomplete: " + ", ".join(absent))
    return config


def start_fresh_stack(environment: dict[str, str]) -> None:
    """Stop DimOS's active stack, then start this test's one coordinator."""
    subprocess.run([str(RUNTIME), "stop"], cwd=ROOT, env=environment, check=False)
    subprocess.Popen(
        [str(RUNTIME), "--can-port", "can0", "run", BLUEPRINT, "--daemon"],
        cwd=ROOT, env=environment, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def send_initial_prompt(environment: dict[str, str]) -> None:
    """Deliver the prompt only once the MCP agent has registered its tools."""
    deadline = time.monotonic() + AGENT_READY_TIMEOUT_S
    last_response = ""
    while time.monotonic() < deadline:
        sent = subprocess.run(
            [str(RUNTIME), "agent-send", INITIAL_PROMPT],
            cwd=ROOT, env=environment, check=False, text=True, capture_output=True,
        )
        if sent.returncode == 0 and "Message sent to agent:" in sent.stdout:
            print(sent.stdout, end="", flush=True)
            return
        last_response = (sent.stdout + sent.stderr).strip()
        time.sleep(AGENT_READY_POLL_S)
    raise RuntimeError(
        f"MCP agent did not become ready within {AGENT_READY_TIMEOUT_S}s: {last_response}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=ROOT / "local-setup" / "openyam_bench.json",
        help="Measured local configuration (default: local-setup/openyam_bench.json)",
    )
    parser.add_argument(
        "--api-key-env", default="OPENAI_API_KEY",
        help="Environment variable containing the OpenAI API key (default: OPENAI_API_KEY)",
    )
    parser.add_argument(
        "--model", default=os.environ.get("OPENYAM_LLM_MODEL"),
        help="Optional OpenAI model override passed as OPENYAM_LLM_MODEL",
    )
    args = parser.parse_args()
    local_environment = load_local_environment()
    if not (os.environ.get(args.api_key_env) or local_environment.get(args.api_key_env)):
        raise RuntimeError(f"Required API key environment variable {args.api_key_env} is not set")
    config_path = args.config.resolve()
    config = load_config(config_path)
    environment = os.environ | local_environment | {
        "OPENYAM_BENCH_CONFIG": str(config_path),
        "OPENYAM_COLLISION_MODEL": str(ROOT / "local-setup" / "collision-model" / "yam_collision.urdf"),
    }
    if args.model:
        environment["OPENYAM_LLM_MODEL"] = args.model
    start_fresh_stack(environment)
    send_initial_prompt(environment)


if __name__ == "__main__":
    main()

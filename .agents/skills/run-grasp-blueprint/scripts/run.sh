#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
dimos_dir="$repo_dir/../dimos"
dimos_bin="$dimos_dir/.venv/bin/dimos"

status_output="$("$dimos_bin" status)"
while [[ "$status_output" != *"No running DimOS instance"* ]]; do
    if [[ "$status_output" != *"Run ID:"* ]]; then
        printf 'Could not identify the running DimOS process:\n%s\n' "$status_output" >&2
        exit 1
    fi
    printf '%s\n' "$status_output"
    "$dimos_bin" stop
    status_output="$("$dimos_bin" status)"
done

can_details="$(ip -details link show dev can0 2>/dev/null || true)"

if ! grep -Eq '<([^,>]*,)*UP(,|>)' <<<"$can_details" || \
   ! grep -Eq 'bitrate[[:space:]]+1000000([[:space:]]|$)' <<<"$can_details"; then
    (cd "$dimos_dir" && uv run --no-sync dimos hardware can setup can0)
fi

cd "$repo_dir"
export OPENYAM_WORKSPACE_CONFIG="$repo_dir/workspace-config/openyam_bench.json"
export PYTHONPATH="$repo_dir/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$dimos_dir/.venv/bin/dimos" --can-port can0 run \
    openyam-coordinator-agentic.openyam-grasp-graspgenx-agent --daemon

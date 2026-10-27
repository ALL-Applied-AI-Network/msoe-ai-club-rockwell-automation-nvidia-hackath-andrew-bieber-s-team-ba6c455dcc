#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
physical_uv_dir="$HOME/.local/physical-uv"
physical_uv_command=$(command -v uv || true)
physical_uv_resolved=""
if [[ -n "$physical_uv_command" ]]; then
    physical_uv_resolved=$(readlink -f -- "$physical_uv_command")
fi
if [[ -x "$physical_uv_dir/bin/uv" ]]; then
    export PATH="$physical_uv_dir/bin:$PATH"
elif [[ -z "$physical_uv_command" || "$physical_uv_command" == /snap/* ||
        "$physical_uv_command" == /var/lib/snapd/* || "$physical_uv_resolved" == /snap/* ||
        "$physical_uv_resolved" == /usr/bin/snap ]]; then
    echo "Installing a private, non-Snap uv for the physical client."
    python3 -m venv "$physical_uv_dir"
    "$physical_uv_dir/bin/python" -m pip install uv
    export PATH="$physical_uv_dir/bin:$PATH"
fi
hash -r
uv --version
uv sync --locked
SORT_GRIPPER_PROFILE=legacy uv run python -m unittest -v test_sorting.py test_physical.py test_workflow.py test_batch.py test_batch_watch.py test_gripper_profile.py test_dual_clearance.py test_dual_policy.py test_dual_pixels.py
uv run preflight.py
uv run camera_check.py --output camera_mapping.png
uv run run.py --mock --trials 1 --trial-s 3
printf '\nSetup passed. For later commands in this/new ROSIE shell, run:\n'
printf 'export PATH="$HOME/.local/physical-uv/bin:$PATH"\n'

#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if ! command -v uv >/dev/null 2>&1; then
    python3 -m pip install --user uv
    export PATH="$HOME/.local/bin:$PATH"
fi
uv sync --locked
uv run python -m unittest -v test_sorting.py test_physical.py test_workflow.py
uv run preflight.py
uv run camera_check.py --output camera_mapping.png
uv run run.py --mock --trials 1 --trial-s 3

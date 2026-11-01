#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
read -r -p "Contact email for the robot-test request: " physical_test_contact
uv run prepare_submission.py --contact "Andrew Bieber, $physical_test_contact"
uv run preflight.py --submission
cat submission/placement.yaml
printf '\nReview submission/submission.yaml and submission/placement.yaml above.\n'
printf 'Nothing has been submitted. When ready: uv run run.py --submit\n'

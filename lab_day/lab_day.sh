#!/usr/bin/env bash
# Lab-day launcher for the physical cell. Run it on ROSIE (GPU session) against the
# SAME physical-client folder that holds your .request.json. Either copy this file
# into that folder, or keep it in lab_day/ and set PHYSICAL_CLIENT_DIR=<that folder>.
#
#   bash lab_day.sh check  [legacy|dual_zone_v2]   # checks only, no robot
#   bash lab_day.sh run    [legacy|dual_zone_v2]   # connect and wait for the operator
#   bash lab_day.sh image  [legacy|dual_zone_v2] table.png   # SAM on a saved frame
#
# This script never changes the motion gates in sort_config.py. Motion stays off
# (preview only) until you edit them by hand with the operator.
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${PHYSICAL_CLIENT_DIR:-}" ]]; then client="$PHYSICAL_CLIENT_DIR"
elif [[ -f "$here/run.py" ]]; then client="$here"              # copied into the client folder
else client="$here/../physical-client"; fi                      # repo layout: lab_day/ next to it
[[ -f "$client/run.py" ]] || { echo "No physical-client found at $client. Set PHYSICAL_CLIENT_DIR."; exit 2; }
cd -- "$client"
echo "Using physical client: $(pwd)"

mode=${1:-check}
profile=${2:-legacy}
case "$profile" in legacy|dual_zone_v2) ;; *) echo "Profile must be legacy or dual_zone_v2"; exit 2;; esac

export PATH="$HOME/.local/physical-uv/bin:$PATH"
export SORT_GRIPPER_PROFILE="$profile"
export SORT_SAM_PYTHON="${SORT_SAM_PYTHON:-$HOME/venvs/sam2-check/bin/python}"
mkdir -p runs/lab_day
log="runs/lab_day/${mode}_${profile}_$(date +%Y%m%d_%H%M%S).log"

gates() {
    uv run --locked python - <<'EOF'
import sort_config as S
print(f"  gripper profile : {S.PHYSICAL_GRIPPER_PROFILE}")
print(f"  allowed objects : {S.PHYSICAL_ALLOWED_KINDS}")
print(f"  MOTION          : {'ON' if S.PHYSICAL_EXECUTE and S.PHYSICAL_POSES_CONFIRMED else 'OFF (preview only)'}"
      f"  [PHYSICAL_EXECUTE={S.PHYSICAL_EXECUTE}, PHYSICAL_POSES_CONFIRMED={S.PHYSICAL_POSES_CONFIRMED}]")
if S.PHYSICAL_GRIPPER_PROFILE != "legacy":
    print(f"  printed fingers : mount={S.DUAL_ZONE_MOUNT_CONFIRMED} objects={S.DUAL_ZONE_OBJECTS_CONFIRMED}"
          f" layout={S.DUAL_ZONE_LAYOUT_CONFIRMED} tip offset (m)={S.DUAL_ZONE_PRIMARY_OFFSET_TCP_M}")
EOF
}

{
echo "== lab_day.sh $mode  profile=$profile  $(date)"
[[ -f .request.json ]] || echo "WARNING: no .request.json here. Run from the folder you submitted from, or the client cannot get its token."
gates
case "$mode" in
  check)
    uv run --locked preflight.py
    uv run --locked preflight.py --sam --require-cuda
    uv run --locked run.py --mock --trials 1 --trial-s 3
    uv run --locked run.py --status
    ;;
  image)
    [[ -n "${3:-}" ]] || { echo "Usage: bash lab_day.sh image <profile> table.png"; exit 2; }
    uv run --locked camera_check.py --image "$3" --output runs/lab_day/real_camera_mapping.png
    uv run --locked policy.py --image "$3"
    ;;
  run)
    uv run --locked run.py --status
    echo "Leave this running. The operator starts each rollout. Ctrl+C only when the operator says the run is over."
    uv run --locked run.py
    echo "Run folders from this session:"; ls -dt runs/physical_* 2>/dev/null | head -5
    ;;
  *) echo "Mode must be check, image or run"; exit 2;;
esac
} 2>&1 | tee "$log"
echo "Log saved to $log"

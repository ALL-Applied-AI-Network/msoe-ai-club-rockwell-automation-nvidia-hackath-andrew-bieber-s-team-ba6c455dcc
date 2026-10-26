# Physical test handoff — October 22, 2026

> **Archived morning record.** The status and instructions below describe the
> October 22 morning preparation, before the physical test and later updates.
> Use [BATCH_HANDOFF.md](BATCH_HANDOFF.md) and [README.md](README.md) for the
> current version, configuration and deployment sequence.

The package is prepared for an operator-reviewed first physical test. The prior
upload/extraction succeeded, but ROSIE setup stopped at Snap uv's permission
error. This updated archive fixes setup's uv selection. Installation on ROSIE,
submission and booking are still unconfirmed. This review did not connect to
ROSIE or the cell. Both motion gates remain false. Use this updated archive;
the earlier archive or branch copy does not contain today's recovery fix.

## 1. Upload — Windows PowerShell on your computer

Save the supplied `physical-client-ready.tar.gz` in Downloads, then:

```powershell
scp -o "User=biebera@ad.msoe.edu" "$env:USERPROFILE\Downloads\physical-client-ready.tar.gz" "dh-mgmt2.hpc.msoe.edu:physical-client-ready.tar.gz"
ssh -l "biebera@ad.msoe.edu" dh-mgmt2.hpc.msoe.edu
```

## 2. Recover/install — ROSIE bash

Use a fresh directory, separate from the simulation repo and the interrupted install:

```bash
mkdir -p ~/innovation-lab
physical_test_dir=$(mktemp -d "$HOME/innovation-lab/physical-test-XXXXXXXX")
tar -xzf ~/physical-client-ready.tar.gz -C "$physical_test_dir"
cd "$physical_test_dir/physical-client"
export PATH="$HOME/.local/physical-uv/bin:$PATH"
bash setup_rosie.sh && pwd
```

Continue only if setup passes, and save the printed path. Setup prefers an
existing private uv, otherwise uses a non-Snap uv or installs one in
`~/.local/physical-uv` using a Python venv. It does not change the cluster's Snap
installation. In every new ROSIE shell, export the PATH shown above.
If setup is interrupted, return to that folder and rerun
`bash setup_rosie.sh`; no need to recreate it. The setup only checks the local
client and local mock. It never submits or connects to the physical robot.
Keep any old `.request.json` private; if a prior request already exists, avoid
submitting a duplicate and reuse its request file in the intended client folder.

In an allocated ROSIE GPU session, return to the saved folder:

```bash
export SORT_SAM_PYTHON="$HOME/venvs/sam2-check/bin/python"
export PATH="$HOME/.local/physical-uv/bin:$PATH"
uv run preflight.py --sam --require-cuda
```

The last check verifies imports and requires CUDA, not weights/inference. Before
the slot, test SAM on an actual uncropped table frame if one is available:
`uv run policy.py --image table.png`. Do not run heavy SAM inference on a login node.

## 3. Generate, review, submit and book — ROSIE bash / dashboard

```bash
bash request_test.sh
cat submission/submission.yaml submission/placement.yaml
```

Enter your actual email when prompted. Confirm your task/team suffix and the
proposed hardware/layout before the next command. The request proposes a screw,
then red/blue/yellow 10 mm cubes individually, then one mixed trial, using marked
bare-table destinations with no cups. The operator must confirm availability and
accept or revise those destinations. The helper has NOT submitted anything.

When you choose to submit:

```bash
uv run run.py --submit
uv run run.py --status
```

Take only the public request ID to https://hackathon.sidclab.org/#calendar.
Select an available **30-minute slot on Thursday, October 22, in Central time**;
wait for organizer approval. 2:30–3:00 p.m. was an earlier candidate, not a verified
current opening or a reservation. Do not paste `.request.json`, its secret or
your private results link into chat. Keep the client folder for the approved slot.

The official sequence was rechecked October 22:
https://hackathon.sidclab.org/ and the organizer client at
https://github.com/SIDCLab/PhysicalAI-Hackathon-2026 (commit `ff7534e`).

## 4. Approved slot — preview before motion

In the allocated GPU session, export the SAM interpreter as above and run:

```bash
uv run run.py --status
uv run run.py
```

Leave the client connected while the operator starts the preview rollout.
With the supplied gates false, it detects once, saves the full frame,
`camera_mapping.png`, `targets.png` and `targets.json` under the printed
`runs/physical_*/cycle_000/` directory, and holds the reported pose/gripper.

With the operator, verify raw camera orientation, measured table landmarks,
the fingertip/tool frame, jaw yaw, clearances, pickup height and every scan/drop
pose. Do not flip the image to look like the simulation. Use four spread-out
real landmarks with `camera_check.py` as described in README.md; projection
arithmetic alone does not prove real calibration. Proposed release is 10 mm
above the bare table and is unverified physically.

Only after these checks, deliberately set both `PHYSICAL_EXECUTE` and
`PHYSICAL_POSES_CONFIRMED` true in `sort_config.py`, and restart **in coordination
with the operator**. Start with one object, one supervised pickup and release;
then progress to the approved trials. Other objects remain disabled. A screw
head wider than 11 mm may cause conservative rejection: inspect the overlay,
do not automatically increase the jaw limit. The system does not do collision planning.

Log object, starting position, attempts, pickup, correct placement, elapsed time,
failure reason and interventions. Source disappearance and a release command
are logged as unverified, not physical success. Record first-attempt success
separately from eventual recovery. See VALIDATION.md for exactly what was tested.

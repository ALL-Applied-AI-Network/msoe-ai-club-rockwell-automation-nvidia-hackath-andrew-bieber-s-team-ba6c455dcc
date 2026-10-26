"""Local checks for the physical client. No robot connection and no SAM inference."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess

import numpy as np
import yaml

from calibration.geometry import load, project, pixel_to_base, UF0_Z_MM, SURFACE_Z
from policy import Policy, base_to_tcp
import sort_config as S


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sam",action="store_true",help="Also check the external SAM interpreter/imports/GPU")
    parser.add_argument("--require-cuda",action="store_true",help="With --sam, fail unless that interpreter can use CUDA")
    parser.add_argument("--submission",action="store_true",help="Check the prepared request and placement files")
    args=parser.parse_args()
    if args.require_cuda and not args.sam:
        parser.error("--require-cuda requires --sam")
    cam=load()["table"]
    points=np.array([[x,y,SURFACE_Z] for x in (.22,.35,.49) for y in (-.27,0,.27)])
    pixels=project(cam,points)
    restored=np.array([pixel_to_base(cam,uv,z=SURFACE_Z) for uv in pixels])
    np.testing.assert_allclose(restored,points,atol=1e-8)
    assert S.SURFACE_Z==SURFACE_Z and S.UF0_Z_MM==UF0_Z_MM, "Physical frame constants differ"
    assert math.isclose(base_to_tcp(0,0,SURFACE_Z)[2],-335.1), "UF0 conversion differs"
    for xy in [*S.DROP_XY.values(),S.SCAN_XY]:
        Policy.validate(base_to_tcp(*xy,S.SURFACE_Z+S.TRAVEL_HEIGHT))
    Policy.validate(base_to_tcp(*S.SCAN_XY,S.SURFACE_Z+S.SCAN_HEIGHT))
    if not 0.001 <= S.DROP_HEIGHT < S.TRAVEL_HEIGHT:
        raise SystemExit("Release height must be above the table and below travel height")
    Policy.batch_mode()  # validate configured scan mode without connecting
    if S.PHYSICAL_BATCH_MAX_IDLE_PASSES < 1:
        raise SystemExit("PHYSICAL_BATCH_MAX_IDLE_PASSES must be positive")
    print("PASS: projection arithmetic, resize/frame conventions, UF0 conversion and configured pose bounds")
    print("These mathematical checks do not establish real-world calibration, grasp or collision accuracy.")
    print(f"Camera orientation: raw physical table frame; no image flip. Native size: {cam['size']}")
    print(f"Release: {S.DROP_HEIGHT*1000:.1f} mm fingertip above bare table (proposed; verify in cell)")
    print(f"Motion enabled: {S.PHYSICAL_EXECUTE}; poses confirmed: {S.PHYSICAL_POSES_CONFIRMED}")
    print(f"Allowed kinds: {S.PHYSICAL_ALLOWED_KINDS}; width upper-bound bypass for verified-fit kinds: {S.PHYSICAL_VERIFIED_FIT_KINDS}")
    print(f"Width upper bound for other kinds: {S.PHYSICAL_MAX_GRASP_WIDTH_MM} mm; finite positive widths still required")
    print(f"Scan mode: {S.PHYSICAL_SCAN_MODE}; destination correction: {S.PHYSICAL_INSPECT_DROP_AREAS}; watch: {S.PHYSICAL_WATCH}")
    if Policy.batch_mode():
        print("One SAM pass per batch; fresh RGB change checks between picks; final rescan even at attempt limit")
    print("SHA256",hashlib.sha256((S.ROOT/'calibration/cameras.json').read_bytes()).hexdigest(),"calibration/cameras.json")
    if args.sam:
        python=Path(S.SAM_PYTHON).expanduser()
        if not python.is_file():
            raise SystemExit(f"Missing SAM interpreter: {python}. Set SORT_SAM_PYTHON to your existing environment.")
        check="import torch,sam2,cv2,numpy,PIL; print('SAM imports OK; CUDA:',torch.cuda.is_available())"
        if args.require_cuda:
            check += "; import sys; sys.exit(0 if torch.cuda.is_available() else 'CUDA unavailable: use an allocated GPU session')"
        subprocess.run([str(python),"-c",check],check=True,timeout=45)
        print("SAM import check does not load weights or run inference.")
    if args.submission:
        sub=yaml.safe_load((S.ROOT/'submission/submission.yaml').read_text())
        place=yaml.safe_load((S.ROOT/'submission/placement.yaml').read_text())
        if "@" not in sub.get('contact','') or '<' in sub.get('team',''):
            raise SystemExit("Run prepare_submission.py with your actual contact first")
        if len(place.get('rollouts',[]))!=sub['rollouts']:
            raise SystemExit("Placement count differs from rollout count")
        print("PASS: request identity and placement/rollout count")


if __name__=="__main__":
    main()

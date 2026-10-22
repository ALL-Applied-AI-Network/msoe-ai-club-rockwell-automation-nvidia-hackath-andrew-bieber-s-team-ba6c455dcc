"""Write a reviewable first-test submission locally. This command does not submit it."""
from __future__ import annotations

import argparse
from pathlib import Path

import yaml
import sort_config as S


def first_trials():
    return [
        {"objects": {"screw": [390,0,0]},
         "prompt": "Place one M6x30 screw with its HEAD at (390,0) mm, yaw 0 toward the tip. Sort it to screws."},
        {"objects": {"cube": [390,-90,20]},
         "prompt": "Use one RED 10 mm cube at (390,-90) mm, yaw 20 degrees. Sort it to red."},
        {"objects": {"cube": [390,90,-20]},
         "prompt": "Use one BLUE 10 mm cube at (390,90) mm, yaw -20 degrees. Sort it to blue."},
        {"objects": {"cube": [350,130,0]},
         "prompt": "Use one YELLOW 10 mm cube at (350,130) mm. Sort it to yellow."},
        {"objects": {"screw": [390,-160,0],
                     "cube": [[340,-50,20],[420,40,-20],[340,150,0]]},
         "prompt": "Mixed test: screw head at (390,-160), yaw 0; RED 10 mm cube at (340,-50), BLUE at (420,40), YELLOW at (340,150). Sort to the corresponding bare-table zones."},
    ]


def write_submission(contact, team="Andrew Bieber's Team", task_id="mvp2-andrew", directory=None):
    if "@" not in contact or any(s in contact.lower() for s in ("your_email","example.com","<contact","todo")):
        raise ValueError("Supply a real contact email (optionally with your name)")
    if not team.strip() or not task_id.strip():
        raise ValueError("Team and task ID must be nonempty")
    directory=Path(directory or S.ROOT / "submission")
    directory.mkdir(parents=True,exist_ok=True)
    zones="; ".join(f"{label}=({S.DROP_XY[label][0]*1000:.0f},{S.DROP_XY[label][1]*1000:.0f}) mm"
                    for label in ("screws","red","blue","yellow"))
    trials=first_trials()
    sub={"team":team,"contact":contact,"policy":"MVP2-SAM2-Physical-v1", "model_type":"Hybrid",
         "task_id":task_id,
         "task":"Sort M6x30 screws and red/blue/yellow 10 mm cubes into separate bare-table zones. "
                "First four rollouts present one object each, followed by one mixed rollout. "
                "No cups or bins. Target centres in robot base XY: "+zones+". "
                "Initial preview holds position; motion is enabled only after the real overlay and poses are checked with the operator. "
                "Success: every presented object released in its designated zone; log attempts, time and interventions. "
                "The operator must confirm that the listed objects and proposed marked destinations are available and acceptable.",
         "objects":"different","layout":"different","rollouts":len(trials),"custom_hardware":"none"}
    (directory/"submission.yaml").write_text(yaml.safe_dump(sub,sort_keys=False,allow_unicode=True))
    (directory/"placement.yaml").write_text(yaml.safe_dump({"rollouts":trials},sort_keys=False,allow_unicode=True))
    return sub,trials


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contact",required=True)
    parser.add_argument("--team",default="Andrew Bieber's Team")
    parser.add_argument("--task-id",default="mvp2-andrew",help="Use your organizer-assigned team suffix if required")
    args=parser.parse_args()
    try:
        sub,_=write_submission(args.contact,args.team,args.task_id)
    except ValueError as e:
        parser.error(str(e))
    print(yaml.safe_dump(sub,sort_keys=False))
    print("Wrote both submission YAML files. Review them, then: uv run run.py --submit")


if __name__=="__main__":
    main()

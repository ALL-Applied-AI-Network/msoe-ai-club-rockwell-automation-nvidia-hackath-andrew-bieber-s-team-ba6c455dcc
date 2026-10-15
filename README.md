# Vision-Guided Grasping and Sorting

Give the supplied FANUC arm eyes so it can find supported objects, pick them up,
and put them where they belong when their starting positions change.

We develop in the supplied digital twin and aim to validate on the real arm
through the approved, supervised testing process.

## Current Status — October 10, 2026

- The starter is in `fanuc_er4ia_sim/` in this repository.
- Isaac Sim runs in a Rosie browser desktop in Andrew's account.
- A scene has been exported and images captured from the three simulated cameras.
- Manual joint movement works in the interactive scene.
- All 33 starter host tests pass.
- A separate one-cube diagnostic, `demo_pick.py`, passes numerical IK checks.
  Its actual simulated pickup and placement have **not yet been verified**.
- Cosmos/model setup, the complete vision-driven pipeline, teammate setup,
  and the physical-arm interface remain unverified.

These results establish a working development environment. They do not yet
establish successful autonomous picking or real-arm performance.

## V1: A Complete First Version

1. Verify one simulated pick-and-place at a known cube position.
2. Locate that supported cube from camera observations and repeat the pick.
3. Sort two colour groups of the same supported cube shape into defined destinations.
4. Check actual pickup and placement, recording every attempt and failure.
5. Compare against a defined baseline on held-out trials.
6. Validate the approved task on the real arm during supervised testing.

Start with separated objects and controlled conditions. Confirm suitable real
objects and destinations with lab staff before committing to physical tests.

**Target:** October 20 for a complete V1, subject to physical access and results.
If access blocks physical validation, record the blocker and confirm an acceptable
demonstration route with organizers; do not present simulation as hardware evidence.

## How We Build It

Reuse the supplied scene, kinematics, grasp execution, and perception components
where they work. Train or fine-tune a model only if measured failures justify it.
We still need development and evaluation data even when no model is trained.

The starter's full benchmark stacks six cubes: yellow, yellow, blue, blue, red,
red. Preserve that benchmark and its checks. The one-cube diagnostic and our
incremental sorting milestones are separate tests, not passes of the full benchmark.

## Start Here

- [Rosie desktop and simulation guide](ROSIE_QUICKSTART.md)
- [Team plan and resource requests](TEAM_PLAN.md)
- [V1 checklist and task ownership](V1_CHECKLIST.md)
- [Architecture and data rules](ARCHITECTURE.md)
- [Contribution workflow](CONTRIBUTING.md)
- [Organizer instructions and submission links](MSAIC_INSTRUCTIONS.md)
- [Starter setup](fanuc_er4ia_sim/SETUP.md)

The Rosie guide documents Andrew's working account. Teammates need their own
repository access and compatible environments before those paths will work.

## After the Basic Loop Works

Evaluate example-based sorting instructions / in-context learning on unseen
arrangements without updating model weights. Compare against the same system
without those examples. Add other object types, uncertainty handling, extra
camera views, or recovery based on measured failures and available time.
Unsupported objects should be flagged; recognizing an object does not prove it
can be grasped by the existing fingers.

## Deliverables and Dates

- Team plan: **October 14, 11:59 PM CT**.
- Final submission: **November 1, 11:59 PM CT** — approximately five-minute video,
  one-page summary, repository link, references and credits.
- Finals: **November 5**.

Report measured performance, the manufacturing problem, limitations, and what
was demonstrated in simulation versus on hardware. Follow `CONTRIBUTING.md`;
keep generated runs, datasets, environments, and model weights out of Git.

## CURRENT WORKING BASELINE:
**Head** : 83029f9
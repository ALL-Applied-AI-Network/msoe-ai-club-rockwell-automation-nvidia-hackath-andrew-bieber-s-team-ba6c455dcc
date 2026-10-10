# Vision-Guided Grasping and Sorting

## The Idea
Give a robot eyes so it can find objects, pick them up,
and put them where they belong—even when their positions change.

We are using the supplied FANUC arm, cameras, digital twin,
and existing gripper fingers.

## V1: A Complete First Version
1. Find one supported object and locate it on the table.
2. Pick it up and move it to a defined destination.
3. Add a second object category and sort into two destinations.
4. Check the outcome and record successes and failures.
5. Measure performance against a baseline.
6. Demonstrate the system on the real arm during supervised testing.

Start with separated objects and controlled conditions.

Target: a submittable V1 within the first 10 days,
subject to equipment, twin, and testing access.

## What Makes V1 Submittable?
- A working demonstration on the real arm.
- Measured results and clearly stated limitations.
- Reproducible code and setup instructions.
- A five-minute video, one-page summary, and references.
- An explanation of the manufacturing problem and potential value.

## After V1
Possible improvements:
- More object types and varied orientations.
- Clutter and unexpected-object handling.
- Failed-grasp recovery.
- Additional camera views or custom fingers.

We will choose extensions based on measured failures and available time.

## Project Documents
- [Organizer instructions](MSAIC_INSTRUCTIONS.md)
- [Team proposal and resource requests](TEAM_PLAN.md)
- [Architecture](ARCHITECTURE.md)
- [Shared V1 checklist](V1_CHECKLIST.md)
- [Contribution workflow](CONTRIBUTING.md)

The architecture, checklist, and contribution documents are coming next.

## Development
Work flows through:
feature branches → integration → main.

Only Andrew merges integration into main.

## Current Status
Planning and setup. Technical choices remain provisional
until we inspect the supplied twin and approved robot interface.
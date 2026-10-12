# First Lab Visit — Grasping and Sorting

**Purpose:** collect the facts needed to connect our simulation work to the real FANUC cell.
**Success:** leave with an interface contact, a camera/data inventory, supported-object details, and a path to supervised testing. Record unknowns explicitly.

Visit date: __________  Staff contact: __________  Team lead: __________

Arrange the visit and recording permission beforehand. Staff operate the equipment. Observe existing operations; do not move cameras, change controller/PLC programs, introduce objects, or deliberately create failures.

## Student 1 — Cameras and files

Owner: __________

- [ ] Identify each available camera: overhead, wrist, side, or other. Do not assume the real cell has all three simulation views.
- [ ] With permission, obtain one original image from each available view. Label camera names and viewpoints.
- [ ] Ask how our software can access images: live stream, API, saved files, or staff export. Obtain documentation or the responsible contact.
- [ ] Record image resolution and frame rate if known. Ask whether frames have timestamps and whether multiple views are synchronized.
- [ ] Ask for camera calibration files and camera-to-robot transforms, including units, coordinate-frame names, and which camera/layout they apply to. Request a documented example if available; do not derive calibration on this visit.
- [ ] Note glare, shadows, blocked views, and whether the gripper hides the object during a pick.

**Bring back:** a camera inventory, approved sample-file locations, calibration/documentation links, and the owner of anything missing.

## Student 2 — Robot interface and testing access

Owner: __________

- [ ] Record the robot and controller model; ask which gripper/tool configuration is installed.
- [ ] Ask: **What approved method can student software use to request a pick-and-place?** Is there an existing program, command interface, or example?
- [ ] Ask what that interface expects: object coordinates, tool pose, joint targets, or a named action. Request its documented units and coordinate frame.
- [ ] Ask what feedback is available: robot ready/busy, command finished, gripper state, and errors. Do not assume a finished command proves a successful grasp.
- [ ] Identify who approves/runs tests and what must be reviewed before our code can be tested.
- [ ] Ask how to book an early supervised test and a later validation session, and how long each session can be.

**Bring back:** interface documentation/example or a named technical contact, plus the test-booking process and proposed dates. No credentials in notes or photos.

## Student 3 — Objects, workflow, and useful evidence

Owner: __________

- [ ] List the objects currently used and which staff say the existing fingers can reliably grasp.
- [ ] Ask whether two cube colours can be our first sorting groups. Record available sizes, orientations, and destination locations. Have staff provide dimensions or approve any measurement.
- [ ] Ask: **When parts are misplaced, what does an operator have to do today?** What delay or repeated work would our solution reduce?
- [ ] Ask staff to explain a normal cycle, a successful placement, and common failure cases.
- [ ] If permitted and available, observe/record 5–10 normal cycles, retaining failures as well as successes. Otherwise take labelled notes; do not induce failures.
- [ ] Record cycle times when observable and mark whether timing was measured or estimated.

**Bring back:** an object/destination list, a short explanation of the real problem, and labelled cycle notes or approved footage.

## Shared cycle log

Use the same trial ID across notes and camera files. Mark unavailable values as unknown.

| Trial ID | Object | Start position/orientation, if known | Destination | Pick outcome | Placement outcome | Cycle time | File / timestamp |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 001 | | | | | | | |

Use **success / failure / uncertain**. This small observation sample informs development; it is not our final performance evaluation.

## Before leaving

- [ ] Each student checks their deliverable with the team lead.
- [ ] Confirm allowed storage/use of images and video, including whether they may appear in the final submission.
- [ ] List missing items with the person to follow up with; never guess an answer.
- [ ] Note obvious differences between the real cell and twin, especially objects, gripper, cameras, and table layout.
- [ ] Agree who will request the supervised test slot.
- [ ] look at the power output, consumption and motor speeds.
## Return to the team

Each owner supplies a short note: **confirmed facts → evidence/file links → unresolved questions and contact**. Store media in the approved data location, not as large Git commits. Andrew and the implementation owners review the findings and select the first supported physical task.

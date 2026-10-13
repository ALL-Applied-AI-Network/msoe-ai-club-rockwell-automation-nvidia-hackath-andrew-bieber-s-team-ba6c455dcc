# V1 Checklist

## Goal

Find supported objects from camera observations, pick them up, sort them, and
report measured results, including supervised real-arm validation when access
is available.

**Target:** October 20. Confirm scope and physical access at the team meeting.
Assign one accountable owner and an evidence link to every deliverable.

## 1. Access and Setup

- [x] Obtain the supplied twin and starter source; include source in the team repository.
- [x] Confirm Isaac scene viewing runs on Rosie in Andrew's account.
- [x] Export a scene and capture images from the three simulated cameras.
- [x] Confirm manual joint movement in the GUI.
- [x] Run the 33 starter host tests successfully.
- [x] Prepare `ROSIE_QUICKSTART.md` with the working desktop procedure.
- [x] Confirm everyone has repository and competition-workspace access.
- [x] Confirm the complete roster, owners, and shared `integration` branch.
- [x] Have a second teammate reproduce the environment and open the scene.
- [ ] Review the supplied camera/gripper calibration and obtain matching real-cell files.
- [ ] Verify the dependencies/model setup required by the selected perception path.
- [x] Finalize and submit `TEAM_PLAN.md` by October 14, 11:59 PM CT.

Done when: setup is reproducible, ownership is clear, and remaining dependencies
are documented. A working viewer does not prove model or grasp execution.

## 2. Early Lab Visit — Run in Parallel

- [ ] Arrange the visit, staff supervision, and recording permission.
- [ ] Assign a camera/data owner: views, samples, access method, calibration files/contact.
- [ ] Assign an interface owner: approved commands, units/frames, feedback, documentation/contact.
- [ ] Assign an object/workflow owner: supported objects, destinations, normal cycles, operator problem.
- [ ] Return evidence and a list of unknowns with the person responsible for each answer.
- [ ] Confirm the approved physical-arm interface and review requirements.
- [ ] Request an early supervised test and a later validation session.

Done when: findings determine our first supported real task and the route to testing.
Staff operate equipment; the visit does not authorize changes to cameras or controls.

## 3. One Working Pick

- [x] Prepare a separate known-position one-cube diagnostic (`demo_pick.py`).
- [x] Check the diagnostic's 552 numerical IK targets.
- [X] Add the reviewed diagnostic to the repository through the contribution workflow.
- [X] Run the diagnostic on Rosie and inspect its logs and `demo_report.json`.
- [X] Confirm the cube actually grips, lifts, and reaches the destination.
- [X] Save a demonstration and reproduction instructions in the approved locations.

Done when: the object moves successfully and the evidence verifies pickup and
placement. This diagnostic is not a camera-based result or the full starter benchmark.

## 4. Give the Robot Eyes

- [x] Demonstrate image capture from the twin.
- [x] Reuse the supplied perception/geometry components where suitable.
- [x] Find the supported cube in a fresh image.
- [x] Convert its observed position into robot coordinates with explicit units and frame.
- [x] Estimate the cube orientation and align the gripper.
- [ ] Complete camera-based picks across varied starting positions.
- [ ] Reject missing, unsupported, or uncertain detections.

Done when: grasp targets come from camera observations, not hidden simulation poses.
Custom model training is only added if experiments establish the need.

## 5. Add Sorting

- [ ] Confirm two colour groups of the same supported cube shape as the initial scope.
- [ ] Define two destinations and an explicit sorting rule.
- [ ] Identify both groups and select the correct destination.
- [ ] Demonstrate repeated camera-based picks and correct placements.
- [ ] Keep objects separated and conditions controlled for V1.

Done when: both groups reach their assigned destinations and outcomes are checked.
Keep the supplied six-cube stacking benchmark and its original gates intact;
report our incremental task results separately.

## 6. Check and Measure

- [ ] Define success, failure, uncertainty, cycle-time boundaries, and the trial denominator.
- [ ] Check actual pickup and placement; retain every attempted trial.
- [ ] Log group, starting condition, outcome, code/model versions, settings, and timing.
- [ ] Define a baseline and compare under equivalent conditions, including varied positions.
- [ ] Keep evaluation arrangements separate from development and in-context examples.
- [ ] Run held-out simulation trials and summarize failures by group/condition.

**Proposed target:** 90% complete-task success across 100 held-out simulation trials.
Confirm feasibility after the initial pick experiments; this is not a result or
an organizer requirement.

## 7. Physical Validation

- [ ] Review the implementation with supervising staff.
- [ ] Validate camera coordinates, tool/robot frames, units, and approved commands.
- [ ] Test both supported groups at varied positions under supervision.
- [ ] Record every outcome and compare physical results with simulation.
- [ ] Fix major issues and retest if access permits.
- [ ] Record permitted footage for the submission.

**Proposed evaluation:** at least 20 physical attempts, subject to staff approval
and access. If blocked, document the limitation and confirm an acceptable demo
route with organizers. Do not label simulated execution as real-arm validation.

## 8. Submission Package

- [ ] Draft an approximately five-minute video and one-page summary.
- [ ] Explain the observed manufacturing problem and plausible business value.
- [ ] Report baseline comparisons, actual results, and remaining limitations.
- [ ] Credit supplied starter code, data, models, tools, and team contributions.
- [ ] Update README setup/results and have another teammate reproduce the demo.
- [ ] Andrew reviews and merges verified V1 work into `main`.
- [ ] Submit video, summary, repository link, and references by November 1, 11:59 PM CT.

## After the Core Loop Works

- [ ] Consider in-context examples for a changed sorting rule.
- [ ] Evaluate unseen arrangements with and without examples, without weight updates.
- [ ] Consider additional objects, views, or recovery only when justified by measured failures.

## Task Ownership

Names and dates are assigned at the meeting; each task has one accountable owner.
Andrew's completed setup work does not make him the default owner of every next task.

| Deliverable | Owner | Due | Status / blocker | Evidence / PR |
| --- | --- | --- | --- | --- |
| Initial Rosie setup and first render | Andrew | Completed Oct 10 | Verified in Andrew's account | Quickstart; link logs/captures |
| Teammate setup reproduction | TBD | TBD | Pending | — |
| Camera/data lab findings | Freshman 1 — assign name | TBD | Visit needed | — |
| Interface/test-access lab findings | Freshman 2 — assign name | TBD | Visit needed | — |
| Object/workflow lab findings | Freshman 3 — assign name | TBD | Visit needed | — |
| Known-position simulated pick | TBD | TBD | Diagnostic ready; physics unverified | `demo_pick.py` / report pending |
| Camera perception and coordinate conversion | TBD | TBD | Model/calibration verification needed | — |
| Two-colour sorting | TBD | TBD | Depends on camera-based pick | — |
| Outcome checks, logging, baseline and evaluation | TBD | TBD | Define trial protocol early | — |
| Physical integration and supervised validation | TBD | TBD | Approved interface/access needed | — |
| Submission drafts | TBD | TBD | Outline can begin now | — |
| Final integration and release review | Andrew | TBD | Pending | — |

## Working Rules

Update status when progress or blockers change. Raise blockers early and explain
what you tried. Link evidence before marking a task complete. Keep the October 20
target visible and revise scope openly if needed.

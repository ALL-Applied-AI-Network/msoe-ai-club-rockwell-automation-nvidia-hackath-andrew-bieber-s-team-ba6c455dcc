# V1 Checklist

## Goal
A complete, submittable first version:
find supported objects, pick them up, sort them,
and demonstrate measured results on the real arm.

Target: October 20, subject to twin availability and physical access.
Owners and task deadlines will be assigned at the team meeting.

## 1. Access and Setup
- [ ] Everyone can access the repository and competition workspace.
- [ ] Review compute, tools, and data-collection guides.
- [ ] Obtain the twin, starter code, camera samples, and calibration files.
- [ ] Confirm where the twin can run.
- [ ] Render the workspace and document how to reproduce it.
- [ ] Confirm the approved physical-arm interface.
- [ ] Request an early supervised test and a later validation session.
- [ ] Finalize and submit TEAM_PLAN.md.

Done when: the twin runs, access is documented, and physical
integration/testing arrangements are understood.

## 2. One Working Pick
- [ ] Select one object supported by the existing fingers.
- [ ] Run a simulated pick-and-place using a known object position.
- [ ] Confirm grip, lift, and placement actually succeed.
- [ ] Save a demonstration and reproduction instructions.

Done when: the object is successfully moved to its destination,
not merely when the arm executes a motion.

## 3. Give the Robot Eyes
- [ ] Capture camera images from the twin.
- [ ] Find the supported object in the image.
- [ ] Convert its location into robot coordinates.
- [ ] Account for orientation where needed.
- [ ] Complete picks across varied starting positions.
- [ ] Flag missing or uncertain detections.

Done when: the pick location comes from camera observations
rather than a fixed position or hidden simulation coordinates.

## 4. Add Sorting
- [ ] Select a second supported object category.
- [ ] Define two destinations and the sorting rule.
- [ ] Identify both categories and choose the correct destination.
- [ ] Demonstrate repeated picks and correct placements.
- [ ] Keep objects separated and conditions controlled.

Done when: both categories are moved to their assigned destinations.

## 5. Check and Measure
- [ ] Define success and failure before evaluation.
- [ ] Check whether each task actually completed.
- [ ] Log every attempt, including failures and uncertain outcomes.
- [ ] Record object category, starting pose, outcome, and cycle time.
- [ ] Record code/model versions and simulation settings.
- [ ] Compare against an agreed baseline under equivalent conditions.
- [ ] Run held-out simulation trials.
- [ ] Summarize results by category and explain common failures.

Proposed simulation target: 90% complete-task success across
100 held-out trials. Confirm this target after initial experiments.

## 6. Physical Validation
- [ ] Review the system with the plant manager and assigned mentor.
- [ ] Validate camera coordinates and approved robot commands.
- [ ] Test both categories at varied positions under supervision.
- [ ] Record every outcome and compare with simulation.
- [ ] Fix major issues and retest if access permits.
- [ ] Record footage of the real system operating.

Proposed evaluation: at least 20 physical attempts.
Confirm feasibility with the supervising staff.

## 7. Submission Package
- [ ] Draft an approximately five-minute video.
- [ ] Draft the one-page project summary.
- [ ] Explain the manufacturing problem and potential business value.
- [ ] Report baseline comparisons and remaining limitations.
- [ ] Add references and credits for data, code, models, and tools.
- [ ] Document setup and execution in README.md.
- [ ] Have another teammate reproduce the simulation.
- [ ] Andrew reviews and merges the verified V1 into main.

Done when: the working hardware demonstration, results,
repository, video, summary, and references are ready to submit.

## Task Ownership
Each task has one accountable owner. Others can help.
Split large tasks into smaller deliverables as needed.

| Task / deliverable | Owner | Due | Status / blocker | Evidence / PR |
| --- | --- | --- | --- | --- |
| Twin setup and first render | TBD | TBD | Not started | — |
| Approved arm interface and test scheduling | TBD | TBD | Not started | — |
| Known-position simulated pick | TBD | TBD | Not started | — |
| Camera-based object detection | TBD | TBD | Not started | — |
| Camera-to-robot coordinate conversion | TBD | TBD | Not started | — |
| Two-category sorting | TBD | TBD | Not started | — |
| Outcome checks and trial logging | TBD | TBD | Not started | — |
| Baseline and evaluation | TBD | TBD | Not started | — |
| Physical validation and recording | TBD | TBD | Not started | — |
| Submission drafts | TBD | TBD | Not started | — |
| Final integration and release review | Andrew | TBD | Not started | — |

## Working Rules
- Update your task status when progress or blockers change.
- Raise blockers early and explain what you tried.
- Link evidence before marking a task complete.
- Add new features only after the V1 requirements work.
- Keep the October 20 target visible; revise scope openly if needed.
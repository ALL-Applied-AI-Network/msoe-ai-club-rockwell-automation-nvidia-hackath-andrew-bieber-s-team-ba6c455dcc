# Team Plan — Draft

## Team

- Name: TBD
- Mentor: Andrew Bieber
- Listed members: Kshitij Batra, Peter Gerrits, Asher Jaenchen, Zachary Millmann,
  Evan Ohme, Khalil Spires
- Confirm the complete roster and assign accountable owners at the team meeting.

## Challenge and Problem

Dataset 3: Grasping and Sorting.

Fixed robot motions can fail when objects appear in unexpected positions.
We want the supplied FANUC arm to find supported objects and move them to the
correct destinations. During the lab visit, confirm the operator intervention
or delay this would reduce and how the current process handles misplaced parts.

## First Working Version

- Reuse the supplied digital twin, existing gripper fingers, and starter code.
- Verify one known-position simulated cube pick before adding camera-based targeting.
- Start sorting with two colour groups of the same supported cube shape.
- Use separated objects, controlled lighting, and defined destinations.
- Check pickup and placement from evidence and record failures and uncertainty.
- Measure complete-task success and cycle time against an agreed baseline.
- Aim to demonstrate the approved task on the real arm during supervised testing.

**Target:** October 20, subject to test access and experimental results.
Physical object compatibility, destinations, and command access must be confirmed.

The starter's full six-cube stacking benchmark remains intact and separate from
our incremental diagnostics. A one-cube success is not completion of that benchmark.

## Current Evidence — October 10

Rosie supports interactive scene viewing, manual joint movement, and simulated
camera capture in Andrew's account. The 33 host tests pass. The one-cube diagnostic
passes numerical IK checks; an actual simulated grasp is the next test.

The complete perception/execution pipeline, Cosmos loading, real-arm integration,
and reproducibility in another teammate's account are not yet verified.

## Technical Approach

Use camera observations, calibrated geometry, the supplied kinematics and motion
code, and suitable existing perception models. No custom model training is required
for the first mechanics test. Train or fine-tune only if measured failures justify it.

After the basic loop works, evaluate in-context examples for adapting sorting
rules. Test on held-out arrangements and compare with a no-example baseline.
Do not rely on a language model to supply precise motor commands or assume an
unfamiliar object is physically graspable.

## Compute and Requested Tools

### Compute: Rosie First; Brev Contingency

Rosie is our current development environment for simulation and evaluation.
Initial viewing, capture, and manual motion work; full-pipeline performance,
model compatibility, and concurrent team access still need testing.

Retain an approximately **$60 Brev contingency request**, subject to approval,
if Rosie availability or compatibility blocks an essential workload. Select the
instance and estimate active hours from the actual workload and current quote;
no fixed hourly rate or hardware requirement is assumed here.

Record environment versions, runtime, and any credits used. End idle allocations
and stop paid instances between sessions. Request additional resources only with
a specific blocker or measured need. Initial setup does not establish support for
all future model-training workloads.

### ChatGPT Pro: Three Individual Plans

- Recipients: Andrew Bieber and two teammates taking substantial implementation
  ownership; teammate names TBD.
- Duration: one month during the competition.
- Proposed budget: $200 per person; $600 total, subject to organizer approval and
  verification of current pricing/provisioning.
- Preference: provision Andrew's existing personal account if supported, to preserve
  project context. Confirm the organizer's account/provisioning process.

These plans support code development and review, debugging, robotics-tool learning,
integration, and mentoring preparation. Each recipient uses their own account and
is responsible for understanding and verifying generated work.

### ChatGPT Plus

No paid Plus plans requested. Any student promotion must be verified individually;
the project does not depend on a promotion being available.

### Additional Requests

| Resource | Owner | Purpose or blocker | Quantity / duration | Estimated cost |
| --- | --- | --- | --- | --- |
| TBD | TBD | TBD | TBD | TBD |

Andrew reviews and consolidates justified requests before submission.

## Early Lab Visit

Assign three students distinct evidence-gathering roles during an arranged,
staff-supervised visit:

| Role | Required result |
| --- | --- |
| Cameras and calibration | Actual camera views, sample images, data access method, calibration files/contact |
| Robot interface and access | Approved command method, documentation/example or contact, supervised-test booking process |
| Objects and workflow | Supported objects, destinations, normal-cycle observations, operator problem and available timing evidence |

Staff operate the equipment. Obtain recording permission. Do not move cameras,
change programs, introduce unapproved objects, or deliberately create failures.
Bring back confirmed facts, evidence links, and unresolved questions with contacts.

## Remaining Questions

- Which real objects can the existing fingers handle, and are our proposed colour groups suitable?
- What approved interface can our software use, with which units and coordinate frames?
- Which camera streams and current calibration files can we access? Are multiple views required?
- What feedback is available to check real pickup and placement?
- When can we book an early supervised test and a later validation session?
- Can a second teammate reproduce our setup and can the complete pipeline run within our resource limits?

The supplied starter includes grasp execution code; our next step is verifying it
in our environment. The organizer instructions supplied in the repository state
the plan deadline is October 14, so the earlier October 13/14 question is resolved.

## Team Review and Submission

- Suggestions and resource requests: **Monday, October 12, 6 PM CT**.
- Team plan due: **Wednesday, October 14, 11:59 PM CT**.
- Final submission due: **Sunday, November 1, 11:59 PM CT**.

Andrew consolidates the plan and submits through the organizer's workspace.

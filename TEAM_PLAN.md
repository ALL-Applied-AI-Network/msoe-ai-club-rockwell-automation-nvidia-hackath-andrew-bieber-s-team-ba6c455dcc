# Team Plan — Draft

## Team
- Name: TBD
- Mentor: Andrew Bieber
- Members: Kshitij Batra, Peter Gerrits, Asher Jaenchen, Zachary Millmann, Evan Ohme, Khalil Spires

## Challenge
Dataset 3: Grasping and Sorting.

## Problem
Fixed robot motions fail when objects are in unexpected positions.
We want the arm to find supported objects and move them to the correct destinations.

## First Working Version
- Use the supplied digital twin and existing gripper fingers.
- Start with one object, then add a second category for sorting.
- Test separated objects at varied positions.
- Measure successful picks, correct placements, and cycle time against a baseline.
- Demonstrate the solution on the real arm during supervised testing.

Target: a complete, submittable V1 within the first 10 days, subject to twin availability and physical testing access.

## Compute and Requested Tools

### Compute: MSOE Rosie
We plan to use Rosie for synthetic-data generation, model training, and evaluation. We expect it to meet our V1 compute needs, pending confirmation that the supplied Isaac Sim twin can run in the available environment.

No Brev credits requested initially. We will update the request if compatibility or access requires alternative compute.

### ChatGPT Pro: Three Individual Plans
- Recipients: Andrew Bieber and two teammates taking substantial implementation ownership (names TBD).
- Account preference: Andrew would appreciate having Pro provisioned on his existing personal ChatGPT account, which already contains the project context, planning discussions, and mentoring preparation. This would preserve continuity and avoid rebuilding that context in a separate account.
- Duration: one month during the competition.
- Requested budget: $200 per person; $600 total, subject to organizer approval and current pricing.

These plans would support sustained work on unfamiliar simulation and robotics tools, code development and review, debugging, and integration. Andrew would also use the tool to prepare clear explanations and manageable tasks for teammates. Each recipient would use their own account and remain responsible for understanding and verifying generated work.

### ChatGPT Plus
No paid Plus plans requested. Teammates will check eligibility for the student promotion reported by the team; availability and terms remain to be verified. Our project plan does not depend on that promotion.

### Additional Team Requests
Teammates can add justified needs before the team review deadline.

| Requested tool or resource | Person requesting | Purpose / blocker addressed | Quantity / duration | Estimated cost |
| --- | --- | --- | --- | --- |
| TBD | TBD | TBD | TBD | TBD |

Andrew will review additions and consolidate the final resource request.

## Technical Support Needed
- Digital twin, starter code, camera calibration, and object details.
- Guidance on camera-to-robot coordinates and the approved physical-arm interface.
- An early supervised test and a later validation session.

## Questions
- Is a working pick-and-place example supplied?
- How can our software command the physical arm?
- Which objects can the existing fingers handle?
- Are multiple camera views required?
- Can the supplied twin run on our Rosie environment?
- When can we test on the real arm?
- Is the plan deadline October 13 or October 14?

## Team Review
Suggestions and additional resource requests due: Monday, October 12, 6 PM CT.
Andrew reviews changes and submits the final plan.


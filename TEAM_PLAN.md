# IRIS: Intelligent Robotic Identification and Sorting

## Team

- Team: Andrew Bieber’s Team
- Mentor and team lead: Andrew Bieber
- Listed members: Kshitij Batra, Peter Gerrits, Asher Jaenchen, Zachary Millmann, Evan Ohme, Khalil Spires
- Confirm the complete roster and assign accountable owners at the team meeting.

## Challenge and Problem

**Dataset 3: Grasping and Sorting**

Fixed robot motions fail when objects appear in unexpected positions. IRIS will give the supplied FANUC arm the ability to perceive an unsorted workspace, identify and locate supported objects, pick them one at a time, and place them into the correct sorting destinations.

During the lab visit, the team will confirm the real operator problem, supported objects, sorting rules, and the intervention or delay our solution could reduce.

## First Working Version

- Reuse the working digital twin, gripper, camera views, and starter code.
- Replace predetermined object coordinates with camera-estimated coordinates.
- Begin with separated cubes sorted into two colour groups.
- Detect, pick, and sort every object in the workspace.
- Record pickup success, placement success, cycle time, and uncertainty.
- Move low-confidence objects to a defined review location.
- Demonstrate the validated task on the physical arm during supervised testing.

**Target:** October 20, subject to physical-cell access and experimental results.

Physical object compatibility, destinations, and command access must still be confirmed.

## Current Evidence — October 11

The complete Isaac Sim digital twin is running with the FANUC robot, gripper, workspace, and camera views. The 33 host tests pass.

The robot has completed an actual simulated pick-and-place cycle, including approach, descent, grasp, lift, carry, placement, release, and retreat. The cube rose 129.9 mm, and the automated report confirmed both pickup and placement.

The next milestone is replacing predetermined object coordinates with camera-derived coordinates and extending the demonstration from one object to multi-object sorting.

Physical-cell integration and reproducibility in another teammate’s environment are not yet verified.

## Technical Approach

We will use the digital twin’s camera observations, calibrated geometry, supplied kinematics and motion code, and suitable pretrained open-source vision models.

We will evaluate existing detection, segmentation, and vision-language models before training a custom model for an already well-studied problem. We will train or fine-tune only if measured failures justify it.

After the basic sorting loop works, we may evaluate in-context examples for identifying miscellaneous objects or adapting sorting rules. Low-confidence objects will be moved to a safe review location. A language model will not provide precise motor commands or assume that an unfamiliar object is physically graspable.

Testing will use held-out object arrangements and measure complete-task success, pickup success, placement success, uncertainty, and cycle time.

## Compute and Requested Tools

### Compute: BrevDell GB10 at MSOE (Rosie)

Request approximately **20–30 GPU-hours** to run the digital twin, evaluate pretrained vision models, generate synthetic images, and test the complete pick-and-sort pipeline.

Additional compute will be requested only if model fine-tuning or larger-scale synthetic-data generation becomes necessary.

### AI Agent Tools

We will begin with the free Gemini models. Several teammates may also qualify for the free ChatGPT Plus college-student promotion, leaving only the primary developers needing additional access.

Request **three $100/month Codex plans** for Andrew Bieber and the two students leading vision and robot integration. They will use Codex to develop, debug, and test these components in parallel.

Each recipient will use an individual account and remain responsible for understanding, testing, and verifying generated work.

## Early Lab Visit

A three-student first-year subteam has been assigned distinct evidence-gathering roles for an arranged, staff-supervised lab visit.

| Role | Required result |
|---|---|
| Cameras and calibration | Actual camera views, sample images, data-access method, and calibration files or contact |
| Robot interface and access | Approved command method, documentation or contact, and supervised-testing process |
| Objects and workflow | Supported objects, sorting destinations, normal-cycle observations, operator problem, and timing evidence |

Staff will operate the equipment. Students will obtain recording permission and will not move cameras, change programs, introduce unapproved objects, or deliberately create failures.

Each student will return with confirmed facts, evidence links, unresolved questions, and the appropriate contacts. The team may supplement the collected data with synthetic images generated in Isaac Sim.

## Mentor Support Requested

- **NVIDIA:** Isaac Sim and computer-vision integration.
- **Rockwell Automation:** Physical robot interface, workcell requirements, and safety.
- **Emulate3D / CSI:** Transferring the digital-twin solution to the real workcell.

## Remaining Questions

- Which objects can the existing fingers reliably grasp?
- Are two cube colours suitable for the first sorting groups?
- What approved interface can our software use, and what units and coordinate frames does it expect?
- Which camera streams and calibration files can we access?
- What feedback is available to verify real pickup and placement?
- When can we schedule an early supervised test and a later validation session?
- How will successful picking and sorting be evaluated?
- Are pretrained models, synthetic data, and vision-language models allowed?
- Can another teammate reproduce the simulation environment?

## Team Review and Submission

- Suggestions and resource requests: **Monday, October 12, 6:00 PM CT**
- Team plan due: **Wednesday, October 14, 11:59 PM CT**
- Final submission due: **Sunday, November 1, 11:59 PM CT**

Andrew will consolidate the plan and submit it through the organizer’s workspace.
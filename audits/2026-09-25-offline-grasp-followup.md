# Offline grasp follow-up — September 25, 2026

Scope: repository source and the rotated logs from run
`20260924-230930-openyam-coordinator-agentic-openyam-grasp-graspgenx-agent`.
The arm was unavailable. No hardware commands, blueprint startup, tests, or
dependency changes were performed. Changes are confined to this repository.

## What the recorded run establishes

- Thirteen candidate path rejections: ten Pink IK iteration-budget failures
  and three joint-limit failures. These are solver outcomes, not proof that
  every equivalent grasp is physically unreachable.
- Nineteen perception/generation passes retained 0–12 candidates, usually
  0–3. Although 400 proposals were requested, only 21–134 passed the generator's
  confidence threshold in these passes. Direction, table/scene clearance and
  opposing-contact gates then reduced that pool further.
- Twelve Cartesian timing records span approximately 2.28–2.95 seconds.
  These include feasibility paths, executed insertions, and retreats, so they
  are not twelve physical grasp attempts. The extreme slowdown in the earlier
  run is absent from these records.
- Four completed insertion command sequences ended with approximately 16.1,
  9.8, 13.6, and 34.2 mm TCP error. Their vertical components were approximately
  +13.8, +7.1, +12.0, and +31.4 mm. Two passed the supported-contact gate and
  closed, but both were reported empty. The logged post-close reading near
  0.99 is after the inherited empty-grasp recovery reopened the jaws; the
  verification messages contain the actual closed readings, 0.010 and 0.012.
- One retreat missed its 5 mm convergence gate by reaching approximately
  9.4 mm error, and one retry failed home settling. These are execution issues,
  separate from IK. They remain failures under the current pose tolerances.
- A pregrasp at 21:18:51 UTC commanded wrist joint 4 to -1.6894 rad, only
  0.0036 rad above its configured lower limit of -1.69297. Its measured final
  value was approximately -1.4578 rad, and TCP error reached 47 mm / 12.89°.
  The logs cannot establish whether obstruction, holding error or a physical
  limit caused this discrepancy.

Late transport/device errors belong to the later offline period and are not
used to explain the preceding grasp planning failures.

## Target-offset trace

The generator returns gripper-body poses. The adapter calculates
`world_from_tcp = world_from_grasp @ grasp_frame_to_tcp`. The 0.12 m translation
is along the generated local approach axis and matches the URDF's
`gripper_tip_joint`; deleting it without redefining the TCP would create a
120 mm targeting error. No additional world-Z translation is applied to the
contact target, and no 200 mm contact-height adjustment was found.

The 100 mm axial pregrasp and 100 mm post-close vertical lift are separate
waypoints. The preexisting collision model extension is 20 **mm**, not 20 cm,
and does not add a target-pose translation. Workspace calibration and these
gripper frame definitions were preserved.

The recorded tracking errors vary by pose and motion direction. A constant
vertical correction is not justified by these observations. Previously noted
depth-plane bias and physical TCP uncertainty also cannot be resolved offline.

## Changes implemented

1. **Equivalent jaw orientations.** After a retained candidate fails planning,
   also try a 180° rotation about its TCP approach axis. Contact position and
   approach direction stay fixed. The full geometry filter is rerun because
   symmetric jaw contact does not establish symmetric palm/robot clearance.
   This restores a planning alternative previously removed by symmetry-aware
   diversity filtering.
2. **Shorter pregrasp fallback.** Try 100 mm axial standoff first, then 60 mm
   for the same contact. Both orientations are considered at each distance.
   All options undergo transit and insertion collision checks. The total
   planning-option budget is 20, and the 30-second observation age limit remains.
3. **Reuse checked transit.** Retain and execute the feasibility transit by its
   plan ID instead of solving its IK and joint path a second time. Require fresh
   encoder feedback within 0.03 rad per joint of that plan's start. The insertion
   is still replanned from measured feedback after pregrasp settling. The old
   double solve was a consistency defect; these particular logs do not show a
   large branch switch between the two solves.
4. **Consistent Cartesian feasibility.** Check a direct path from the transit's
   FK endpoint to contact, matching execution. Remove the tiny intermediate
   correction to the ideal pregrasp, which unnecessarily introduced another
   waypoint/corner into feasibility timing.
5. **IK configuration.** Expose position/orientation convergence at 2 mm / 1°,
   always capped by the existing 3 mm / 2° endpoint gate. The old upstream
   wrapper required 1 mm / 0.57°. Keep ten solver attempts, raise iteration
   budget from 200 to 300, and set the inward joint-limit posture target margin
   to 0.05 rad. Log solver status, errors and iteration count.
6. **Avoid joint-stop grasps.** Require 0.05 rad clearance from configured joint
   limits for the pregrasp endpoint and checked insertion path. This selects
   another candidate/variant when possible; it does not change joint limits,
   controller gains, or the folded home target.
7. **Proposal supply.** Increase the bench profile from 400 to 800 proposals.
   Keep confidence at 0.7 and preserve contact, direction and collision gates.
   This increases inference/filtering work; improvement in candidate yield is
   expected but has not been measured.
8. **Feedback bookkeeping.** Skip duplicate still-fresh observations instead
   of resetting settling or immediately failing hold verification. Continue
   rejecting stale feedback, require distinct samples, and record home joint
   errors and execution phases on failures.

Relevant implementation: [motion planning](../src/openyam_coordinator_agentic/checked_motion.py),
[pick workflow](../src/openyam_coordinator_agentic/pick_and_place_module.py),
[bench profile](../workspace-config/openyam_bench.json).

## Remaining limits

These changes address avoidable planning failures and sampling scarcity. They
do not demonstrate that contact tracking, empty closes, or depth bias are fixed.
Physical performance, inference latency with 800 proposals, and the revised IK
settings have not been tested. Collision, encoder convergence, observation-age,
supported-contact and holding guards remain enabled. The updated code will load
on the next requested blueprint restart.

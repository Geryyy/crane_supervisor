# crane_supervisor

The supervisor of [[control_architecture]] §5, in the state PRD §2 slice 3 leaves
it: **status and mode only**.  It watches, it reports, and it does not act.

- a **ROS-free decision core** — `include/crane_supervisor/supervisor_core.hpp`.
  An input struct in, a decision struct out.  No node handle, no message type, no
  clock, no topic, so every cause is reachable from a unit test with no ROS
  runtime in the process.
- a **node** — `crane_supervisor`, which owns the I/O and the timing and nothing
  else.  It publishes `crane_msgs/SupervisorStatus` on `/crane/supervisor/status`
  at 20 Hz, reliable, depth 1, the stream ROS 2 Interfaces §2 and §4 fix and
  `crane_msgs`' own ROS contract test already asserts.
- one **input carried end to end** — `crane_msgs/PendulumState` on
  `/crane/pendulum_state`.

## It has no stop authority, and that is deliberate

§5.2 gives the supervisor a path to zero that does not run through the active
controller, because every trigger it exists for is a symptom of something
upstream having failed.  That path is blocked on
[[commissioning_prerequisites]] row 3: nobody has verified that a software stop
reaches a machine stop, and the owner of that verification is a safety reviewer
on the machine, not an agent and not this package.

Until it is verified the system must not advertise a safety function it cannot
perform (PRD user story 54).  So this package holds **no publisher on a command
topic, no controller-manager client, and nothing that can deactivate a
controller** — and `test/test_no_command_path.py` asserts the absence, because an
absence nobody checks is one that comes back by accident.  It is withheld, not
unfinished.

The status stream is worth having on its own terms: health and mode become
visible while the safety path is still being commissioned (PRD user story 64),
and the behaviour tree gets a **typed cause** to branch on instead of the
inferred abort §5.0 describes.

## What one report says

`fault` is a constant of `crane_msgs/SupervisorStatus` and `message` says why in
words an operator can act on.  Every report carries a cause (PRD user story 53) —
a status with `fault != FAULT_NONE` and an empty `message` is a test failure.

Only two of the ten constants are ever reported here:

| Report | When |
|---|---|
| `FAULT_STATE_HEALTH` | nothing has arrived on `/crane/pendulum_state` yet, or the stream stopped, or its stamp is further in this node's future than the margin, or `pendulum_state_broadcaster` marked the sample unusable |
| `FAULT_NONE` | the passive joint state is arriving inside its margin and the broadcaster reports it usable |

`FAULT_NONE` from this supervisor means *nothing it watches is wrong*, not *the
machine is safe*, and the `message` on a clear report says so.  Tracking, working
cell, solver, sway, reference, emergency stop and interlock are later issues;
`mode` is `MODE_IDLE` and nothing else until mode arbitration exists, because a
supervisor must never report a mode it has not confirmed.

**Absence is not health.**  §5.3 allows no input to stop arriving without a
defined consequence, so an input that never arrived and one that stopped are both
faults rather than a quiet `FAULT_NONE`.  The general staleness policy — per
input, per consequence — is a later issue; what is owed here is that the one
input this slice carries cannot be silently missing.

**The broadcaster's own cause is carried through, not restated.**
`pendulum_state_broadcaster` separates six causes behind `valid == false` and
says which in its `status` string.  That string is copied into `message`
verbatim, because restating it would flatten a distinction the estimator went to
some trouble to make.

## What is not computed

`tracking_error` is `0.0`, `inside_working_cell` is `false` and `deadman_held` is
`false`.  None of the three is computed in this slice, and each carries the value
that claims nothing rather than a stub that claims something.  In particular the
virtual working cell of §5.1 needs forward kinematics that `crane_model`'s
production backend does not have until slice 4, and an `inside_working_cell`
computed from nothing would be worse than one that is visibly not computed yet.

## Configuration

One parameter, `pendulum_state_timeout`, declared in
`src/crane_supervisor_parameters.yaml` and shipped in
`config/crane_supervisor.yaml`.  Everything else about this node is a contract
rather than a setting: the topic names and the 20 Hz rate are constants in the
source, so no deployment can rename or re-rate a stream the task layer and the
operator panel subscribe to.

## Composition

The `fake` and `hardware` profiles of `crane_bringup` compose the node; `sim`
does not, exactly as it did not for the pendulum broadcaster.  It is a node
beside the controller manager, not a controller inside it: it claims no
interface, no spawner names it, and the launch gives it no remapping.

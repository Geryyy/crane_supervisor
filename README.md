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
- three **inputs carried end to end** — `crane_msgs/PendulumState` on
  `/crane/pendulum_state`, `epsilon_crane_msgs/RemoteCtrlStates` on
  `/crane/remote_ctrl_states`, and `control_msgs/JointTrajectoryControllerState`
  on `/crane/controller_state`.
- one **service** — `/crane/clear_fault` (`std_srvs/Trigger`, ROS 2 Interfaces
  §5), which acknowledges a latched emergency stop and does nothing else.

> **The emergency stop and the deadman are read here for diagnosis and clean
> recovery. They are not the protection.** The stop chain is hardware and PLC
> (§6.1); it acts whether or not this software is running, and no safety
> argument may take credit for anything in this package. `FAULT_ESTOP` on the
> status stream means *this supervisor noticed*, never *this supervisor stopped
> the machine*.

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

Reading the emergency stop does not change that, and the guard did not have to be
relaxed to let it in.  What it lost was a blanket ban on `create_service`, which
was only ever there because `/crane/clear_fault` was a later issue; what replaced
it names the one service this package may serve and the one type it may serve it
with, so `/crane/set_mode` — still a later issue — cannot arrive unnoticed.
`std_srvs/Trigger` carries no request fields at all, which is what makes it safe
to expose from a node with no authority: there is nothing in it for a caller to
ask this supervisor to do.

Reading the trajectory controller's state cost the guard one more relaxation, of
the same shape.  `JointTrajectory` was on the banned-identifier list as a
stand-in for `trajectory_msgs`, the package a *commanded* trajectory is typed
with — and it also matched `control_msgs/JointTrajectoryControllerState`, which
is a controller describing itself.  The ban is now on `trajectory_msgs` by name,
so a publisher of the command type still cannot appear, and the guard pins the
exact three message types this node subscribes to rather than trusting a
substring.

The status stream is worth having on its own terms: health and mode become
visible while the safety path is still being commissioned (PRD user story 64),
and the behaviour tree gets a **typed cause** to branch on instead of the
inferred abort §5.0 describes.

## What one report says

`fault` is a constant of `crane_msgs/SupervisorStatus` and `message` says why in
words an operator can act on.  Every report carries a cause (PRD user story 53) —
a status with `fault != FAULT_NONE` and an empty `message` is a test failure.

Only five of the ten constants are ever reported here, and they are resolved in
this order:

| Report | When |
|---|---|
| `FAULT_ESTOP` | `em_stop` is asserted on `/crane/remote_ctrl_states`, **or** that stream is not arriving at all, **or** a stop that was one of those is latched and not yet acknowledged |
| `FAULT_STATE_HEALTH` | nothing has arrived on `/crane/pendulum_state` yet, or the stream stopped, or its stamp is further in this node's future than the margin, or `pendulum_state_broadcaster` marked the sample unusable |
| `FAULT_STATE_HEALTH` | the trajectory controller's own state has never arrived on `/crane/controller_state`, or it stopped, or its stamp is too far ahead — a controller that stopped publishing is not a crane that is tracking perfectly |
| `FAULT_TRACKING` | an actuated axis is outside **its own** velocity-tracking tolerance |
| `FAULT_INTERLOCK` | the remote is arriving, the stop is clear, and the configured deadman button is not held |
| `FAULT_NONE` | all three streams are arriving inside their margins, the broadcaster reports the sample usable, no axis is outside its tolerance, nothing is latched and the deadman is held |

**The order is not arbitrary.**  The stop is first because it is the only one of
them with no field of its own: a cycle that reported something else instead
would not report it at all.  The passive state comes before tracking because a
supervisor whose own view of the crane is stale should say that before it says
anything derived.  The controller state's freshness is judged immediately before
the comparison that consumes it, since an error that is not arriving cannot be
compared to anything.  The interlock is last because the deadman *does* have a
field — `deadman_held` is filled on every report whatever `fault` says — so
putting a released button, which is the ordinary resting state of the machine,
above a defect would hide the defect behind a routine.  One consequence to know
before reading a panel: a supervisor started before `gpio_controller` sits in
`FAULT_ESTOP` until the remote arrives, which is what treating absence as
asserted means in practice.

`FAULT_NONE` from this supervisor means *nothing it watches is wrong*, not *the
machine is safe*, and the `message` on a clear report says so.  Working cell,
solver, sway and reference are later issues; `mode` is `MODE_IDLE` and nothing
else until mode arbitration exists, because a supervisor must never report a
mode it has not confirmed.

## The stop chain, consumed in one direction only

Three rules, each §6.1's or §6.2's rather than this package's:

**Absence is asserted.**  A remote that never arrived, one that stopped, and one
whose stamp is further in this node's future than the margin all raise
`FAULT_ESTOP`, because a dead GPIO reader, a crashed driver and a released button
must not look alike (§5.3).  Each cause says which one it is in `message`, and a
released *button* is a different constant entirely — a test asserts that no two
of the five reports read the same.

**The stop latches.**  It survives `em_stop` going back to released, so the
software comes back in a defined state instead of resuming from whatever it was
doing when the machine stopped underneath it.  `/crane/clear_fault` is the only
thing that lowers it.

**The deadman is continuous.**  It is re-checked on every status cycle rather
than once at the start of a motion, and it clears itself the cycle after the
button is pressed again: an interlock is a fact about the operator, not a latch.

`/crane/clear_fault` answers in three ways and none of them is an empty success
(ROS 2 Interfaces §1):

| `success` | When | Why |
|---|---|---|
| `false` | `em_stop` is still asserted, or the remote is still not arriving | the latch is not what is holding the fault up, the condition is; clearing it would report a success the next cycle undoes |
| `false` | nothing is latched | an acknowledgement of nothing is reported as a no-op rather than as a clear, so `success == true` always means *a latch existed and is now down* |
| `true` | a latch existed and the condition behind it is gone | and if the stop recurs the latch is raised again on the next cycle, which is the specified behaviour and not a failed clear |

**Nothing here acts.**  No stop, no ramp, no deactivation, no command — on the
emergency stop least of all, since §6.1 makes the software's relationship to the
stop chain supplementary and one-directional.  Refusing new goals while the stop
is latched is a *mode* decision and belongs to the issue where this supervisor
first has authority over anything.

## Which button the deadman is

`deadman_button` defaults to **12**, and the number is read from the retained
stack rather than inferred from the message:

- `epsilon_crane_behavior_tree`'s approval gate returns `msg.button12` —
  `src/plugins/condition/check_user_approval.cpp` and
  `include/epsilon_crane_behavior_tree/plugins/decorator/get_user_approval.hpp`,
  whose comment says "`RemoteCtrlStates::button12` has to be pressed and held".
- the manual-control TUI defines `KEYCODE_BUTTON12` as `0x7A`, the `z` key
  (`timber_crane_manual_control/timber_crane_tui/src/remote_ctrl.cpp`) — and `z`
  is the key §6.2 names as the simulated deadman.
- the simulation's remote publisher sets `button12` to approve
  (`epsilon_crane_bringup_sim/test/remote_ctrl_publisher.cpp`).

`gpio_controller` copies the twelve booleans off the state interfaces in order
and names none of them the deadman: it fixes the wire index, and the approval
gate fixes the meaning.  It is a parameter and not a constant because it is a
property of a remote's wiring, which the architecture does not own — but it is
read-only, because which button stops the machine is not a runtime adjustment.

**Absence is not health.**  §5.3 allows no input to stop arriving without a
defined consequence, so an input that never arrived and one that stopped are both
faults rather than a quiet `FAULT_NONE`.  The general staleness policy — per
input, per consequence — is a later issue; what is owed here is that neither
input this package carries can be silently missing, and that the stop signal's
absence is read as asserted rather than merely reported.

**The broadcaster's own cause is carried through, not restated.**
`pendulum_state_broadcaster` separates six causes behind `valid == false` and
says which in its `status` string.  That string is copied into `message`
verbatim, because restating it would flatten a distinction the estimator went to
some trouble to make.

## The tracking error, and why it is two quantities

§5.0 is the reason this package exists at all.  The behaviour tree already
retries a stalled move and **its ownership of that decision is correct**; what
was wrong is the signal.  The goal tolerance is kept deliberately tight so the
controller aborts on goal time and the tree infers a stall from that abort, which
couples a tuning parameter to a recovery mechanism through a side effect.
`FAULT_TRACKING` on this stream is the typed cause that replaces the inference.
**The decision stays in the task layer; only the signal changes** — nothing here
stops, retries or re-plans.

The error is not re-derived from `/joint_states`.  The forked trajectory
controller computes it and publishes it per joint on its own
`~/controller_state`, and two different quantities come off that one message:

| On the wire | Where it comes from | What it is for |
|---|---|---|
| `tracking_error` | `error.positions`, max over the actuated joints of the absolute value | the field ROS 2 Interfaces §6 fixes, **rad or m** |
| `FAULT_TRACKING` | `error.velocities`, per axis against that axis's `dq_a` | the verdict, **rad/s or m/s** |

**Mixed units are handled, not hidden.**  Four axes are angles and two are
lengths, so the max is taken across two units: it is an *indicator* — "this is
the largest single-axis deviation anywhere on the crane" — and nothing is decided
from it.  Every decision is per axis against that axis's own number.

The verdict is on the velocity error and not on the position error because
`crane_control/config/tracking_tolerance.yaml` carries exactly one quantity,
`dq_a`, and it is a *velocity*-tracking tolerance: PRD §6 defines the gap the
seam clamp absorbs as the tracking error, and merge gate (ii-b) measures a
"per-axis velocity-tracking tolerance on a commanded ramp".  Comparing a position
error against it would be rad against rad/s.  The duty table's row 1 writes the
trigger as a position-error norm, so the architecture is not internally
consistent here; what is implemented is the dimensionally sound half of it, and
the fault message says outright which quantity each number is.

**There is no threshold, and none is invented.**  All six rows of
`tracking_tolerance.yaml` are `-1.0`, and the file says in its own header that a
value which is not finite and positive is not a tolerance.  No automated test in
this workspace can produce one — mock hardware mirrors commands into states, so
the tracking error there is identically zero — and the number comes from gate
(ii-b) or the slice 1b campaign, both human-only.  So an axis without one raises
no tracking fault: the node warns once at configuration, names the axes and names
who owns the missing number, and goes on publishing `tracking_error` as a
measurement.  A clear report in that state says so rather than implying a check
nobody made.  This is the same thing the velocity controller's seam clamp already
does when it goes transparent and warns.

**The number is read, never restated.**  The parameter is declared here with a
`NaN` default and given no value in this package's configuration; the values come
from `tracking_tolerance.yaml`, whose node key is the wildcard so that the seam
clamp, the MPC's constraint margin and this supervisor read the same six rows out
of the same file.  `test/test_no_tolerance_of_its_own.py` asserts that no copy
lives here, because a copy is how one number becomes three.

## What is not computed

`inside_working_cell` is `false` and is not computed in this slice: the virtual
working cell of §5.1 needs forward kinematics that `crane_model`'s production
backend does not have until slice 4, and an `inside_working_cell` computed from
nothing would be worse than one that is visibly not computed yet.

`deadman_held` and `tracking_error` are no longer among them.  Both are read off
an input and reported on every status, whatever `fault` says.

## Configuration

Four read-only parameters are shipped in `config/crane_supervisor.yaml`:
`pendulum_state_timeout`, `remote_ctrl_timeout`, `controller_state_timeout` and
`deadman_button`.  `joints` is declared with the six actuated joints of ROS 2
Interfaces §3.2 as its default and is not restated in the shipped file — a
deployment that wrote the list out again could only get it wrong.
`tracking_tolerance` is declared and deliberately left unset, for the reason
above.  All of it is in `src/crane_supervisor_parameters.yaml`.

Everything else about this node is a contract rather than a setting: the topic
and service names and the 20 Hz rate are constants in the source, so no
deployment can rename or re-rate a stream the task layer and the operator panel
subscribe to.

None of the three margins is a safety timing requirement.  By the time
`remote_ctrl_timeout` expires the hardware chain has long since acted; what the
number decides is how far the *diagnosis* lags the event, against how easily a
slow link raises a false one.

## Composition

The `fake` and `hardware` profiles of `crane_bringup` compose the node; `sim`
does not, exactly as it did not for the pendulum broadcaster.  It is a node
beside the controller manager, not a controller inside it: it claims no
interface, no spawner names it, and the launch gives it no remapping.

Two inputs have producers that do not already publish on the contract name.  The
retained `gpio_controller` publishes `RemoteCtrlStates` on its own private
`~/remote_ctrl_states`, and ROS 2 Interfaces §4 fixes the cross-node name as
`/crane/remote_ctrl_states`.  The trajectory controller likewise publishes
`JointTrajectoryControllerState` on its private `~/controller_state`, which
resolves under whatever the profile named that controller —
`trajectory_controller_a2b` today.  Lining either of them up is a remap in
`crane_bringup` and belongs to whoever composes the profile; this node subscribes
to the contract name and to nothing else, because §1 makes cross-node contracts
absolute and this package may not rely on a namespace.

**Two things are owed outside this package** and neither is in it: ROS 2
Interfaces §4 has no row for `/crane/controller_state` yet, and no profile
supplies the remap or passes `tracking_tolerance.yaml` to this node.  Until they
land the supervisor reports `FAULT_STATE_HEALTH` for a controller state that
never arrives, which is the honest reading of the situation and not a defect in
this package.

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
- four **inputs carried end to end** — `crane_msgs/PendulumState` on
  `/crane/pendulum_state`, `epsilon_crane_msgs/RemoteCtrlStates` on
  `/crane/remote_ctrl_states`, `control_msgs/JointTrajectoryControllerState`
  on `/crane/controller_state`, and `crane_msgs/VelocityControllerHealth` on
  `/crane/velocity_controller/health`.
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
exact four message types this node subscribes to rather than trusting a
substring.

Reading the inner loop's health cost the guard nothing.
`/crane/velocity_controller/health` is a **subscription** and stays one — a topic
named for a controller is exactly the shape a status node would grow a command
path in, so the guard names it among the five ROS names this package may hold and
pins it to the subscription list.

The status stream is worth having on its own terms: health and mode become
visible while the safety path is still being commissioned (PRD user story 64),
and the behaviour tree gets a **typed cause** to branch on instead of the
inferred abort §5.0 describes.

## What one report says

`fault` is a constant of `crane_msgs/SupervisorStatus` and `message` says why in
words an operator can act on.  Every report carries a cause (PRD user story 53) —
a status with `fault != FAULT_NONE` and an empty `message` is a test failure.

Eight of the ten constants are reported here, and two of the eight are not this
package's verdicts at all — `FAULT_REFERENCE_STALE` and `FAULT_NOT_COMMISSIONED`
are the inner velocity loop's, merged as it numbered them.  They are resolved in
this order:

| Report | When |
|---|---|
| `FAULT_ESTOP` | `em_stop` is asserted on `/crane/remote_ctrl_states`, **or** that stream is not arriving at all, **or** a stop that was one of those is latched and not yet acknowledged |
| `FAULT_STATE_HEALTH` | nothing has arrived on `/crane/pendulum_state` yet, or the stream stopped, or its stamp is further in this node's future than the margin, or `pendulum_state_broadcaster` marked the sample unusable |
| `FAULT_STATE_HEALTH` | the trajectory controller's own state has never arrived on `/crane/controller_state`, or it stopped, or its stamp is too far ahead — a controller that stopped publishing is not a crane that is tracking perfectly |
| `FAULT_STATE_HEALTH` | the inner velocity loop's own health has never arrived on `/crane/velocity_controller/health`, or it stopped, or its stamp is too far ahead — an uncommissioned axis reported to nobody is the state that stream exists to end |
| `FAULT_STATE_HEALTH`, `FAULT_REFERENCE_STALE` | the inner velocity loop raised one of its own **health** codes, carried through unedited |
| `FAULT_SWAY` | a passive joint **rate** is past its own configured bound, and the report names which of the two coordinates crossed it |
| `FAULT_TRACKING` | an actuated axis is outside **its own** velocity-tracking tolerance |
| `FAULT_NOT_COMMISSIONED` | the inner velocity loop raised the **commissioning** code — an axis has no identified valve map and ran PI only |
| `FAULT_INTERLOCK` | the remote is arriving, the stop is clear, and the configured deadman button is not held |
| `FAULT_NONE` | all four streams are arriving inside their margins, the broadcaster reports the sample usable, neither passive rate is past its sway bound, the inner loop reports nothing wrong with itself, no axis is outside its tolerance, nothing is latched and the deadman is held |

**The order is not arbitrary.**  The stop is first because it is the only one of
them with no field of its own: a cycle that reported something else instead
would not report it at all.  The passive state comes before tracking because a
supervisor whose own view of the crane is stale should say that before it says
anything derived.  The controller state's freshness is judged immediately before
the comparison that consumes it, since an error that is not arriving cannot be
compared to anything.  Sway sits below every health cause — a degraded estimate
cannot reach it at all, by construction — and above tracking, because of the two
it is the one with no field of its own: `tracking_error` is filled on every
report whatever `fault` says, so a tracking excess stays visible in a cycle sway
owns, while a swinging load reported behind a tracking fault would be invisible.
The **health** codes are reported in preference to the
**commissioning** code because [[commissioning_prerequisites]] §2 says so and not
because this package prefers it: a missing calibration will still be missing next
cycle, while a state that just went stale is the one an operator has to act on
now.  The commissioning code nonetheless sits *above* the interlock, because on
the `hardware` profile that condition is standing — prerequisite 4 holds until
someone records a calibration — while a released deadman is the ordinary resting
state of the machine, and putting the routine first would hide the report.  The
interlock is last because the deadman *does* have a field — `deadman_held` is
filled on every report whatever `fault` says — so putting a released button above
a defect would hide the defect behind a routine.  One consequence to know before
reading a panel: a supervisor started before `gpio_controller` sits in
`FAULT_ESTOP` until the remote arrives, which is what treating absence as
asserted means in practice.

`FAULT_NONE` from this supervisor means *nothing it watches is wrong*, not *the
machine is safe*, and the `message` on a clear report says so.  Working cell and
solver are later issues; `mode` is `MODE_IDLE` and nothing else until mode
arbitration exists, because a supervisor must never report a mode it has not
confirmed.

## Every input has a freshness deadline, and none is exempt

§5.3 names the failure mode this stack is most exposed to: not a wrong value, an
**absent** one.  No input may simply stop arriving without a defined
consequence, and this supervisor is the only element that sees every input that
reaches it by topic — so it is where the rule is enforced for those.

A rule enforced by four hand-written checks is a rule that lapses the first time
somebody adds a fifth subscription, so the policy is a shape rather than a
checklist:

- `Input` in `supervisor_core.hpp` is the **registry**.  `kInputCount` comes off
  the enum rather than sitting beside it.
- `kInputPolicies` has one row per enumerator — a `static_assert` refuses a table
  that is shorter, longer or out of order — carrying the topic, the type, the
  producer, what is lost while the input is missing, the `SupervisorStatus`
  constant its absence raises, and whether that fault latches.
- `SupervisorConfig::freshness_deadline` and `SupervisorInput::streams` are both
  sized by `kInputCount`, so a new input gets a slot in each for free.
- **There is no default deadline.**  The array is value-initialised to zero,
  `validate()` refuses a zero, and the node throws out of its constructor.  An
  input added with no margin is a node that refuses to start, never an input
  nobody watches.
- Every subscription is created by one helper that takes an `Input`, so one that
  named no input — and therefore had no deadline — cannot be written; the topic
  comes off the policy row rather than off the call site; and the constructor
  refuses to finish while any enumerator is left unsubscribed.
- `test/test_every_input_has_a_deadline.py` asserts the shape statically:
  `create_subscription` appears exactly once in the package, every enumerator is
  subscribed *and* has a row *and* is assigned a deadline out of a parameter, in
  that order, and no contract name is written down twice.

The deadlines are **per input**, not one number: `/crane/pendulum_state` comes
off the manager's 100 Hz cycle and `/crane/remote_ctrl_states` is a 20 Hz
contract, so an age that is healthy on one is a dead publisher on the other.
The numbers and the derivation of each are in
`src/crane_supervisor_parameters.yaml`, once.

The sweep is `kInputCount` comparisons on the **status timer**, and that is the
point rather than an optimisation: a deadline evaluated in a subscription
callback could never fire, because the case it exists for is the one where no
callback runs again.

### Which of the three causes fired

§5.3's "Stale state" now has three causes, and they are different in kind: the
producer's **health flag**, the measurement **age**, and a sample that stopped
**refreshing** behind a header that keeps moving.  What this supervisor can see
of them is not symmetric, and the report says which fired:

| Cause | Where it is measured | How it reads here |
|---|---|---|
| age | here, from `header.stamp` against this input's deadline | `never connected`, `stopped arriving`, or `stamped ahead of this clock` — three reports, because "the publisher never came up" and "the publisher died" are different things to chase |
| health flag | the producer | `valid == false`, and the producer's own `status` string carried through unedited |
| refresh | the producer | inside that same string.  There is **no topic-level version of it here on purpose**: `pendulum_state_broadcaster` can ask whether seven doubles moved because differenced-gyro noise is thirty times the quantiser, while a supervisor asking the same of `RemoteCtrlStates` would be asking whether twelve booleans moved, and an operator holding a button produces bit-identical payloads for minutes |

**Recovery is symmetric.**  An input that starts arriving again inside its
deadline clears its own fault with no acknowledgement.  The emergency stop is
the single exception, and `InputPolicy::latches` is where that asymmetry is
written down rather than implied.

**What this does not cover, and says so.**  §6.3's controller-side timeouts are
still ad hoc — the manual forwarding controller zeroes 0.5 s after its own input
stops and the proportional controller never times out at all — and none of that
is this supervisor's to fix in this slice.  The clear report names the gap
instead of leaving `FAULT_NONE` to imply it was closed.

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
faults rather than a quiet `FAULT_NONE` — and the stop signal's absence is read
as asserted rather than merely reported.  How that is enforced for every input
rather than for the ones somebody remembered is the section above.

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

## The sway, as a bound and as a predicate

§5 row 7 gives the supervisor one narrow duty about the sway: **refuse to start a
motion that depends on sway being settled, and report.**  It does not damp — that
is slice 6 — and it does not stop a motion already running for being swingy.  The
*refusal* half needs an authority over a mode that this package will not have
until `/crane/set_mode` exists, so what is here is the report and the signal
somebody else refuses on.

Two different things come off the passive rate and they are not interchangeable:

| | What it is | Where it goes |
|---|---|---|
| the bound | a configured rate per passive coordinate, crossed or not | `FAULT_SWAY`, naming which of `theta6_tip_joint` and `theta7_tilt_joint` crossed it, with the rate and the bound |
| the predicate | **three-valued** — settled, not settled, unknown — held over a dwell and released through a hysteresis | the clause every report ends in; see the gap below |

**Both are on the rate and neither is on the angle.**  Two independent reasons,
either of which would be enough on its own.  `pendulum_state_broadcaster` reads
the two passive coordinates out on the *nominal* hinge axes, because the
calibrated 2-D spline for the real double hinge needs calibration data this
workspace does not carry — so the published angle carries an uncalibrated
constant offset, and `theta7_tilt_joint`'s limits are not centred on zero either.
The rate is a composition of the two gyro readings through the known chain rather
than a derivative of the angle, so none of that reaches it.

**`velocity_covariance` is not consulted, deliberately.**  The block is the
identified noise of the differenced gyro pair — a property of the sensors,
constant while the estimate is trusted and withdrawn to `-1` where it was never
identified for the hardware.  Weighting a bound with it would be a
confidence-weighted test whose confidence never moves.  What the number *is* good
for is fixing the floor a design threshold has to clear, and that use is in
`src/crane_supervisor_parameters.yaml` as a derivation rather than as a runtime
read.  `test_sway_monitor.cpp` asserts the shipped settle bound clears it.

### Three states, and the third one is the point

An estimate that is absent, stale or marked unusable makes "settled"
**unknowable**, and unknowable must not read as settled: a grip action gated on a
two-valued predicate would descend onto a swinging block the moment the
bracketing IMU stopped answering.  So the predicate has three values, and a
degraded estimate produces `Unknown` *and* `FAULT_STATE_HEALTH` — never
`FAULT_SWAY`.  A sensor that stopped saying anything is a different fact from a
load that is swinging, and an operator does different things about them.

The predicate is decided **before** the precedence chain runs, the way
`deadman_held` and `tracking_error` are filled before any branch returns.  A
dwell that only advanced on the cycles where sway was what went wrong would
restart every time the operator let go of the deadman.

### It does not chatter, and that costs two mechanisms

- a **dwell** of 2.0 s on the way in — 40 consecutive status cycles.  It is not a
  smoothing constant: a pendulum's rate passes through zero twice a period, so a
  window shorter than the 1.57 s half period can sit on a turning point and see a
  swing at its slowest.
- a **hysteresis** on the way out — the release bound is 1.5× the settle bound, so
  one noise sample past the bound does not cost a whole dwell at 20 Hz.

### The gap: the predicate has no field of its own

`crane_msgs/SupervisorStatus` carries `mode`, `fault`, `tracking_error`,
`inside_working_cell`, `deadman_held` and `message`, and none of them is a
three-valued sway predicate.  **`crane_msgs` is frozen**, and PRD §15's amendment
rule makes a *field add* a slice of its own that has to name every consumer;
`crane_msgs` is also outside this issue's scope, and no message package already in
this node's dependency whitelist carries a stamped tri-state.  So what the
predicate rides on today is the `message` string, in a clause every report ends
in and whose prefix is a constant (`kSettledClausePrefix`) rather than a literal
somebody greps for.

The `fault` field is **not** an alternative and was rejected rather than
overlooked: it carries one cause per cycle in a fixed precedence, so a cycle
reporting `FAULT_ESTOP` says nothing about the sway, and "no `FAULT_SWAY`" would
read as "settled" — which is exactly the two-valued defect the three states exist
to prevent.

**What closes it** is one additive amendment, which is *not* a field add and
therefore not a slice: a new `crane_msgs/SwaySettled` message
(`std_msgs/Header header`, `uint8 SETTLED_UNKNOWN=0`/`SETTLED_NO=1`/`SETTLED_YES=2`,
`uint8 settled`, `float64[2] velocity`, `string message`) published on
`/crane/sway_settled` at the status rate, with the ROS 2 Interfaces §4 row in the
same commit.  The core is already shaped for it: `SwaySettled` is the enum,
`SupervisorDecision::sway` is the value, and the adapter change is a second
publisher and four assignments.  It needs the `crane_msgs` repo, which this
issue's `repos:` does not carry.

## The inner loop's fault, carried and not re-derived

`crane_velocity_controller` computes a `SupervisorStatus` fault code on every one
of its 100 Hz cycles.  Until `/crane/velocity_controller/health` existed the only
way to read it was `CraneVelocityController::fault()`, an accessor whose own
documentation says it is there for the S5 harness and is never on the control
path — so the fault stopped at the controller manager and reached nobody.  The
concrete consequence: prerequisite 4, the uncalibrated PZS100 gripper axis, is
reported as `FAULT_NOT_COMMISSIONED` on the `hardware` profile, and that report
was going nowhere.

**This supervisor could not have re-derived it, and did not try.**  Which axes
the active tool has an identified valve map for is in neither `/joint_states` nor
the trajectory controller's state; the controller is the only element that knows.
So the fault stays where it is computed and travels as a message, and this package
merges a code rather than translating one:

| On the wire | What this package does with it |
|---|---|
| `fault` | a `crane_msgs/SupervisorStatus` constant, unrenumbered, reported at the place in the order above that [[commissioning_prerequisites]] §2 fixes |
| `joint_names` + `feedforward_applied` | the axes that ran PI only, **named** in `message` — a panel that says `q9_left_rail_joint` tells an operator which calibration to run, and one that says "one axis" does not |

A code the loop is not supposed to be able to raise is carried through unedited
rather than folded into one of the three, because inventing a cause here would
hide the drift between `crane_velocity_controller`'s `static_assert` block and the
frozen message.

**Whether a missing prerequisite is reported at all is not decided here.**
[[commissioning_prerequisites]] §3 puts that on the `hardware` profile only, and
the switch is `crane_velocity_controller`'s own `profile` parameter.  This package
holds no profile and no rig name: the `fake` rig's report and the machine's are two
different messages on the wire, and nothing on the path from the loop to
`/crane/supervisor/status` looks at anything else.  The per-axis flags are not a
second switch either — an axis can run PI only on `fake` with no fault raised, and
this supervisor still reports `FAULT_NONE`.

Staleness of this stream is judged by the same policy as every other input's:
"no message yet" must not read as a healthy, commissioned inner loop, so a
report that never arrived, one that stopped and one whose stamp cannot be placed
in time are all `FAULT_STATE_HEALTH` with their own account of which they are.

## What is not computed

`inside_working_cell` is `false` and is not computed in this slice: the virtual
working cell of §5.1 needs forward kinematics that `crane_model`'s production
backend does not have until slice 4, and an `inside_working_cell` computed from
nothing would be worse than one that is visibly not computed yet.

`deadman_held` and `tracking_error` are no longer among them.  Both are read off
an input and reported on every status, whatever `fault` says.

## Configuration

Nine read-only parameters are shipped in `config/crane_supervisor.yaml`:
`pendulum_state_timeout`, `remote_ctrl_timeout`, `controller_state_timeout`,
`controller_health_timeout`, the four under `sway`, and
`deadman_button`.  The first four are the freshness deadlines of the four
`Input`s, one each, and they are the only place those numbers exist — the core's
array has no default, so a deadline that is not configured is a node that does
not start.  Their names are the ones `crane_bringup` already passes; they read
`timeout` where the core reads `deadline`, and the two mean the same thing.

`sway.dq_u_max`, `sway.dq_u_settled`, `sway.settled_release_factor` and
`sway.settle_dwell` are the sway duty's bounds, and **every one of them is a
design value rather than a measured one**.  The file says so beside each number,
gives the derivation, and names what would replace it — the same discipline
`tracking_tolerance.yaml` and the broadcaster's `health.max_measurement_age`
already follow.  Unlike the tracking tolerance they are *refused* when absent
rather than reported: they ship with the package, so a deployment without one has
been misconfigured rather than left waiting on a human campaign, and a supervisor
that quietly stopped judging the sway would report `FAULT_NONE` for a duty it was
not performing.
`joints` is declared with the six actuated joints of ROS 2
Interfaces §3.2 as its default and is not restated in the shipped file — a
deployment that wrote the list out again could only get it wrong.
`tracking_tolerance` is declared and deliberately left unset, for the reason
above.  All of it is in `src/crane_supervisor_parameters.yaml`.

Everything else about this node is a contract rather than a setting: the topic
and service names and the 20 Hz rate are constants in the source, so no
deployment can rename or re-rate a stream the task layer and the operator panel
subscribe to.

None of the four margins is a safety timing requirement.  By the time
`remote_ctrl_timeout` expires the hardware chain has long since acted; what the
number decides is how far the *diagnosis* lags the event, against how easily a
slow link raises a false one.

## Composition

The `fake` and `hardware` profiles of `crane_bringup` compose the node; `sim`
does not, exactly as it did not for the pendulum broadcaster.  It is a node
beside the controller manager, not a controller inside it: it claims no
interface, no spawner names it, and the launch gives it no remapping.

Two of the four inputs have producers that do not already publish on the contract
name.  The retained `gpio_controller` publishes `RemoteCtrlStates` on its own
private `~/remote_ctrl_states`, and ROS 2 Interfaces §4 fixes the cross-node name
as `/crane/remote_ctrl_states`.  The trajectory controller likewise publishes
`JointTrajectoryControllerState` on its private `~/controller_state`, which
resolves under whatever the profile named that controller —
`trajectory_controller_a2b` today.  Lining either of them up is a remap in
`crane_bringup` and belongs to whoever composes the profile; this node subscribes
to the contract name and to nothing else, because §1 makes cross-node contracts
absolute and this package may not rely on a namespace.

`/crane/velocity_controller/health` needs no remap: `crane_velocity_controller`
creates the publisher with that absolute name itself, and ROS 2 Interfaces §4
carries the row.

**Two things are owed outside this package** and neither is in it: ROS 2
Interfaces §4 has no row for `/crane/controller_state` yet, and no profile
supplies the remap or passes `tracking_tolerance.yaml` to this node.  Until they
land the supervisor reports `FAULT_STATE_HEALTH` for a controller state that
never arrives, which is the honest reading of the situation and not a defect in
this package.

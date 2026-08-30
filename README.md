# crane_supervisor

The supervisor of [[control_architecture]] §5, in the state PRD §2 slice 3 leaves
it: **status and mode only**.  It watches, it reports, and the one thing it
decides is which controller holds the claim.

- a **ROS-free decision core** — `include/crane_supervisor/supervisor_core.hpp`.
  An input struct in, a decision struct out.  No node handle, no message type, no
  clock, no topic, so every cause is reachable from a unit test with no ROS
  runtime in the process.
- a **node** — `crane_supervisor`, which owns the I/O and the timing and nothing
  else.  It publishes `crane_msgs/SupervisorStatus` on `/crane/supervisor/status`
  at 20 Hz, reliable, depth 1, the stream ROS 2 Interfaces §2 and §4 fix and
  `crane_msgs`' own ROS contract test already asserts.
- a **second status stream** — `crane_msgs/SwaySettled` on `/crane/sway_settled`,
  same rate and same QoS, carrying the three-valued settled predicate as a field
  a behaviour tree branches on.  It is a stream of its own because
  `SupervisorStatus` is frozen and widening it is a slice of its own (PRD §15);
  it is published from the same cycle, off the same decision, with the same
  `header.stamp`.
- four **inputs carried end to end** — the passive pair as
  `sensor_msgs/JointState` on `/joint_states`, `epsilon_crane_msgs/RemoteCtrlStates` on
  `/crane/remote_ctrl_states`, `control_msgs/JointTrajectoryControllerState`
  on `/crane/controller_state`, and `crane_msgs/VelocityControllerHealth` on
  `/crane/velocity_controller/health`.
- two **services** — `/crane/clear_fault` (`std_srvs/Trigger`), which
  acknowledges a latched emergency stop and does nothing else, and
  `/crane/set_mode` (`crane_msgs/SetMode`), which is the **only** way a mode
  changes (ROS 2 Interfaces §5).
- a **fifth stream, watched but not swept** — `crane_msgs/SolverHealth` on
  `/crane/mpc/solver_health`.  It is deliberately not one of the four `Input`s:
  its *absence* raises no fault on the status stream, because an optimizer that
  is quiet while the machine is in `MODE_FOLLOW` is the ordinary state of this
  stack.  Its *content* decides three things: whether `MODE_MPC` may be entered
  (PRD §10 step 2), the `FAULT_SOLVER` this package merges while `MODE_MPC` is
  live (ROS 2 Interfaces §4), and whether [[mpc]] §6's repeated-failure
  escalation has taken the command path away.
- two **clients on the controller manager** — `list_controllers`, which is where
  the reported mode comes from, and `switch_controller`, which is the one call
  this package makes that reaches the machine.  ROS 2 Interfaces §5 puts that
  client here and nowhere else: mode changes go **through the supervisor**, not
  from the behaviour tree.
- one **client on the horizon producer** — `rcl_interfaces/SetParameters` on the
  node `mpc_node` names, carrying one parameter: `crane_mpc`'s `mode`.  ROS 2
  Interfaces §4 makes which of the two producers is live the supervisor's
  decision *alone*, issue 053 made that a state of `crane_mpc`, and this is the
  call that moves it.

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
topic and nothing that writes a setpoint** — and `test/test_no_command_path.py`
asserts the absence, because an absence nobody checks is one that comes back by
accident.  It is withheld, not unfinished.

**A mode change is not a stop, and the difference is the whole of what is still
withheld.**  §5.2 gives the stop three steps: deactivate the claim, *zero at the
driver boundary regardless*, and ramp rather than step.  The second is the one
that makes it a path to zero, it belongs to the driver, and it is blocked on
prerequisite row 3 — so nothing here zeroes anything, and this package still
holds no publisher on a command topic.

The first step happens in exactly **one** place that is not a request, and it
arrived with issue 055: [[mpc]] §6's repeated-failure escalation, where the
horizon producer has stopped publishing and `crane_velocity_controller` has run
out of plan.  That is [[control_architecture]] §5 row 3's "fall back, *then* stop
and report", and the section
*[The escalation, and the stop that follows the fall
back](#the-escalation-and-the-stop-that-follows-the-fall-back)*
is where the four conditions are written down.  It releases to `MODE_IDLE`, it
goes through the same `arbitrate_mode()` an operator's request goes through, and
it is still not §5.2's stop: the machine reaches zero because the *receiver's*
`horizon_expiry_ramp` took it there before the claim moved, not because anything
in this package commanded it.

`MODE_IDLE` is the closest the two come, and the report on that switch says
outright that releasing the arm claim is not a way to bring the machine to rest:
nothing is zeroed at the driver boundary, and §7.2 measured that a manual
controller which deactivates leaves its last velocity latched on the interface it
just released.

The guard was **updated rather than relaxed** each time this package grew a
reach, and each time what replaced a blanket ban was narrower.

`create_service` came off the banned list when `/crane/clear_fault` landed; what
replaced it names the services this package may serve and the types it may serve
them with.  `std_srvs/Trigger` carries no request fields at all, and
`crane_msgs/SetMode` carries one `uint8` — no setpoint, no joint, no duration —
so neither is a shape a caller can smuggle a motion command through.

`create_client`, `controller_manager`, `SwitchController` and `switch_controller`
came off it in issue 026, for the same kind of reason: they were banned because
the arbitration was a later issue, and that is this issue.  ROS 2 Interfaces §5
makes the supervisor the only element that may call
`/controller_manager/switch_controller`, so the ban had to lift somewhere for the
stack to have an arbiter at all — and the safest place is the one package whose
every other reach is asserted.  What replaced it pins the two service *types* by
name and the two service *names* as literals, so a third client cannot appear and
neither of the two can be pointed anywhere else.  What stays banned is everything
that would let this package *hold* a piece of the control loop rather than call
the manager over the graph: `controller_interface` and `hardware_interface` are
how a package becomes a controller, `command_interface` is how it writes one, and
`load_controller`, `unload_controller`, `configure_controller` and
`set_hardware_component_state` are the manager calls that go beyond arbitrating a
claim.  `controller_manager` itself stays a **test** dependency: the harness
builds a manager inside the test process, and linking one into the deployed node
would be a different reach from calling it.

Reading the trajectory controller's state cost the guard one more relaxation, of
the same shape.  `JointTrajectory` was on the banned-identifier list as a
stand-in for `trajectory_msgs`, the package a *commanded* trajectory is typed
with — and it also matched `control_msgs/JointTrajectoryControllerState`, which
is a controller describing itself.  The ban is now on `trajectory_msgs` by name,
so a publisher of the command type still cannot appear, and the guard pins the
exact four message types this node subscribes to rather than trusting a
substring.

Putting the settled predicate on the wire cost the guard its **publisher count**,
and that is the one number in it that was doing the most work — a second
publisher is how a status node becomes a command node, since the topic would be
new, the QoS would be new and nothing else about the package would look
different.  So it was relaxed the way the other three were: by *naming the second
stream*, by type and in source order, rather than by counting differently.  Three
things carry the argument.  `crane_msgs/SwaySettled` is a **report** — a `uint8`
verdict, the two rates it was decided from and a sentence, with no joint, no
duration and no setpoint, so it is not a shape a motion command fits in.  Its
consumer is the **task layer** and not a controller: nothing in this stack
subscribes to it inside the real-time cycle, and the refusal §5 row 7 asks for
happens at the goal, which this package still has no authority over.  And the
ban that would actually stop a command path is untouched — `trajectory_msgs` is
still banned outright, so a publisher of the *command* type cannot appear
whatever the topic were called.  What did not change is everything else: the four
subscriptions, the two services, the two clients, and the set of ROS names this
package may know, which gained exactly one entry.

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

Nine of the ten constants are reported here, and three of the nine are not this
package's verdicts at all — `FAULT_REFERENCE_STALE` and `FAULT_NOT_COMMISSIONED`
are the inner velocity loop's and `FAULT_SOLVER` is the horizon producer's, each
merged as its producer numbered it.  They are resolved in this order:

| Report | When |
|---|---|
| `FAULT_ESTOP` | `em_stop` is asserted on `/crane/remote_ctrl_states`, **or** that stream is not arriving at all, **or** a stop that was one of those is latched and not yet acknowledged |
| `FAULT_STATE_HEALTH` | nothing naming the passive pair has arrived on `/joint_states` yet, or that half stopped, or its stamp is further in this node's future than the margin |
| `FAULT_STATE_HEALTH` | the trajectory controller's own state has never arrived on `/crane/controller_state`, or it stopped, or its stamp is too far ahead — a controller that stopped publishing is not a crane that is tracking perfectly |
| `FAULT_STATE_HEALTH` | the inner velocity loop's own health has never arrived on `/crane/velocity_controller/health`, or it stopped, or its stamp is too far ahead — an uncommissioned axis reported to nobody is the state that stream exists to end |
| `FAULT_SOLVER` | `MODE_MPC` is the live mode and the horizon producer's newest report — inside `horizon_timeout` — carries a fault, carried through unedited |
| `FAULT_STATE_HEALTH`, `FAULT_REFERENCE_STALE` | the inner velocity loop raised one of its own **health** codes, carried through unedited |
| `FAULT_SWAY` | a passive joint **rate** is past its own configured bound, and the report names which of the two coordinates crossed it |
| `FAULT_TRACKING` | an actuated axis is outside **its own** velocity-tracking tolerance |
| `FAULT_NOT_COMMISSIONED` | the inner velocity loop raised the **commissioning** code — an axis has no identified valve map and ran PI only |
| `FAULT_INTERLOCK` | the remote is arriving, the stop is clear, and the configured deadman button is not held |
| `FAULT_NONE` | all four streams are arriving inside their margins, the broadcaster reports the sample usable, the horizon producer is not faulting where it drives, neither passive rate is past its sway bound, the inner loop reports nothing wrong with itself, no axis is outside its tolerance, nothing is latched and the deadman is held |

**The order is not arbitrary.**  The stop is first because it is the only one of
them with no field of its own: a cycle that reported something else instead
would not report it at all.  The passive state comes before tracking because a
supervisor whose own view of the crane is stale should say that before it says
anything derived.  The controller state's freshness is judged immediately before
the comparison that consumes it, since an error that is not arriving cannot be
compared to anything.  The solver sits below all of those and *above* the inner
loop's own codes, because when the optimizer stops the receiver runs out of plan
and raises `FAULT_REFERENCE_STALE` — the consequence of this cause, one layer
down — and §5.0 is explicit that the task layer is owed the typed cause rather
than a symptom it has to infer.  Sway sits below every health cause — a degraded estimate
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
machine is safe*, and the `message` on a clear report says so.  The working cell
is the one constant of the ten that stays unreachable — it needs the virtual
envelope of §5.1, which nothing in this stack computes.  `mode` is the section
below.

## The mode, and the one authority this supervisor has

`/crane/set_mode` is the only way a mode changes, and this node is the only
element in the stack that calls `/controller_manager/switch_controller`
(ROS 2 Interfaces §5).

**The exclusion already existed; what is added is the decision in front of it.**
§7: manual and autonomous both claim the same `velocity` command interfaces, so
ros2_control refuses to activate one while the other holds it, and §7 calls that
a sound foundation to keep.  This supervisor keeps it and makes the exclusion an
*arbitrated* decision with a stated reason, instead of an activation failure
somebody has to read a log to understand.

### What is reported is what is active

`mode` on the status stream is re-derived from the controller manager on every
cycle, never remembered from the last successful request.  A mode is active when
the controllers of the arm claim that are up are **exactly** the ones configured
for it; when none is up, the arm claim is free and that is `MODE_IDLE` as an
observation rather than as a default.

**Equality and not containment, and slice 6 is why.**  Until `MODE_MPC` was
populated the rule was "every controller configured for it is active" and the
lists were required to be pairwise disjoint, which made that rule sound.  PRD
§10 step 3 puts the *same* `crane_velocity_controller` instance on both paths —
"same controller instance across both paths, so the handover is an ordinary
seam, not new machinery" — so `MODE_MPC`'s list is `MODE_FOLLOW`'s minus the
trajectory controller, and a rule requiring the lists to be pairwise disjoint
would have outlawed the architecture.  Under containment the nesting is worse
than untidy: `MODE_FOLLOW`
holding the claim would satisfy `MODE_MPC` as well, and `MODE_MPC` holding it
would leave `MODE_FOLLOW` half up and read as drift.  Under equality both are
answered exactly, and what `validate()` has to insist on is only that no two
modes name the *same set* — two identical claims under two names would both
match and the answer would depend on which was checked first.

Three things follow, and each is a case a remembered mode would have hidden:

| What the manager says | What goes on the wire |
|---|---|
| it is not answering, or its newest answer is older than `controller_manager_timeout` | `MODE_IDLE`, and the clause says the mode is **not known** and why.  No mode change is admitted while this holds |
| some of a mode's controllers are up and not all | `MODE_IDLE`, and the clause names the ones that are up.  A half-state is not a mode, and rounding it into one would be a mode nobody can act on |
| a controller is active, owns command interfaces and is in no configured mode and no tool claim | reported by name in the clause.  It is a claim this configuration does not describe, and it can refuse a switch by holding an interface the incoming controller needs |

The clause is on **every** report, whatever the fault is: `MODE_IDLE` on a report
whose manager is silent and `MODE_IDLE` on one whose arm claim is free are the
same byte and different facts, and only the words tell them apart.  The mode is
polled and not subscribed, so it is deliberately not one of the four `Input`s —
`ControllerManagerReport` in `supervisor_core.hpp` says how §5.3's rule is met for
something whose failure shows at the call site.

### What a request is answered with

Every precondition is checked **before** anything is deactivated, and the plan is
a single activate/deactivate pair for one strict `switch_controller` call — so a
switch that cannot succeed is refused from the mode the machine is already in and
is never discovered half-way (PRD §10 step 2, user story 35).  In order:

| Refused when | Because |
|---|---|
| the request is not one of the four constants | a `uint8` is not a mode until it has been checked, and answering 200 with a plausible mode is worse than refusing it |
| the mode is `MODE_MPC` and the horizon is not fresh | PRD §10 step 2 and user story 35, and **it is checked first on purpose** — see below.  The refusal names freshness, how old the producer's newest report is, what the deadline was and what its last solve said |
| the controller manager has not answered inside its deadline | a precondition cannot be checked against a view this supervisor does not have, and a switch issued blind is the half-way discovery §10 forbids |
| a fault is latched **and** the mode is a motion mode | the latched cause is the reason, carried rather than restated, and clearing it goes through `/crane/clear_fault`.  `MODE_IDLE` is still reachable: releasing the claim is the one direction a latched stop does not argue against |
| the deployment configures no controller for the mode | there is nothing to activate, and an empty switch reported as a success is a no-op an operator cannot see.  `MODE_MANUAL` is in this state on today's profiles |
| a controller the mode needs is not loaded on the manager | it could never have been activated, and the machine keeps the claim it holds |
| a controller the mode needs is loaded and not `inactive` or `active` | that is not a state a switch can move, and a controller that will not configure is a failure to chase on the manager rather than in this request |

Every one of them sets `success=false`, says which precondition fired, and ends
in *Nothing was deactivated* (ROS 2 Interfaces §1, PRD user story 36).
`active_mode` on the response is **what is actually active after the call** —
read back off the manager afterwards, never assumed — so a refusal reports the
mode the machine was already in, and a switch the manager accepted but that
landed somewhere else is reported as where it landed, with a clause saying so.
A request for the mode that is already active is a no-op and is reported as one
rather than as a switch.

### The handover, and why step 2 is first

PRD §10 sequences the change into and out of the MPC path in four steps.  Two of
them are this package's, and the order between them is the whole content:

**Step 2 — freshness, before the controller manager is asked for anything.**
Not before the switch: before the *poll*.  `mpc_horizon_refusal()` is evaluated
on the observation this node already holds, and only if it comes back empty does
`/crane/set_mode` go on to read the machine.  So a switch into a dead MPC costs
the manager nothing and is refused from the mode the machine is already in
(user story 35).  `arbitrate_mode()` runs the same function again at precondition
2, off the same observation, so the early answer and the arbitration cannot
disagree about what was verified.

The evidence is `/crane/mpc/solver_health` and **not** `/crane/mpc/horizon`, and
the substitution is forced rather than chosen: in shadow — the state every switch
into `MODE_MPC` is made *from* — `crane_mpc` publishes nothing at all on the
contract topic, so a check against the horizon could never pass and `MODE_MPC`
would be unreachable by construction.  `solver_health` runs in both modes and
carries the solve's own three-valued verdict, which is exactly what step 1 means
by "the MPC runs warm before the switch; shadow mode already implies it solves".

Two things have to hold and the second is not implied by the first.  The report
has to be **fresh**, against `horizon_timeout`; and the newest solve has to have
**converged**.  A producer that is alive, publishing at rate and failing every
solve reads as fresh and is precisely the dead MPC step 2 exists to refuse.
`SOLVE_BUDGET_EXCEEDED` is refused with `SOLVE_FAILED`, deliberately: §5 row 3
gives `FAULT_SOLVER` to a deadline miss and to non-convergence alike, and
entering the mode on a plan with nothing behind it costs a handover while
refusing costs one more request.

That stream is deliberately **not** an `Input`.  An optimizer that is quiet while
the machine is in `MODE_FOLLOW` is the ordinary state of this stack rather than a
defect, so its *absence* raises nothing.  §5.3's rule is met the way
`ControllerManagerReport` meets it — its own configured margin, refused by
`validate()` when it is missing, and a defined, narrow consequence stated where
it matters: no freshness, no switch.

Its `fault` is merged onto `/crane/supervisor/status` all the same, and the two
are different questions — see *[The optimizer's verdict, and the one mode this
supervisor leaves on its own](#the-optimizers-verdict-and-the-one-mode-this-supervisor-leaves-on-its-own)*
below.

**The producer's own mode, and the order is not symmetric.**  ROS 2 Interfaces
§4's "One command path" ends by saying that which of the two producers is live is
the supervisor's decision *alone* and that the two never drive at once.  Issue
053 made that a state of `crane_mpc` and left its `mode` parameter writable for
exactly this, so a switch moves it as well as moving the claim:

| Direction | Order | Why |
|---|---|---|
| into `MODE_MPC` | producer to `active` **before** the claim moves | the MPC then publishes while the trajectory controller is still chained, which cannot steal the command, and the velocity controller has a fitted spline in hand at the instant the follower lets go |
| out of `MODE_MPC` | producer to `shadow` **after** the claim has moved | the same argument read the other way |

The producer is therefore active across the union of the two intervals rather
than across either exactly.  What settles it afterwards is the mode **read back**
off the manager rather than the one that was asked for, so a switch that failed
in either direction leaves the producer where the claim actually is — and a
producer that refused, timed out or was not there is *named on the response*
rather than left for a silent horizon to be the evidence of.

**Steps 3 and 4 are not here and are not this package's at all.**  The seam clamp
and the setpoint the re-entering trajectory is anchored on live in
`crane_control`, where the command is, and the reason they need no supervisor is
step 3's own: the same controller instance runs both paths.  That is also why the
plan never *names* `crane_velocity_controller` in either direction — a plan that
did would be asking for the sole claimant of the six `velocity` command
interfaces to be released and re-claimed, which is a different thing from the
chained-mode transition the manager performs on it internally and one it is not
written to survive.

**The bound the clamp acts with does not exist.**  All six values in
`crane_control/config/tracking_tolerance.yaml` are negative and that file's own
header says why: the number comes from merge gate (ii-b) or the slice 1b
campaign, both human-only.  So the clamp is *transparent* on the deployed
configuration, the report this node composes about a missing tolerance says so in
as many words rather than implying a bound nothing enforces, and the mechanism is
tested against an injected number in `crane_control/test/test_mode_handover.cpp`.

### Two claims, not one machine

§7.3: with the block gripper the tool axis is driven by its **own** controller
rather than by the arm's, so it can move while the arm controller is inactive — a
second, independent claim on the same pump, and *the supervisor, not the
controller layer, is what keeps a gripper action from being issued during an arm
motion*.  A mode model that mapped one machine onto one claim would be wrong on
the machine this stack runs on, and wrong silently: it would deactivate a tool
controller it never meant to touch.

So the configuration carries the arm claim's controllers **per mode** and the
tool claim's **separately**, a mode switch plans over the arm claim alone, and
the tool claim is reported on every answer and never switched by a mode request.
`validate()` refuses a controller that is in both, because an arm mode change
would then take down the gripper it is supposed to leave alone.

**Slice 3 cannot see the tool claim on the deployed profiles, and the shipped
configuration says so rather than modelling it away.**
`crane_velocity_controller` claims all six `velocity` interfaces including
`q9_left_rail_joint`, so the CBS stack has one claim today and `tool_controllers`
ships empty; the clause on every report says that outright instead of staying
quiet, because a report that mentioned the tool only when it was held would be a
one-claim model with an exception in it.  `test/test_mode_switch.cpp` is where the
second claim exists: it composes a tool controller on the gripper axis and
asserts that activating, changing and releasing an arm mode leaves it exactly
where it was.

### Where it is tested, and where it is not

Against a real `controller_manager` **inside the test process**, with mock
hardware and the control loop driven by hand, on the isolated domain
`ralph/verify.sh` pins — the S5 shape `crane_control` already uses.  A switch is
the one call that must never be issued against the machine by accident, and this
workspace's sim runs join the real crane's DDS graph, so it is never asserted
against a launch.  The arbitration itself is decided in the ROS-free core, so
every refusal is reachable from a unit test with no runtime at all.

**A real manager keeps no record of who called it**, and two of the things slice
6 has to assert are about exactly that.  "The switch was never issued" and "the
switch was issued and the manager refused it" are indistinguishable from the
outcome alone, and so are "the horizon was checked first" and "the manager was
consulted first and happened to answer".  So `test/test_handover_sequence.cpp`
runs the same node against a controller manager that can be **watched** — a spy
that answers `list_controllers` from a scripted table and counts every
`switch_controller` call — and a `crane_mpc` stub that records what the claim was
doing at the instant its mode changed.  It is an instrument rather than a
stand-in: there is no control loop and no hardware in it, and what it exists to
observe is which calls this node makes and in what order.  One of its tests takes
the manager off the graph entirely, which is what turns "freshness is checked
first" into an observation: a supervisor that consulted the manager first would
have exactly one thing to say, and the refusal that comes back names the horizon.

The same spy is what makes the escalation's *timing* assertable.  "The claim was
not released yet" is a switch that must **not** have happened, which no outcome
can show, so `TheRepeatedFailureEscalationTakesTheModeOutOfMpc` drives the
escalation with the receiver still executing, asserts the call count did not
move, then reports the reference stale and asserts it moved by exactly one.  That
last "exactly one" is also how the once-per-episode rule is checked: it keeps
publishing the escalation for another half second and asserts the count is still
one.

**And the two streams, not only the two responses.**  A `/crane/set_mode`
response says what one caller was told; `/crane/supervisor/status` is what the
task layer branches on.  `TheStatusStreamCarriesModeMpcAndTheProducersOwnCode`
drives all four inputs healthy — which the older tests in that file do not need
to, and which `FAULT_SOLVER` does need, since it sits below every cause that
removes part of this supervisor's own view — and reads the mode and the fault off
**one** report, because a stream that showed `MODE_MPC` on one cycle and
`FAULT_SOLVER` on another would satisfy two separate waits without either having
been true at once.

**One cost is known and is not fixed here.**  `/crane/set_mode` and the status
timer share this node's default callback group, so while a switch is in flight
the 20 Hz stream does not tick and no subscription callback runs.  On a healthy
manager that is a control cycle — milliseconds, far inside every input's
freshness deadline.  A manager that hangs can hold the handler for about four
seconds, and the next status report will then say inputs went stale: *true*, since
this supervisor was not looking, but the message names the producer rather than
the pause.  What would fix it is the arbitration in a callback group of its own on
a multi-threaded executor, which costs a lock over the held messages and the latch
that are plain members today — a change to the node's concurrency model, and not
this issue's.

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

The deadlines are **per input**, not one number: the passive pair comes
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

There used to be a second cause here — the producer's own `valid == false`, with
its `status` string carried through unedited.  `sensor_msgs/JointState` carries
no such flag, so it is gone: age is the whole test for this input, and a dead
IMU behind a live publisher will not be noticed.  That is a real capability loss
and it is deliberate — the passive source has no health signal left to report,
and a flag fed by something that cannot fail-report is worse than no flag.

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

**Nothing on this path acts on its own.**  No stop, no ramp, no command — on the
emergency stop least of all, since §6.1 makes the software's relationship to the
stop chain supplementary and one-directional.  (The one thing in this package
that does act unasked is the escalation hand-back, and it is on a different
signal entirely: the horizon producer's, not the stop chain's.)  What the latch
*does* reach is the
arbitration: a latched fault refuses a switch into a motion mode, with the
latched cause as the reason, and `MODE_IDLE` stays reachable.  That is a refusal
of something asked for, not an action taken; the latch still lowers only through
`/crane/clear_fault`, and the expression that decides whether it is up is shared
with the status cycle so the two cannot disagree about what is latched.

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
is slice 6 — and it does not stop a motion already running for being swingy.

The *refusal* half is still not wired, and `/crane/set_mode` is not where it
belongs.  A mode change is not a motion: activating `MODE_FOLLOW` puts a
controller in charge of the claim, and the motion starts when a goal reaches that
controller.  Refusing the mode for a swinging load would refuse the crane the
ability to *hold*, which is the opposite of the duty.  What the row asks for is a
refusal at the **goal**, and this supervisor has no authority over goals: the
task layer sends them straight to the trajectory controller's action.  So what is
here is the report and the three-valued signal somebody else refuses on, and the
gap is named rather than closed by putting the check where it happens to fit.

Two different things come off the passive rate and they are not interchangeable:

| | What it is | Where it goes |
|---|---|---|
| the bound | a configured rate per passive coordinate, crossed or not | `FAULT_SWAY`, naming which of `theta6_tip_joint` and `theta7_tilt_joint` crossed it, with the rate and the bound |
| the predicate | **three-valued** — settled, not settled, unknown — held over a dwell and released through a hysteresis | `crane_msgs/SwaySettled` on `/crane/sway_settled`, **and** the clause every status report ends in |

**Both are on the rate and neither is on the angle.**  Two independent reasons,
either of which would be enough on its own.  The published angle carries an
uncalibrated constant offset — the calibrated 2-D spline for the real double
hinge needs calibration data this workspace does not carry — and
`theta7_tilt_joint`'s limits are not centred on zero either.
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

### The predicate is a field, and it is a stream of its own

`crane_msgs/SupervisorStatus` carries `mode`, `fault`, `tracking_error`,
`inside_working_cell`, `deadman_held` and `message`, and none of them is a
three-valued sway predicate.  **`crane_msgs` is frozen**, and PRD §15's amendment
rule makes a *field add* a slice of its own that has to name every consumer —
while a *new message* is additive and has no ceremony.  So the predicate got a
message rather than a field: `crane_msgs/SwaySettled` on `/crane/sway_settled`,
`SETTLED_UNKNOWN=0`/`SETTLED_NO=1`/`SETTLED_YES=2`, the two rates the verdict was
decided from, and the sentence.  `SupervisorStatus` was not touched.

The `fault` field is **not** an alternative and was rejected rather than
overlooked: it carries one cause per cycle in a fixed precedence, so a cycle
reporting `FAULT_ESTOP` says nothing about the sway, and "no `FAULT_SWAY`" would
read as "settled" — which is exactly the two-valued defect the three states exist
to prevent.  `diagnostic_msgs/DiagnosticArray` was the other candidate and is
recorded as settled in `crane_msgs`' own README, not re-opened here.

**The clause stayed.**  `kSettledClausePrefix` still ends every status report,
and the two cannot disagree because they are not two compositions of the same
verdict: `SwaySettled.message` is the bytes `decide()` already appended to the
report, lifted out of it by the adapter, and `SwaySettled.settled` is a cast of
the value that produced them.  Nothing about the predicate is recomputed in the
node.  `test_status_stream.cpp` pairs the two streams by the `header.stamp` they
share and asserts, cycle by cycle, that the field names the verdict the sentence
opens with — starts-with and not contains, because "not settled" contains
"settled" and a containment test would pass on the one disagreement worth
catching.

What is still owed is the *acting*: §5 row 7's action is "refuse to **start** a
motion that depends on sway being settled", and the refusal belongs at the goal,
which this package has no authority over.  The signal is now a field somebody
else can branch on, which is what it was for.

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

## The optimizer's verdict, and the one mode this supervisor leaves on its own

`/crane/mpc/solver_health` carries a `SupervisorStatus` code for the same reason
`/crane/velocity_controller/health` does, and it is merged the same way: the
value on `/crane/supervisor/status` is the value `crane_mpc` put on its own
stream, unrenumbered and un-restated, with the producer's account of the cycle
carried into `message` (ROS 2 Interfaces §4).  `FAULT_SOLVER` is the one code that
producer raises.

**The merge is scoped by the mode, not by the stream.**  In `MODE_FOLLOW`
`crane_mpc` is shadowing: it solves, it publishes its verdict, and it drives
nothing, so a failed shadow solve is not a fault of the machine — and reporting
it would put a standing fault on the operator's stream on every profile from the
day the producer was composed.  In `MODE_MPC` the same report is the command path
having stopped producing.  That is why this stream is not an `Input` (an `Input`
is a stream whose *silence* raises a fault, and this one's must not) while its
*code* travels all the same.  Two more conditions: the report has to be inside
`horizon_timeout`, because a code nobody has heard since is not an observation;
and a producer that goes silent altogether raises no solver fault at all — what
an operator gets then is the receiver running out of plan, which is the only
thing either node can still see.

It sits **above** the inner loop's codes in the order of *[What one report
says](#what-one-report-says)* and below everything that removes part of this
supervisor's own view.  Above the inner loop because when the optimizer stops,
the receiver runs out of plan and raises `FAULT_REFERENCE_STALE` — the
consequence, one layer down, of this cause — and §5.0 is explicit that what the
task layer is owed is the typed cause rather than a symptom it has to infer.  The
cost is that the inner loop's own `FAULT_STATE_HEALTH` is outranked while the
optimizer is failing in `MODE_MPC`; it is a real cost and it is the smaller of
the two.

### The escalation, and the stop that follows the fall back

[[mpc]] §6 ends with "on repeated failure, stop and hand control back to the
supervisor", and [[control_architecture]] §5 row 3 is the only duty in that table
whose action has two halves: *fall back*, and **then** *stop and report*.  The
fall back is not this package's.  `crane_mpc` shifts its previous solution while
it still believes it; past `max_consecutive_failures` it stops publishing
altogether and says so; `crane_velocity_controller` then runs out of plan and
takes the velocity command to zero over its own `horizon_expiry_ramp`.  Three
mechanisms, none of them here.

The stop is this package's, and it is §5.2 step 1: deactivate the claim.  Four
conditions, and `solver_handback()` is where they are written down:

| | What it reads | Why |
|---|---|---|
| the mode | `active_mode()` is `MODE_MPC` | in `MODE_FOLLOW` the escalation stops a horizon nobody is executing |
| freshness | the report is inside `horizon_timeout` | a verdict nobody has heard since is not an observation |
| the escalation | `FAULT_SOLVER` **and** `applied_previous_solution == false` | the producer saying nothing went out, rather than that a shifted plan did |
| the fall back | the inner loop is reporting `FAULT_REFERENCE_STALE` | the receiver saying it has run out of plan, so the ramp has engaged |

The last one is the whole reason this is a predicate and not a branch in
`resolve()`.  Releasing the claim on the escalation *itself* would take the
command interface off a receiver that still had a second of good plan in hand and
put on it exactly the commanded step §5.2 step 3 and [[mpc]] §6 both refuse.  What
it cannot wait for is the ramp *finishing*: neither `VelocityControllerHealth` nor
`SolverHealth` carries a field for that, and adding one is a `crane_msgs` field
add, which PRD §15 makes a slice of its own.  So the release lands inside the ramp
rather than after it, and the residual step is bounded by `horizon_expiry_ramp` —
0.2 s of a command already on its way down — instead of by the speed the crane
happened to be moving at.  That bound is stated rather than measured, and it is
the one loose end in this path.

**It releases to `MODE_IDLE`, never to `MODE_FOLLOW`.**  §5's `[!important]` box
is the reason: the supervisor stops, it does not decide what happens next.
Resuming a motion is the task layer's decision, and PRD §10 step 4 needs a
reference that starts at the current MPC setpoint, which no supervisor can
produce.  The release goes through `arbitrate_mode()` exactly as an operator's
`MODE_IDLE` request does — same preconditions, same single strict switch call,
same exclusion of a controller the incoming mode also wants — and the producer
goes back to `shadow` **after** the claim has moved, because it is the same
handover run by a different caller.

It is attempted **once** per episode.  The calls block this node's status timer
for as long as a mode switch does, and a release the controller manager refused
will be refused again next cycle; retrying it every 50 ms would take the 20 Hz
stream down with it.  A failure is logged at `ERROR` with the manager's own
account, and `FAULT_SOLVER` goes on standing on the stream — which is the signal
the task layer branches on.

## What is not computed

`inside_working_cell` is `false` and is not computed in this slice: the virtual
working cell of §5.1 needs forward kinematics that `crane_model`'s production
backend does not have until slice 4, and an `inside_working_cell` computed from
nothing would be worse than one that is visibly not computed yet.

`deadman_held` and `tracking_error` are no longer among them.  Both are read off
an input and reported on every status, whatever `fault` says.

## Configuration

Fourteen read-only parameters are shipped in `config/crane_supervisor.yaml`:
`pendulum_state_timeout`, `remote_ctrl_timeout`, `controller_state_timeout`,
`controller_health_timeout`, `controller_manager_timeout`, `horizon_timeout`,
`mpc_node`, `mode_controllers.follow`, `mode_controllers.mpc`,
`tool_controllers`, the four under `sway`, and `deadman_button`.  The first four are the freshness deadlines of the four
`Input`s, one each, and they are the only place those numbers exist — the core's
array has no default, so a deadline that is not configured is a node that does
not start.  Their names are the ones `crane_bringup` already passes; they read
`timeout` where the core reads `deadline`, and the two mean the same thing.

`controller_manager_timeout` is the fifth margin and the one that is not an
`Input`'s: how old the manager's newest answer may be and still say which mode is
active.  Past it the mode reads as *not known* and every mode change is refused.
Its age is measured from **arrival** rather than from a header stamp, because
`ListControllers` carries none — weaker than the four above, and weaker in the
safe direction, since it cannot be fooled by a publisher whose clock ran ahead.

`horizon_timeout` is the sixth, and the second that is not an `Input`'s: how old
the horizon producer's newest `crane_msgs/SolverHealth` may be and still say it
is solving.  It is PRD §10 step 2's whole margin — past it a request for
`MODE_MPC` is refused with the age and the deadline in the sentence, before the
trajectory controller is deactivated and before the controller manager is asked
anything at all.  Derived like the two 20 Hz margins, from a 12.5 Hz contract:
80 ms of publication period plus 50 ms of status period plus the 50.4 ms worst
control-cycle gap in the recorded machine data is 180 ms, and 300 ms clears it.

`mpc_node` names the node a profile composed the horizon producer under, and is
the one name in this package that reaches a *node* rather than a topic or a
service.  Empty is admissible and means this deployment composes no producer;
`validate()` then refuses a configuration that gives `MODE_MPC` controllers
anyway, because the claim would move to a path whose producer publishes
nothing.

`mode_controllers` and `tool_controllers` are names and not types, because the
architecture fixes the four modes and a profile fixes what implements them and
under which names the manager loaded them.  `follow` is the pair the `fake` and
`hardware` profiles spawn, in the cascade's activation order (PRD §5).  `mpc` is that
pair without the trajectory controller, because PRD §10 step 3 puts the same
inner loop on both paths.  `manual` is left unset and that is the honest state:
no CBS profile composes a manual controller, so `MODE_MANUAL` is a mode this
deployment does not implement and the request is refused with that as the reason.
`tool_controllers` is empty because this composition has one claim — see *Two
claims, not one machine*.  `validate()` refuses two modes that name the *same
set* of controllers, a name repeated within one list, a controller shared with
the tool claim, any controller under `idle`, and a `MODE_MPC` that is implemented
with no producer named.  It no longer refuses lists that merely overlap: that
rule was right while `mpc` was empty and became wrong the moment it was
populated — see *What is reported is what is active*.

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
carries the row.  Nor does `/crane/mpc/solver_health`: `crane_mpc` publishes on
the absolute contract name, and both profiles compose it.

The `mode` this node moves on `crane_mpc` is the other half of the composition
and is owed by the profile as well: `mpc_node` has to name the node the producer
was loaded under, and nothing else composed beside it may hold a parameter client
on that node.  `crane_bringup`'s launch contract asserts both, because "which
path is live is the supervisor's decision **alone**" is a claim about the whole
running system and no single package can make it.

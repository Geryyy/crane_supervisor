"""Static proof that this package has no path to a motion command.

wiki/control_architecture.md 5.2 gives the supervisor a stop path that does not
run through the active controller, and
wiki/implementation/commissioning_prerequisites.md row 3 makes that path
conditional on a verification only a safety reviewer on the machine can perform.
Until it is performed the system must not advertise a safety function it cannot
deliver (PRD user story 54).  So the absence below is the feature, and a feature
that is not asserted is a feature that comes back by accident.

**Since issue 028 the publisher list has two entries, and the second is named
rather than the rule loosened.**  The settled predicate of
wiki/control_architecture.md 5 row 7 is what a behaviour tree gates a grip action
on, and `crane_msgs/SupervisorStatus` has no field for it, so it rides on a
second stream (`crane_msgs/SwaySettled` on `/crane/sway_settled`).  The assertion
below still pins every publisher *by type and in source order* -- what changed is
that the list is two long instead of one.  It is a status stream by every test
the first one passes: reliable depth 1 at 20 Hz, `header.frame_id` empty, and a
payload of one `uint8`, two rates and a sentence.  Nothing on it is a setpoint,
and no controller subscribes to it -- the consumer is the task layer, which is
where the *refusal* half of that row belongs.

**Issue 054 added a third client and a second subscription site, on the same
terms.**  ROS 2 Interfaces 4's "One command path" ends by saying that which of
the two producers is live is the supervisor's decision *alone* and that the two
never drive at once; issue 053 made that a state of `crane_mpc` and left its
`mode` parameter writable so that this node could move it.  So there is one
`rcl_interfaces/SetParameters` client, bounded from three sides: the type is
pinned, the parameter it may carry is pinned as a literal, and the two values
that parameter may take are pinned as literals too.  The subscription is
`/crane/mpc/solver_health`, the evidence PRD 10 step 2's freshness check is made
against; it does not go through the `subscribe()` helper because that helper
takes an `Input`, and an `Input` is a stream whose absence raises a fault on the
status -- which this one's must not.  Both call sites and both types are pinned
below, so a third subscription is still a line somebody has to write here.

**Since issue 026 the guard has one service-call exception, and it is named
rather than removed.**  ROS 2 Interfaces 5 puts `/controller_manager/switch_controller`
behind the supervisor -- mode changes go through it, not from the behaviour
tree -- so this package holds two clients on the controller manager and nothing
else does.  The guard was updated for that and not weakened: the blanket bans on
`create_client` and on the string `controller_manager` are gone, and what
replaced them is narrower.  The two service types are pinned by name, so a third
client cannot appear; the two service names are pinned as literals, so a client
cannot be pointed anywhere else; every identifier that would let this package
*hold* a controller, a plugin or a hardware interface is still banned; and the
publisher list is unchanged, so nothing here can emit a setpoint.  A switch
decides which controller holds the claim and carries no command.

Static, and it reads code rather than prose: comments and string literals are
stripped before the identifier scan, so the README and the header comments are
free to explain at length what is missing and why without the guard mistaking
the explanation for the thing.  The one scan that *does* read literals is the
ROS-name check, because a topic is a literal and nothing else.

Nothing here starts a node, a launch or a DDS participant.
"""

import re
import xml.etree.ElementTree as ET
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CMAKE = PACKAGE_ROOT / "CMakeLists.txt"
PACKAGE_XML = PACKAGE_ROOT / "package.xml"

NON_SOURCE_DIRECTORIES = {".git", "__pycache__", "build", "install", "log"}

# The contract names of ROS 2 Interfaces 4 and 5 this package may know: the two
# status streams it owns, the four inputs it carries end to end, the two services
# it serves, and the two it calls on the controller manager.  Any other ROS name
# in the sources is a reach this package does not have.
STATUS_TOPIC = "/crane/supervisor/status"
SWAY_SETTLED_TOPIC = "/crane/sway_settled"
PENDULUM_STATE_TOPIC = "/crane/pendulum_state"
REMOTE_CTRL_STATES_TOPIC = "/crane/remote_ctrl_states"
CONTROLLER_STATE_TOPIC = "/crane/controller_state"
CONTROLLER_HEALTH_TOPIC = "/crane/velocity_controller/health"
CLEAR_FAULT_SERVICE = "/crane/clear_fault"
SET_MODE_SERVICE = "/crane/set_mode"

# The two names of the first exception.  `list_controllers` is read-only by
# construction -- the request carries no fields at all -- and `switch_controller`
# is the one call in this package that reaches the machine.  Both are pinned as
# literals so that a client cannot be pointed at a third service without this
# file changing.
LIST_CONTROLLERS_SERVICE = "/controller_manager/list_controllers"
SWITCH_CONTROLLER_SERVICE = "/controller_manager/switch_controller"

# The third exception, added by issue 054, and the one ROS name in this package
# that is **not** written out whole.  ROS 2 Interfaces 4's "One command path"
# makes which of the two producers is live the supervisor's decision alone, and
# issue 053 made that a state of `crane_mpc` reachable through its own parameter
# service.  Which node that is is a profile's decision -- it is the `mpc_node`
# parameter -- so the source carries the suffix and composes the rest.
#
# That is a hole in the standalone-name scan below, because a single-segment
# literal is not a ROS name to that expression, so it is closed by hand: the
# scan for single-segment names is what pins this one, and nothing else in the
# package may carry one.
PARAMETER_SERVICE_SUFFIX = "/set_parameters"
PERMITTED_NAME_FRAGMENTS = {PARAMETER_SERVICE_SUFFIX}
# A ROS name with exactly one segment.  Only ever a fragment: a service or topic
# this package actually used would be absolute and would have at least two.
NAME_FRAGMENT = re.compile(r"/[A-Za-z_][A-Za-z0-9_]*")

# The horizon producer's own status stream, and the evidence PRD 10 step 2's
# freshness check is made against.  It is a subscription and stays one, and it
# is deliberately `solver_health` rather than `/crane/mpc/horizon`: in shadow
# mode -- the state every switch into MODE_MPC is made from -- `crane_mpc`
# publishes nothing at all on the contract topic, so a check against the horizon
# could never pass.  It also keeps this package clear of `trajectory_msgs`,
# which is banned outright below.
SOLVER_HEALTH_TOPIC = "/crane/mpc/solver_health"

PERMITTED_ROS_NAMES = {
    STATUS_TOPIC,
    SWAY_SETTLED_TOPIC,
    PENDULUM_STATE_TOPIC,
    REMOTE_CTRL_STATES_TOPIC,
    CONTROLLER_STATE_TOPIC,
    CONTROLLER_HEALTH_TOPIC,
    SOLVER_HEALTH_TOPIC,
    CLEAR_FAULT_SERVICE,
    SET_MODE_SERVICE,
    LIST_CONTROLLERS_SERVICE,
    SWITCH_CONTROLLER_SERVICE,
}

# The command path itself, which this package **mentions and never touches**.
# The refusal PRD 10 step 2 composes has to say why the freshness of the horizon
# is judged on the solver-health stream rather than on the horizon, and the
# answer names the horizon: in shadow mode nothing is published on it at all, so
# a check against it could never pass.  So it is admissible inside a sentence
# and inadmissible as a name this package uses -- the standalone scan below is
# exact and does not contain it, which is what keeps the distinction real.
HORIZON_TOPIC = "/crane/mpc/horizon"
MENTIONED_ROS_NAMES = PERMITTED_ROS_NAMES | {HORIZON_TOPIC}

# A ROS name standing on its own, and a ROS name quoted inside a sentence.  The
# second is needed because the status messages name the topic they are about,
# which is the whole reason an operator can act on them.
ROS_NAME = re.compile(r"/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)+")
CRANE_NAME = re.compile(r"/crane/[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*")

# Names of the machine-facing graph.  None of them may appear in a literal at
# all: this package neither calls them nor mentions them in something it
# publishes.  `/controller_manager` came off this list when the switch client
# landed and was replaced by something narrower -- the exact two names above,
# asserted as a set below -- because a blanket ban on the string would also have
# banned the two calls ROS 2 Interfaces 5 puts here on purpose.
FORBIDDEN_ROS_NAMES = ("/joint_states", "/cbs/")

# Every way this package could reach the machine, by the name it would have to
# use to do it.
#
# `create_service` came off it when `/crane/clear_fault` landed -- the reason it
# was ever there was that the service was a later issue.  What replaced the
# blanket ban is narrower and stricter: the test below names the services this
# package may serve and the types it may serve them with.
#
# `create_client`, `controller_manager`, `SwitchController` and
# `switch_controller` came off it the same way, in issue 026, and for the same
# kind of reason: they were banned because the arbitration was a later issue, and
# this is that issue.  ROS 2 Interfaces 5 makes the supervisor the only element
# that calls `/controller_manager/switch_controller`, so a ban here would have to
# be lifted somewhere for the stack to have an arbiter at all -- and the safest
# place for it to be lifted is the one package whose every other reach is
# asserted.  What replaces them is the client test below: two service types by
# name, two service names by literal, and nothing else.
#
# What stays banned is everything that would let this package *hold* a piece of
# the control loop rather than call the manager over the graph:
# `controller_interface` and `hardware_interface` are how a package becomes a
# controller, `command_interface` is how it writes one, and `load_controller`,
# `unload_controller`, `configure_controller` and
# `set_hardware_component_state` are the manager calls that go beyond arbitrating
# a claim -- loading and unloading are a bringup authority this node does not
# have, and a hardware component's state is the machine itself.
#
# `JointTrajectory` came off it earlier, and for the same kind of reason: it was
# a stand-in for `trajectory_msgs`, the package a *commanded* trajectory is typed
# with, and it also matched `control_msgs/JointTrajectoryControllerState` --
# which is a controller describing itself, subscribed to and never published.
# The ban is now on the package that carries the command type, so a
# `trajectory_msgs::msg::JointTrajectory` publisher still cannot appear, and the
# subscription test below pins the exact four types this node consumes.
FORBIDDEN_IDENTIFIERS = (
    "load_controller",
    "unload_controller",
    "configure_controller",
    "set_hardware_component_state",
    "command_interface",
    "CommandInterface",
    "hardware_interface",
    "controller_interface",
    "trajectory_msgs",
    "FollowJointTrajectory",
    "JointJog",
    "Twist",
    "create_generic_publisher",
)

# Every publisher this package holds, by type and in source order.  A publisher
# is how a status node becomes a command node -- the topic would be new, the QoS
# would be new, and nothing else about the package would look different -- so the
# list is enumerated one entry at a time and never widened by a pattern.
#
# `crane_msgs/SwaySettled` was added in issue 028 and is the second and last
# entry.  The argument for it is the one the README records: the settled
# predicate is a *report*, its consumer is the task layer rather than a
# controller, and the message carries a `uint8`, two rates and a sentence -- no
# joint, no duration, no setpoint, so it is not a shape a motion command fits in.
# What would *not* be admitted here is a publisher of a type that carries one,
# whatever the topic were called: `trajectory_msgs` is banned outright above.
PERMITTED_PUBLISHER_TYPES = [
    "crane_msgs::msg::SupervisorStatus",
    "crane_msgs::msg::SwaySettled",
]

# The two services this package serves, and the only types it may serve them
# with, in source order.  `std_srvs/Trigger` takes no arguments at all;
# `crane_msgs/SetMode` takes one `uint8` and nothing else -- there is no
# setpoint, no joint and no duration in either request, so neither is a shape a
# caller can smuggle a motion command through.
PERMITTED_SERVICE_TYPES = ["std_srvs::srv::Trigger", "crane_msgs::srv::SetMode"]

# The three clients this package holds, and the only types it may hold them
# with.  A fourth would be a reach this package does not have; the request field
# of `ListControllers` is empty, the request field of `SwitchController` carries
# controller names and no setpoint, and the `SetParameters` request is checked
# below to carry one parameter whose name is a literal in this package.
#
# The third landed in issue 054 and it is named rather than the rule relaxed,
# exactly as the first two were.  ROS 2 Interfaces 4's "One command path" ends
# by saying that which of the two producers is live is the supervisor's decision
# *alone* and that the two never drive at once; issue 053 made that a state of
# `crane_mpc` and left its `mode` parameter writable so that this node could
# move it.  A parameter call is not a setpoint: it decides which producer may
# publish, and carries no joint, no duration and no value of the machine's.
PERMITTED_CLIENT_TYPES = [
    "controller_manager_msgs::srv::ListControllers",
    "controller_manager_msgs::srv::SwitchController",
    "rcl_interfaces::srv::SetParameters",
]

# The only parameter that client may ever carry, and the only two values it may
# set it to.  Both are `crane_mpc`'s contract rather than this package's
# setting, and pinning them here is what keeps the reach at "which producer may
# publish" rather than at "any parameter of any node".
PRODUCER_MODE_PARAMETER = "mode"
PRODUCER_MODE_VALUES = ("active", "shadow")

# What the package is allowed to build against.  A dependency is the cheapest
# way to grow a reach, so the manifest is a whitelist and not a blacklist.
PERMITTED_DEPENDENCIES = {
    "ament_cmake",
    "ament_cmake_gtest",
    "ament_cmake_pytest",
    "control_msgs",
    "controller_manager",
    "controller_manager_msgs",
    "crane_model",
    "crane_msgs",
    "epsilon_crane_msgs",
    "generate_parameter_library",
    "rcl_interfaces",
    "rclcpp",
    "std_srvs",
    "velocity_controllers",
}

# `controller_manager` and `velocity_controllers` are *test* dependencies and
# stay ones.  The S5 harness builds a manager inside the test process and
# switches two real controllers in it; a runtime dependency on either would be
# this package able to instantiate a controller rather than call a service over
# the graph, which is a different reach entirely.  `crane_model` is here for the
# reason it always was: the supervisor asserts the installed dynamics seam and
# links no model into anything it runs.
TEST_ONLY_DEPENDENCIES = {
    "ament_cmake_gtest",
    "ament_cmake_pytest",
    "controller_manager",
    "crane_model",
    "velocity_controllers",
}

RUNTIME_DEPENDENCY_TAGS = {
    "depend",
    "build_depend",
    "build_export_depend",
    "exec_depend",
}

DEPENDENCY_TAGS = {
    "depend",
    "build_depend",
    "build_export_depend",
    "exec_depend",
    "test_depend",
    "doc_depend",
    "buildtool_depend",
}


def _sources():
    """Return every C++ source and header this package builds, sorted."""
    found = sorted(
        path
        for directory in ("src", "include")
        for path in (PACKAGE_ROOT / directory).rglob("*")
        if path.is_file() and path.suffix in (".cpp", ".hpp")
    )
    assert found, "no C++ source was scanned; the guard would pass vacuously"
    return found


def _split(source):
    """Return `(code, literals)` of one C++ translation unit.

    A hand-rolled scanner rather than a parser, because the question is only
    which characters are inside a comment, inside a string, or neither -- and
    the answer decides whether a word in this package is an explanation or a
    call.
    """
    code = []
    literals = []
    index = 0
    length = len(source)
    while index < length:
        character = source[index]
        pair = source[index : index + 2]
        if pair == "//":
            index = source.find("\n", index)
            if index == -1:
                break
        elif pair == "/*":
            end = source.find("*/", index + 2)
            index = length if end == -1 else end + 2
        elif character in "\"'":
            end = index + 1
            while end < length and source[end] != character:
                end += 2 if source[end] == "\\" else 1
            literals.append(source[index + 1 : end])
            code.append(" ")
            index = end + 1
        else:
            code.append(character)
            index += 1
    return "".join(code), literals


def _code_and_literals():
    """Return `(code, literals)` over every source, as two flat collections."""
    code, literals = [], []
    for path in _sources():
        split_code, split_literals = _split(path.read_text(encoding="utf-8"))
        code.append((path, split_code))
        literals += split_literals
    return code, literals


def _dependencies(tags=DEPENDENCY_TAGS):
    root = ET.parse(PACKAGE_XML).getroot()
    return {
        element.text.strip()
        for element in root
        if element.tag in tags and element.text and element.text.strip()
    }


def test_no_source_reaches_a_controller_or_a_command_interface():
    """The identifier scan, over code with comments and literals removed."""
    code, _literals = _code_and_literals()
    for path, body in code:
        for identifier in FORBIDDEN_IDENTIFIERS:
            assert identifier not in body, f"{path.name}: {identifier}"


def test_the_package_publishes_two_status_streams_and_only_reads_its_inputs():
    """Two publishers, four subscriptions, two services, two clients.

    A publisher nobody enumerated is how a status node becomes a command node:
    the topic would be new, the QoS would be new, and nothing else about the
    package would look different.  So the list is pinned by type and in source
    order, and it did *not* change when the switch client landed -- whatever this
    package can now ask the controller manager to do, it still cannot emit a
    setpoint.

    It grew by one in issue 028, and by naming the entry rather than by relaxing
    the rule: the settled predicate had nowhere to go on a frozen
    `SupervisorStatus`, and `crane_msgs/SwaySettled` carries a verdict, the two
    rates it was decided from and a sentence.  Both entries are reports.

    `/crane/velocity_controller/health` is a subscription and stays one.  The
    inner loop's fault reaches this node because the controller publishes it;
    nothing here may publish on a topic named for a controller.
    """
    code, literals = _code_and_literals()
    publishers, subscriptions, subscribed, services, clients = [], [], [], [], []
    for _path, body in code:
        publishers += re.findall(r"create_publisher<([A-Za-z0-9_:]+)>", body)
        subscriptions += re.findall(r"create_subscription<([A-Za-z0-9_:]+)>", body)
        subscribed += re.findall(r"subscribe<([A-Za-z0-9_:]+)>\(\s*Input::", body)
        services += re.findall(r"create_service<([A-Za-z0-9_:]+)>", body)
        clients += re.findall(r"create_client<([A-Za-z0-9_:]+)>", body)

    assert publishers == PERMITTED_PUBLISHER_TYPES, publishers
    # The exceptions, pinned by type.  A fourth client -- or any of these three
    # replaced by something that carries a command -- is a reach this package
    # does not have and a line this file has to gain.
    assert clients == PERMITTED_CLIENT_TYPES, clients
    # `create_subscription` is called in exactly two places, and the second is
    # named rather than the rule loosened.  The first is the `subscribe()`
    # helper that every `Input` goes through, which takes an enumerator and
    # therefore a freshness deadline (`test_every_input_has_a_deadline.py`
    # asserts the registry side of that).  The second is the horizon producer's
    # own status stream, which *cannot* go through the helper: an `Input` is by
    # definition a stream whose absence raises a fault on the status, and this
    # one's must not -- an optimizer that is quiet while the machine is in
    # MODE_FOLLOW is the ordinary state of this stack, and ROS 2 Interfaces 4
    # records that the supervisor merging `FAULT_SOLVER` onto its status stream
    # is a slice of its own.  What stays pinned here is the set of message
    # *types* this node consumes, because a command path would have to change it.
    assert subscriptions == ["MessageT", "crane_msgs::msg::SolverHealth"], subscriptions
    assert subscribed == [
        "crane_msgs::msg::PendulumState",
        "epsilon_crane_msgs::msg::RemoteCtrlStates",
        "control_msgs::msg::JointTrajectoryControllerState",
        "crane_msgs::msg::VelocityControllerHealth",
    ], subscribed
    assert services == PERMITTED_SERVICE_TYPES, services

    # A literal that is nothing but a ROS name is one this package uses; a ROS
    # name inside a sentence is one it tells an operator about.  Both are held
    # to the same names.
    standalone = {literal for literal in literals if ROS_NAME.fullmatch(literal)}
    assert standalone == PERMITTED_ROS_NAMES, sorted(standalone)

    # And the one name this package composes rather than writes out.  A
    # single-segment literal is not a ROS name to the expression above, so it
    # would slip through the set assertion; it is pinned here instead, and the
    # set is exact, so a second fragment -- a suffix pointing a client at
    # somebody else's service -- fails this line.
    fragments = {
        literal
        for literal in literals
        if NAME_FRAGMENT.fullmatch(literal) and not ROS_NAME.fullmatch(literal)
    }
    assert fragments == PERMITTED_NAME_FRAGMENTS, sorted(fragments)

    quoted = {name for literal in literals for name in CRANE_NAME.findall(literal)}
    assert quoted <= MENTIONED_ROS_NAMES, sorted(quoted - MENTIONED_ROS_NAMES)

    for literal in literals:
        for forbidden in FORBIDDEN_ROS_NAMES:
            assert forbidden not in literal, f"{forbidden}: {literal}"


def test_the_parameter_client_may_carry_one_parameter_and_two_values():
    """The third client is narrow, and its narrowness is not a comment.

    `rcl_interfaces/SetParameters` would reach any parameter of any node it was
    pointed at, so what makes it a bounded reach is what this package puts in
    the request: one parameter, named by a literal here, set to one of two
    values that are also literals here.  All three are `crane_mpc`'s contract
    rather than this package's setting -- the producer's own declaration
    validates the string against exactly those two -- and pinning them is what
    keeps the exception at "which producer may publish".

    The node it is pointed at is deliberately *not* pinned: which node a
    deployment composed the producer under is a profile's decision and reaches
    this package as the `mpc_node` parameter, which is why the service name is
    composed from a suffix rather than written out.
    """
    code, literals = _code_and_literals()
    assert PRODUCER_MODE_PARAMETER in literals, PRODUCER_MODE_PARAMETER
    for value in PRODUCER_MODE_VALUES:
        assert value in literals, value

    # One `push_back` onto the request, so the call cannot grow a second
    # parameter without this file changing.
    pushed = sum(
        len(re.findall(r"parameters\.push_back", body)) for _path, body in code
    )
    assert pushed == 1, pushed


def test_the_manifest_and_the_build_declare_no_way_to_reach_the_machine():
    """The dependency surface, as a whitelist, split by when it is reachable.

    `controller_manager_msgs` is a runtime dependency and carries two service
    *types*: the node calls the manager over the graph and holds no controller,
    no plugin and no hardware interface.  `controller_manager` itself, and the
    two real controllers the harness switches between, are test dependencies and
    are asserted to stay ones -- linking the manager into the deployed node
    would be a different reach from calling it.
    """
    declared = _dependencies()
    assert declared <= PERMITTED_DEPENDENCIES, sorted(declared - PERMITTED_DEPENDENCIES)

    runtime = _dependencies(RUNTIME_DEPENDENCY_TAGS)
    leaked = runtime & TEST_ONLY_DEPENDENCIES
    assert not leaked, sorted(leaked)

    source = re.sub(r"#[^\n]*", "", CMAKE.read_text(encoding="utf-8"))
    found = set(re.findall(r"find_package\s*\(\s*([A-Za-z0-9_]+)", source))
    assert found <= PERMITTED_DEPENDENCIES, sorted(found - PERMITTED_DEPENDENCIES)

    # And the test-only ones are found only inside `if(BUILD_TESTING)`, so a
    # deployment build cannot link them even by accident.
    deployment = source[: source.index("if(BUILD_TESTING)")]
    deployed = set(re.findall(r"find_package\s*\(\s*([A-Za-z0-9_]+)", deployment))
    assert not (deployed & TEST_ONLY_DEPENDENCIES), sorted(
        deployed & TEST_ONLY_DEPENDENCIES
    )


def test_the_package_installs_no_plugin_a_controller_manager_could_load():
    """A controller is loaded by name out of a pluginlib description.

    This package is a node and not a controller, so an exported plugin
    description would be the one artifact that lets `controller_manager`
    instantiate something out of it -- inside the real-time cycle, with the
    command interfaces in reach.
    """
    exported = [
        path
        for path in PACKAGE_ROOT.rglob("*.xml")
        if not set(path.parts) & NON_SOURCE_DIRECTORIES and path.name != "package.xml"
    ]
    assert not exported, [str(path) for path in exported]

    source = CMAKE.read_text(encoding="utf-8")
    assert "pluginlib_export_plugin_description_file" not in source

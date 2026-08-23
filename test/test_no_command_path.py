"""Static proof that this package has no path to a motion command.

wiki/control_architecture.md 5.2 gives the supervisor a stop path that does not
run through the active controller, and
wiki/implementation/commissioning_prerequisites.md row 3 makes that path
conditional on a verification only a safety reviewer on the machine can perform.
Until it is performed the system must not advertise a safety function it cannot
deliver (PRD user story 54).  So the absence below is the feature, and a feature
that is not asserted is a feature that comes back by accident.

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

# The two contract names of ROS 2 Interfaces 4 this package may know: the status
# stream it owns, and the one input slice 3 carries end to end.  Any other ROS
# name in the sources is a reach this slice does not have.
STATUS_TOPIC = "/crane/supervisor/status"
PENDULUM_STATE_TOPIC = "/crane/pendulum_state"
PERMITTED_ROS_NAMES = {STATUS_TOPIC, PENDULUM_STATE_TOPIC}

# A ROS name standing on its own, and a ROS name quoted inside a sentence.  The
# second is needed because the status messages name the topic they are about,
# which is the whole reason an operator can act on them.
ROS_NAME = re.compile(r"/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)+")
CRANE_NAME = re.compile(r"/crane/[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*")

# Names of the machine-facing graph.  None of them may appear in a literal at
# all: this package neither calls them nor mentions them in something it
# publishes.
FORBIDDEN_ROS_NAMES = ("/controller_manager", "/joint_states", "/cbs/")

# Every way this package could reach the machine, by the name it would have to
# use to do it.  `create_client` and `create_service` are on the list not
# because a service commands anything but because `/crane/set_mode` and
# `/crane/clear_fault` are a later issue, and a mode this supervisor has not
# arbitrated is a mode it must not accept.
FORBIDDEN_IDENTIFIERS = (
    "controller_manager",
    "SwitchController",
    "switch_controller",
    "load_controller",
    "unload_controller",
    "configure_controller",
    "set_hardware_component_state",
    "command_interface",
    "CommandInterface",
    "hardware_interface",
    "controller_interface",
    "JointTrajectory",
    "FollowJointTrajectory",
    "JointJog",
    "Twist",
    "create_client",
    "create_service",
    "create_generic_publisher",
)

# What the package is allowed to build against.  A dependency is the cheapest
# way to grow a reach, so the manifest is a whitelist and not a blacklist.
PERMITTED_DEPENDENCIES = {
    "ament_cmake",
    "ament_cmake_gtest",
    "ament_cmake_pytest",
    "crane_model",
    "crane_msgs",
    "generate_parameter_library",
    "rclcpp",
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


def _dependencies():
    root = ET.parse(PACKAGE_XML).getroot()
    return {
        element.text.strip()
        for element in root
        if element.tag in DEPENDENCY_TAGS and element.text and element.text.strip()
    }


def test_no_source_reaches_a_controller_or_a_command_interface():
    """The identifier scan, over code with comments and literals removed."""
    code, _literals = _code_and_literals()
    for path, body in code:
        for identifier in FORBIDDEN_IDENTIFIERS:
            assert identifier not in body, f"{path.name}: {identifier}"


def test_the_package_publishes_one_stream_and_subscribes_to_one_input():
    """One publisher, one subscription, and both on the contract names.

    A second publisher is how a status node becomes a command node: the topic
    would be new, the QoS would be new, and nothing else about the package would
    look different.
    """
    code, literals = _code_and_literals()
    publishers, subscriptions = [], []
    for _path, body in code:
        publishers += re.findall(r"create_publisher<([A-Za-z0-9_:]+)>", body)
        subscriptions += re.findall(r"create_subscription<([A-Za-z0-9_:]+)>", body)

    assert publishers == ["crane_msgs::msg::SupervisorStatus"], publishers
    assert subscriptions == ["crane_msgs::msg::PendulumState"], subscriptions

    # A literal that is nothing but a ROS name is one this package uses; a ROS
    # name inside a sentence is one it tells an operator about.  Both are held
    # to the same two names.
    standalone = {literal for literal in literals if ROS_NAME.fullmatch(literal)}
    assert standalone == PERMITTED_ROS_NAMES, sorted(standalone)

    quoted = {name for literal in literals for name in CRANE_NAME.findall(literal)}
    assert quoted <= PERMITTED_ROS_NAMES, sorted(quoted - PERMITTED_ROS_NAMES)

    for literal in literals:
        for forbidden in FORBIDDEN_ROS_NAMES:
            assert forbidden not in literal, f"{forbidden}: {literal}"


def test_the_manifest_and_the_build_declare_no_way_to_reach_the_machine():
    """The dependency surface, as a whitelist.

    `crane_model` is a test dependency and stays one: the supervisor asserts the
    installed dynamics seam and links no model into anything it runs.
    """
    declared = _dependencies()
    assert declared <= PERMITTED_DEPENDENCIES, sorted(declared - PERMITTED_DEPENDENCIES)

    source = re.sub(r"#[^\n]*", "", CMAKE.read_text(encoding="utf-8"))
    found = set(re.findall(r"find_package\s*\(\s*([A-Za-z0-9_]+)", source))
    assert found <= PERMITTED_DEPENDENCIES, sorted(found - PERMITTED_DEPENDENCIES)


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

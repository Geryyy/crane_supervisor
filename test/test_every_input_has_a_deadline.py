"""Static proof that every input this supervisor holds has a freshness deadline.

wiki/control_architecture.md 5.3: no input may simply stop arriving without a
defined consequence.  This supervisor is the only element that sees every input
that reaches it by topic, so it is where the rule is enforced for those -- and a
rule enforced by a checklist is a rule that lapses the first time somebody adds a
fifth subscription.

So the guard is structural rather than a list.  `Input` is the registry: the
deadline array, the stream array and the policy table are all sized by it, the
one helper that creates a subscription takes one, and a deadline of zero is
refused by `validate()`.  What this file asserts is the shape those pieces have
to keep for that to hold:

* `create_subscription` appears exactly twice in the whole package: inside the
  helper, and on the one stream that deliberately is not an `Input`.  A third
  call site could take a topic name of its own, and an input with no `Input`
  behind it has no deadline, no policy row and no report.

  The second site landed in issue 054 and is named here rather than the count
  loosened, which is the same move the command-path guard makes for its service
  clients.  It is `/crane/mpc/solver_health`, the evidence PRD 10 step 2's
  freshness check is made against, and it *cannot* be an `Input`: an `Input` is
  by definition a stream whose absence raises a fault on the status stream, and
  an optimizer that is quiet while the machine is in MODE_FOLLOW is the ordinary
  state of this stack rather than a defect.  Its `fault` **is** merged onto
  `/crane/supervisor/status` since issue 055 -- unrenumbered, the way
  `VelocityControllerHealth`'s is -- and that is a different question: what the
  merge is scoped by is the *mode*, not the stream's presence, so an absent
  producer still raises nothing here.  5.3's rule is met the way
  the polled controller-manager view meets it: its own configured margin,
  refused by `validate()` when it is missing, and a defined, narrow consequence
  stated where it matters -- no freshness, no switch, with the age and the
  deadline in the refusal.  That margin is asserted below beside the four.
* every enumerator of `Input` is subscribed, has a policy row, and is assigned a
  deadline out of a parameter -- in that same order, so a row cannot describe one
  input while a deadline is written into another.
* `freshness_deadline` carries no default, because a plausible default is exactly
  how an input that nobody configured goes on being watched by a number nobody
  chose.
* no contract name is written down twice, because a topic that lives in two
  places can be changed in one of them.

Static, and stdlib only.  Nothing here starts a node, a launch or a DDS
participant.
"""

import re
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CORE_HEADER = PACKAGE_ROOT / "include" / "crane_supervisor" / "supervisor_core.hpp"
NODE_HEADER = PACKAGE_ROOT / "include" / "crane_supervisor" / "supervisor_node.hpp"
NODE_SOURCE = PACKAGE_ROOT / "src" / "supervisor_node.cpp"
PARAMETER_DECLARATION = PACKAGE_ROOT / "src" / "crane_supervisor_parameters.yaml"

# Every `create_subscription` this package may hold, by type and in source
# order.  The first is the `subscribe()` helper every `Input` goes through -- the
# type is its template parameter, which is what makes "one call site" and "one
# per input" different assertions.  The second is the horizon producer's own
# status stream, which is deliberately not an `Input`; see the module docstring.
PERMITTED_SUBSCRIPTION_TYPES = ["MessageT", "crane_msgs::msg::SolverHealth"]

# A ROS name standing on its own.  The same expression the command-path guard
# uses, for the same reason: a literal that is nothing but a ROS name is a name
# this package uses, as opposed to one it tells an operator about.
ROS_NAME = re.compile(r"/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)+")


def _split(source):
    """Return `(code, literals)` of one C++ translation unit.

    A hand-rolled scanner rather than a parser, because the question is only
    which characters are inside a comment, inside a string, or neither.  The
    header below explains at length which inputs exist and why; an explanation
    must not be mistaken for a subscription.
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


def _code(path):
    return _split(path.read_text(encoding="utf-8"))[0]


def _sources():
    found = sorted(
        path
        for directory in ("src", "include")
        for path in (PACKAGE_ROOT / directory).rglob("*")
        if path.is_file() and path.suffix in (".cpp", ".hpp")
    )
    assert found, "no C++ source was scanned; the guard would pass vacuously"
    return found


def _inputs():
    """Return the enumerators of `Input`, in order, without the `Count` bound."""
    body = _code(CORE_HEADER)
    registry = re.search(r"enum\s+class\s+Input\s*:[^{]*\{(.*?)\}", body, re.S)
    assert registry, "the Input registry is gone; nothing sizes the deadlines any more"
    names = re.findall(r"[A-Za-z_][A-Za-z0-9_]*", registry.group(1))
    assert names, names
    assert names[-1] == "Count", names
    return names[:-1]


def test_every_input_is_subscribed_through_the_one_helper_that_gives_it_a_deadline():
    """Two `create_subscription` sites, and one `subscribe()` call per input.

    The helper takes an `Input`, so a subscription that named no input -- and
    therefore had no deadline, no policy row and no way to be reported -- cannot
    be written through it.  The one stream that is not an `Input` is enumerated
    beside it by type, and a third `create_subscription` anywhere in the package
    would be the subscription this guard exists against, so the list is what is
    asserted rather than the topic.
    """
    created = []
    for path in _sources():
        created += re.findall(r"create_subscription<([A-Za-z0-9_:]+)>", _code(path))
    assert created == PERMITTED_SUBSCRIPTION_TYPES, created

    subscribed = re.findall(
        r"subscribe<[A-Za-z0-9_:]+>\(\s*Input::([A-Za-z0-9_]+)", _code(NODE_SOURCE)
    )
    assert subscribed == _inputs(), subscribed


def test_the_stream_that_is_not_an_input_still_has_a_margin_of_its_own():
    """5.3's rule, met the way the polled controller-manager view meets it.

    `/crane/mpc/solver_health` is not an `Input` and must not be: an optimizer
    that is quiet while the machine is in MODE_FOLLOW is the ordinary state of
    this stack, so its *absence* raises nothing.  What it is *not* exempt from
    is having a deadline at all, and issue 055 gave that margin a second reader:
    PRD 10 step 2 refuses MODE_MPC and says how old the newest report is against
    what margin, and `resolve()` merges `FAULT_SOLVER` only off a report inside
    the same margin -- a code nobody has heard since is not an observation.
    There is no margin to say either thing without a configured one.

    So the same three things the four inputs get are asserted here one at a
    time: a member on the config, a parameter it is read out of, and no default
    on the struct -- because a plausible default is exactly how a stream nobody
    configured goes on being judged by a number nobody chose.
    """
    core = _code(CORE_HEADER)
    declaration = re.search(r"double\s+horizon_deadline\s*(\{[^;]*\})?\s*;", core)
    assert declaration, "the horizon producer's margin is no longer on SupervisorConfig"
    assert (declaration.group(1) or "").strip() in ("", "{0.0}"), declaration.group(1)

    assert "config_.horizon_deadline = parameters." in _code(NODE_SOURCE)
    assert "horizon_timeout:" in PARAMETER_DECLARATION.read_text(encoding="utf-8")


def test_every_input_has_a_policy_row_and_a_deadline_out_of_a_parameter():
    """One row and one assignment per input, both in enum order.

    Order and not just membership: a row that described one input while a
    deadline was written into another would judge a stream by somebody else's
    margin and report it under somebody else's topic, and both stay invisible for
    as long as the two streams are healthy.
    """
    inputs = _inputs()

    rows = re.findall(r"\{\s*Input::([A-Za-z0-9_]+)\s*,", _code(CORE_HEADER))
    assert rows == inputs, rows

    assigned = re.findall(
        r"deadline\(\s*Input::([A-Za-z0-9_]+)\s*\)\s*=", _code(NODE_SOURCE)
    )
    assert assigned == inputs, assigned


def test_the_deadlines_have_no_default_so_an_unconfigured_input_cannot_start():
    """Value-initialised to zero, and zero is what `validate()` refuses.

    A plausible default is how an input nobody configured goes on being watched
    by a number nobody chose.  The numbers and the derivation of each live in
    `src/crane_supervisor_parameters.yaml`, once.
    """
    declaration = re.search(
        r"std::array<double,\s*kInputCount>\s*freshness_deadline\s*(\{[^;]*\})?\s*;",
        _code(CORE_HEADER),
    )
    assert declaration, "freshness_deadline is no longer an array sized by the registry"
    assert (declaration.group(1) or "").strip() in ("", "{}"), declaration.group(1)


def test_no_contract_name_is_written_down_twice():
    """A topic that lives in two places can be changed in one of them.

    Each input's contract name is written once, in its policy row, and the node's
    constants alias that row rather than restating it -- so the stream a report
    names is the stream the node subscribes to, by construction.
    """
    standalone = []
    for path in _sources():
        _code_body, literals = _split(path.read_text(encoding="utf-8"))
        standalone += [name for name in literals if ROS_NAME.fullmatch(name)]
    assert len(standalone) == len(set(standalone)), sorted(standalone)


def test_the_node_constants_alias_the_policy_rows_rather_than_restating_them():
    """The four input topics reach the node header off the registry."""
    header = _code(NODE_HEADER)
    for name in _inputs():
        assert f"policy_of(Input::{name}).topic" in header, name

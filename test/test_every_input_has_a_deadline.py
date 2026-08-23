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

* `create_subscription` appears exactly once in the whole package, inside the
  helper.  A second call site could take a topic name of its own, and an input
  with no `Input` behind it has no deadline, no policy row and no report.
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
    """One `create_subscription`, and one `subscribe()` call per input.

    The helper takes an `Input`, so a subscription that named no input -- and
    therefore had no deadline, no policy row and no way to be reported -- cannot
    be written.  A second `create_subscription` anywhere in the package would be
    that subscription, so the count is what is asserted rather than the topic.
    """
    created = []
    for path in _sources():
        created += re.findall(r"create_subscription<([A-Za-z0-9_:]+)>", _code(path))
    assert created == ["MessageT"], created

    subscribed = re.findall(
        r"subscribe<[A-Za-z0-9_:]+>\(\s*Input::([A-Za-z0-9_]+)", _code(NODE_SOURCE)
    )
    assert subscribed == _inputs(), subscribed


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

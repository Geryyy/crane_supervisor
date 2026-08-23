"""Static proof that this package restates no tracking tolerance.

The per-axis velocity-tracking tolerance is one number with four consumers --
the seam clamp in `crane_velocity_controller`, the MPC's constraint margin, the
D4 acceptance measure, and `FAULT_TRACKING` here (PRD 6 and 10).  They are
parameterised out of `crane_control/config/tracking_tolerance.yaml` so that they
cannot drift apart, and the way they drift apart is that one of them keeps a
copy.

So the copy is what is asserted absent.  This package may declare the parameter
-- it has to, to receive it -- but the declared default is NaN, its shipped
configuration names no tolerance at all, and no number that could be mistaken
for one lives under `config/`.  A value is a fabrication until a human has
measured it on the machine, and a fabricated default is worse than the gap.

Static, and stdlib only.  Nothing here starts a node, a launch or a DDS
participant.
"""

import re
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DECLARATION = PACKAGE_ROOT / "src" / "crane_supervisor_parameters.yaml"
SHIPPED_CONFIG = PACKAGE_ROOT / "config" / "crane_supervisor.yaml"
CONFIG_DIRECTORY = PACKAGE_ROOT / "config"

# `dq_a: <number>` with any sign, in any of the shapes a ROS parameter file
# writes one.  The key is the tolerance file's own, so a copy that arrived here
# under a different name would not be the drift this guard is about.
TOLERANCE_VALUE = re.compile(r"dq_a\s*:\s*(-?[0-9.]+(?:[eE][-+]?[0-9]+)?)")


def _uncommented(path):
    """Return the file with `#` comments removed, so prose is not evidence."""
    return "\n".join(
        line.split("#", 1)[0] for line in path.read_text(encoding="utf-8").splitlines()
    )


def test_the_declared_default_is_the_absence_of_a_number():
    """NaN, not zero and not a plausible small value.

    Zero would be a tolerance no axis can meet; a small positive value would be
    a fault raised against a number nobody measured.  NaN is the only default
    that says "there is no number here", and `is_tolerance()` reads it as one.
    """
    body = _uncommented(DECLARATION)
    assert "tracking_tolerance:" in body, "the parameter is no longer declared"

    declaration = body.split("tracking_tolerance:", 1)[1]
    dq_a = declaration.split("dq_a:", 1)[1]
    default = dq_a.split("default_value:", 1)[1].splitlines()[0].strip()
    assert default == ".NAN", default


def test_the_shipped_configuration_states_no_tolerance():
    """The one file a profile loads into this node carries no tolerance.

    `crane_bringup` passes `config/crane_supervisor.yaml`.  A tolerance written
    there would be read in preference to the shared file for this consumer and
    for no other, which is exactly the disagreement the shared file exists to
    make impossible.
    """
    body = _uncommented(SHIPPED_CONFIG)
    assert "tracking_tolerance" not in body, body


def test_no_number_under_config_could_be_mistaken_for_a_tolerance():
    """Not one, anywhere in the installed configuration directory.

    Scanned over the whole directory rather than the one file, because the way
    this comes back is a second profile file added later by someone who found
    the warning at startup inconvenient.  Fixtures live under `test/` and are
    not installed; this scan does not reach them, and their own header says
    outright that they are not tolerances for any machine.
    """
    for path in sorted(CONFIG_DIRECTORY.rglob("*.yaml")):
        found = TOLERANCE_VALUE.findall(_uncommented(path))
        assert not found, f"{path.name}: {found}"

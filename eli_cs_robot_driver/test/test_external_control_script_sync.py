"""
Guard the "must be edited in lockstep" rule for external_control.script.

Two identical copies exist (this driver's resources/, and the Elite SDK's
source/resources/) because the driver sends the former at runtime, but the
SDK ships its own copy too. Nothing enforces they stay identical except
discipline -- this test makes that automatic.
"""

from pathlib import Path

_THIS_FILE = Path(__file__).resolve()
DRIVER_SCRIPT = _THIS_FILE.parents[1] / "resources" / "external_control.script"
SDK_SCRIPT = (
    _THIS_FILE.parents[3] / "Elite_Robots_CS_SDK" / "source" / "resources"
    / "external_control.script"
)


def test_driver_and_sdk_copies_are_byte_identical():
    assert DRIVER_SCRIPT.is_file(), f"expected driver copy at {DRIVER_SCRIPT}"
    assert SDK_SCRIPT.is_file(), f"expected SDK copy at {SDK_SCRIPT}"
    assert DRIVER_SCRIPT.read_bytes() == SDK_SCRIPT.read_bytes(), (
        f"{DRIVER_SCRIPT} and {SDK_SCRIPT} have diverged -- "
        "external_control.script must be edited in lockstep in both locations."
    )

"""
Test harness for external_control.script.

external_control.script is EliteScript: a constrained, Python-like language
with no import mechanism of its own and no way to split across files. This
harness reads the HEADER region (constants and function definitions, between
the "# HEADER_BEGIN"/"# HEADER_END" markers) straight from the real file,
substitutes its "{{...}}" template placeholders the same way EliteDriver.cpp
does for real deployment, and exec()s it into a sandbox namespace with mocked
EliteScript builtins. Tests pull functions out of that namespace and call
them directly.

The NODE_CONTROL_LOOP region (after "# HEADER_END") opens real sockets and
runs an infinite loop immediately on exec(), so it is deliberately never
exec()'d by this harness -- the header/body split is what makes any of this
testable at all.

EXTERNAL_CONTROL_SCRIPT_TESTS.md in this directory explains the testing
strategy and how to extend it.
"""

import re
from pathlib import Path
from unittest.mock import MagicMock

SCRIPT_PATH = Path(__file__).resolve().parent.parent / "resources" / "external_control.script"
HEADER_END_MARKER = "# HEADER_END"

# Mirrors the real substitution EliteDriver.cpp performs before sending the script
# (ControlCommon.hpp:11-13 for the zoom ratios, EliteDriver.cpp's stopj_acc/servoj
# defaults, the *_DATA_SIZE_REPLACE constants from the interface headers, and
# EliteDriverConfig's servoj_start_delta_limit/payload_mass_max defaults).
# Overridable per test via template_overrides.
DEFAULT_TEMPLATE_VALUES = {
    "POS_ZOOM_RATIO_REPLACE": 1000000,
    "TIME_ZOOM_RATIO_REPLACE": 1000,
    "COMMON_ZOOM_RATIO_REPLACE": 1000000,
    "STOP_J_REPLACE": 4,
    "REVERSE_DATA_SIZE_REPLACE": 8,
    "TRAJECTORY_DATA_SIZE_REPLACE": 21,
    "SCRIPT_COMMAND_DATA_SIZE_REPLACE": 26,
    "SERVO_J_REPLACE": "t=0.008, lookahead_time=0.32, gain=1000",
    "SERVER_IP_REPLACE": "127.0.0.1",
    "REVERSE_PORT_REPLACE": 50001,
    "TRAJECTORY_SERVER_PORT_REPLACE": 50003,
    "SCRIPT_COMMAND_PORT_REPLACE": 50004,
    "SERVOJ_START_DELTA_LIMIT_REPLACE": 0.0872665,
    "PAYLOAD_MASS_MAX_REPLACE": 14.4,
}


class ScriptHalted(Exception):
    """
    Raised by the mocked halt().

    Lets a test assert the script halted at a given point without actually
    terminating the test process.
    """


def load_header_source(template_overrides=None):
    """
    Return the HEADER region's source text with every {{KEY}} substituted.

    This is a real text substitution, not pre-defined globals -- "{{KEY}}"
    parses as a nested-set literal in real Python, not a substitution point,
    so it must be replaced before exec(), the same way the real driver does
    it.
    """
    text = SCRIPT_PATH.read_text()
    header_text, _, _ = text.partition(HEADER_END_MARKER)
    values = dict(DEFAULT_TEMPLATE_VALUES)
    if template_overrides:
        values.update(template_overrides)
    for key, value in values.items():
        header_text = header_text.replace("{{" + key + "}}", str(value))
    return header_text


def _make_fake_start_thread():
    """
    Build a fake start_thread() that never runs the target function.

    EliteScript's own start_thread/stop_thread/join_thread concurrency model
    is undocumented anywhere in this repo (preemptive vs. cooperative is
    unknown), so tests never depend on it -- this fake just records the call
    and returns a fake handle. Each fake numbers its handles from 1, so handle
    numbers are predictable within one test.
    """
    counter = {"next": 1}

    def fake_start_thread(fn, args):
        handle = counter["next"]
        counter["next"] += 1
        return handle

    return fake_start_thread


def _fake_norm(value):
    """
    Stand in for EliteScript's norm().

    The script only passes scalars, where norm() is the absolute value.
    """
    return abs(value)


def make_mock_builtins(builtin_overrides=None):
    """
    Build mocks for every EliteScript runtime-injected global the script calls.

    These are not Python's real __builtins__ -- real Python builtins (int,
    str, etc.) remain available automatically since exec() only receives
    this as the globals dict, not a restricted __builtins__.
    """
    builtins_dict = {
        "textmsg": MagicMock(),
        "popup": MagicMock(),
        "halt": MagicMock(side_effect=ScriptHalted),
        "get_actual_joint_positions": MagicMock(return_value=[0.0] * 6),
        "servoj": MagicMock(),
        "stopj": MagicMock(),
        "movej": MagicMock(),
        "movel": MagicMock(),
        "speedl": MagicMock(),
        "speedj": MagicMock(),
        "zero_ftsensor": MagicMock(),
        "set_payload": MagicMock(),
        "set_tool_voltage": MagicMock(),
        "force_mode": MagicMock(),
        "end_force_mode": MagicMock(),
        "freedrive_mode": MagicMock(),
        "end_freedrive_mode": MagicMock(),
        "get_steptime": MagicMock(return_value=0.008),
        "norm": MagicMock(side_effect=_fake_norm),
        "socket_open": MagicMock(),
        "socket_close": MagicMock(),
        "socket_read_binary_integer": MagicMock(return_value=[0]),
        "socket_send_int": MagicMock(),
        "start_thread": MagicMock(side_effect=_make_fake_start_thread()),
        "stop_thread": MagicMock(),
        "join_thread": MagicMock(),
    }
    if builtin_overrides:
        builtins_dict.update(builtin_overrides)
    return builtins_dict


# --- Host-side contract, read from the Elite SDK sources ---
#
# The enum values below come straight from the SDK headers, not from the
# script. Tests that build packets with them therefore check the script
# against what the host really sends.

SDK_ROOT = Path(__file__).resolve().parents[3] / "Elite_Robots_CS_SDK"


def parse_sdk_enum(header_relpath, enum_name):
    """Return {member: value} for an `enum class` in an Elite SDK header."""
    text = (SDK_ROOT / header_relpath).read_text()
    match = re.search(r"enum class " + enum_name + r"\b[^{]*\{(.*?)\};", text, re.S)
    if match is None:
        raise LookupError(f"enum class {enum_name} not found in {header_relpath}")
    body = re.sub(r"//[^\n]*", "", match.group(1))
    return {name: int(value) for name, value in re.findall(r"(\w+)\s*=\s*(-?\d+)", body)}


CONTROL_MODE = parse_sdk_enum("include/Elite/ControlMode.hpp", "ControlMode")
TRAJECTORY_CONTROL_ACTION = parse_sdk_enum("include/Elite/DataType.hpp", "TrajectoryControlAction")
TRAJECTORY_MOTION_RESULT = parse_sdk_enum("include/Elite/DataType.hpp", "TrajectoryMotionResult")
FREEDRIVE_ACTION = parse_sdk_enum("include/Elite/DataType.hpp", "FreedriveAction")
TOOL_VOLTAGE = parse_sdk_enum("include/Elite/DataType.hpp", "ToolVoltage")
TRAJECTORY_MOTION_TYPE = parse_sdk_enum(
    "include/Control/TrajectoryInterface.hpp", "TrajectoryMotionType")
SCRIPT_COMMAND = parse_sdk_enum("include/Control/ScriptCommandInterface.hpp", "Cmd")


# --- Wire-format packet builders ---
#
# Each builder mirrors one Elite SDK writer word for word, so tests feed the
# script exactly what the host puts on the socket. The script's
# socket_read_binary_integer() result prepends the count of integers read,
# which shifts every host index up by one. The builders include that count.

def _read_result(words):
    return [len(words)] + list(words)


def _scaled(value, ratio):
    # The reverse and trajectory writers scale with round().
    return int(round(value * ratio))


def _truncated(value, ratio):
    # The script-command writers scale with static_cast, which truncates.
    return int(value * ratio)


def _reverse_words():
    return [0] * DEFAULT_TEMPLATE_VALUES["REVERSE_DATA_SIZE_REPLACE"]


def _reverse_joint_command(mode_name, values, timeout_ms):
    # ReverseInterface::writeJointCommand() puts the timeout in data[0], the
    # scaled values in data[1..6], and the mode in the last word.
    ratio = DEFAULT_TEMPLATE_VALUES["POS_ZOOM_RATIO_REPLACE"]
    data = _reverse_words()
    data[0] = timeout_ms
    data[-1] = CONTROL_MODE[mode_name]
    if values is not None:
        for i in range(6):
            data[i + 1] = _scaled(values[i], ratio)
    return _read_result(data)


def reverse_servoj(positions, timeout_ms=200):
    """Build a reverse-port read for EliteDriver::writeServoj()."""
    return _reverse_joint_command("MODE_SERVOJ", positions, timeout_ms)


def reverse_speedj(velocities, timeout_ms=200):
    """Build a reverse-port read for EliteDriver::writeSpeedj()."""
    return _reverse_joint_command("MODE_SPEEDJ", velocities, timeout_ms)


def reverse_speedl(velocities, timeout_ms=200):
    """Build a reverse-port read for EliteDriver::writeSpeedl()."""
    return _reverse_joint_command("MODE_SPEEDL", velocities, timeout_ms)


def reverse_idle(timeout_ms=200):
    """Build a reverse-port read for EliteDriver::writeIdle()."""
    return _reverse_joint_command("MODE_IDLE", None, timeout_ms)


def reverse_trajectory_control(action_name, point_number, timeout_ms=200):
    """Build a reverse-port read for ReverseInterface::writeTrajectoryControlAction()."""
    data = _reverse_words()
    data[0] = timeout_ms
    data[1] = TRAJECTORY_CONTROL_ACTION[action_name]
    data[2] = point_number
    data[-1] = CONTROL_MODE["MODE_TRAJECTORY"]
    return _read_result(data)


def reverse_freedrive(action_name, timeout_ms=200):
    """Build a reverse-port read for ReverseInterface::writeFreedrive()."""
    data = _reverse_words()
    data[0] = timeout_ms
    data[1] = FREEDRIVE_ACTION[action_name]
    data[-1] = CONTROL_MODE["MODE_FREEDRIVE"]
    return _read_result(data)


def reverse_stop_control():
    """Build a reverse-port read for ReverseInterface::stopControl()."""
    data = _reverse_words()
    data[-1] = CONTROL_MODE["MODE_STOPPED"]
    return _read_result(data)


def trajectory_point(positions, time, blend_radius, cartesian=False):
    """Build a trajectory-port read for TrajectoryInterface::writeTrajectoryPoint()."""
    pos_ratio = DEFAULT_TEMPLATE_VALUES["POS_ZOOM_RATIO_REPLACE"]
    time_ratio = DEFAULT_TEMPLATE_VALUES["TIME_ZOOM_RATIO_REPLACE"]
    data = [0] * DEFAULT_TEMPLATE_VALUES["TRAJECTORY_DATA_SIZE_REPLACE"]
    for i in range(6):
        data[i] = _scaled(positions[i], pos_ratio)
    data[18] = _scaled(time, time_ratio)
    data[19] = _scaled(blend_radius, pos_ratio)
    data[20] = TRAJECTORY_MOTION_TYPE["CARTESIAN" if cartesian else "JOINT"]
    return _read_result(data)


def _script_command(cmd_name, words):
    data = [0] * DEFAULT_TEMPLATE_VALUES["SCRIPT_COMMAND_DATA_SIZE_REPLACE"]
    data[0] = SCRIPT_COMMAND[cmd_name]
    data[1:1 + len(words)] = words
    return _read_result(data)


def script_command_zero_ftsensor():
    """Build a script-command read for ScriptCommandInterface::zeroFTSensor()."""
    return _script_command("ZERO_FTSENSOR", [])


def script_command_set_payload(mass, cog):
    """Build a script-command read for ScriptCommandInterface::setPayload()."""
    ratio = DEFAULT_TEMPLATE_VALUES["COMMON_ZOOM_RATIO_REPLACE"]
    words = [_truncated(mass, ratio)] + [_truncated(c, ratio) for c in cog]
    return _script_command("SET_PAYLOAD", words)


def script_command_set_tool_voltage(voltage):
    """
    Build a script-command read for ScriptCommandInterface::setToolVoltage().

    The host only sends ToolVoltage enum values. Tests may pass any integer
    to model a corrupt packet.
    """
    ratio = DEFAULT_TEMPLATE_VALUES["COMMON_ZOOM_RATIO_REPLACE"]
    return _script_command("SET_TOOL_VOLTAGE", [voltage * ratio])


def script_command_start_force_mode(task_frame, selection_vector, wrench, mode, limits):
    """
    Build a script-command read for ScriptCommandInterface::startForceMode().

    The selection vector and mode travel unscaled. Every other field is scaled.
    """
    ratio = DEFAULT_TEMPLATE_VALUES["COMMON_ZOOM_RATIO_REPLACE"]
    words = [_truncated(v, ratio) for v in task_frame]
    words += list(selection_vector)
    words += [_truncated(v, ratio) for v in wrench]
    words += [mode]
    words += [_truncated(v, ratio) for v in limits]
    return _script_command("START_FORCE_MODE", words)


def script_command_end_force_mode():
    """Build a script-command read for ScriptCommandInterface::endForceMode()."""
    return _script_command("END_FORCE_MODE", [])


def load_control_script_namespace(template_overrides=None, builtin_overrides=None):
    """
    Build a fresh sandbox namespace by exec()ing the real HEADER region.

    Returns the namespace dict -- tests pull functions/constants/globals out
    of it directly, e.g. namespace["servoThread"](), namespace["MODE_SERVOJ"].
    """
    source = load_header_source(template_overrides)
    namespace = {}
    namespace.update(make_mock_builtins(builtin_overrides))
    exec(compile(source, str(SCRIPT_PATH), "exec"), namespace)
    # The header's top-level textmsg("ExternalControl: Script started") fires
    # on every exec(). Reset so tests only see calls made during the test itself.
    namespace["textmsg"].reset_mock()
    return namespace

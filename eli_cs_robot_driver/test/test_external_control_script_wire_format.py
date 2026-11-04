"""
Contract tests between external_control.script and the Elite SDK host side.

The first group checks that the script's constants, template placeholders,
and data sizes agree with the SDK sources. The rest feed the script packets
built word for word like the SDK writers, one socket at a time, and check
what the script does with them.
"""

import re
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from external_control_script_harness import (
    CONTROL_MODE,
    DEFAULT_TEMPLATE_VALUES,
    FREEDRIVE_ACTION,
    load_control_script_namespace,
    reverse_freedrive,
    reverse_idle,
    reverse_servoj,
    reverse_speedj,
    reverse_speedl,
    reverse_stop_control,
    reverse_trajectory_control,
    script_command_end_force_mode,
    script_command_set_payload,
    script_command_set_tool_voltage,
    script_command_start_force_mode,
    script_command_zero_ftsensor,
    SCRIPT_COMMAND,
    SCRIPT_PATH,
    SDK_ROOT,
    TOOL_VOLTAGE,
    TRAJECTORY_CONTROL_ACTION,
    TRAJECTORY_MOTION_RESULT,
    TRAJECTORY_MOTION_TYPE,
    trajectory_point,
)

DESCRIPTION_URDF = Path(__file__).resolve().parents[2] / "eli_cs_robot_description" / "urdf"


def _sdk_text(relpath):
    return (SDK_ROOT / relpath).read_text()


def _sdk_int_constant(relpath, name):
    match = re.search(name + r"\s*=\s*(\d+)\s*;", _sdk_text(relpath))
    assert match is not None, f"{name} not found in {relpath}"
    return int(match.group(1))


def _sdk_float_default(relpath, name):
    match = re.search(name + r"\s*=\s*([0-9.]+)\s*;", _sdk_text(relpath))
    assert match is not None, f"{name} not found in {relpath}"
    return float(match.group(1))


def _xacro_default(filename, name):
    text = (DESCRIPTION_URDF / filename).read_text()
    match = re.search(name + r":=([0-9.]+)", text)
    assert match is not None, f"{name} default not found in {filename}"
    return float(match.group(1))


# --- Static contract with the SDK sources ---


@pytest.mark.parametrize(
    "script_prefix, sdk_enum, name_map",
    [
        ("", CONTROL_MODE, None),
        ("TRAJECTORY_RESULT_", TRAJECTORY_MOTION_RESULT, None),
        ("TRAJECTORY_ACTION_", TRAJECTORY_CONTROL_ACTION, None),
        ("TRAJECTORY_MOTION_", TRAJECTORY_MOTION_TYPE,
         {"JOINT": "JOINT", "CARTESIAN": "CARTESIAN"}),
        ("SCRIPT_CMD_", SCRIPT_COMMAND, None),
        ("", FREEDRIVE_ACTION, None),
        ("TOOL_VOLTAGE_", TOOL_VOLTAGE, {"OFF": "OFF", "V_12": "12V", "V_24": "24V"}),
    ],
)
def test_script_constants_match_sdk_enums(script_prefix, sdk_enum, name_map):
    namespace = load_control_script_namespace()
    members = name_map or {name: name for name in sdk_enum}
    for sdk_name, script_suffix in members.items():
        script_name = script_prefix + script_suffix
        assert namespace[script_name] == sdk_enum[sdk_name], script_name


def test_script_placeholders_match_elite_driver_substitutions():
    script_keys = set(re.findall(r"\{\{(\w+)\}\}", SCRIPT_PATH.read_text()))
    driver_keys = set(re.findall(r'"\{\{(\w+)\}\}"', _sdk_text("source/Elite/EliteDriver.cpp")))
    assert script_keys == driver_keys
    assert set(DEFAULT_TEMPLATE_VALUES) == driver_keys


def test_data_sizes_and_zoom_ratios_match_sdk_headers():
    expected = {
        "REVERSE_DATA_SIZE_REPLACE": _sdk_int_constant(
            "include/Control/ReverseInterface.hpp", "REVERSE_DATA_SIZE"),
        "TRAJECTORY_DATA_SIZE_REPLACE": _sdk_int_constant(
            "include/Control/TrajectoryInterface.hpp", "TRAJECTORY_MESSAGE_LEN"),
        "SCRIPT_COMMAND_DATA_SIZE_REPLACE": _sdk_int_constant(
            "include/Control/ScriptCommandInterface.hpp", "SCRIPT_COMMAND_DATA_SIZE"),
        "POS_ZOOM_RATIO_REPLACE": _sdk_int_constant(
            "include/Control/ControlCommon.hpp", "POS_ZOOM_RATIO"),
        "TIME_ZOOM_RATIO_REPLACE": _sdk_int_constant(
            "include/Control/ControlCommon.hpp", "TIME_ZOOM_RATIO"),
        "COMMON_ZOOM_RATIO_REPLACE": _sdk_int_constant(
            "include/Control/ControlCommon.hpp", "COMMON_ZOOM_RATIO"),
    }
    for key, value in expected.items():
        assert DEFAULT_TEMPLATE_VALUES[key] == value, key


@pytest.mark.parametrize("field", ["servoj_start_delta_limit", "payload_mass_max"])
def test_script_limit_defaults_agree_across_sdk_xacro_and_harness(field):
    sdk_default = _sdk_float_default("include/Elite/EliteDriver.hpp", field)
    harness_default = DEFAULT_TEMPLATE_VALUES[field.upper() + "_REPLACE"]
    assert harness_default == pytest.approx(sdk_default)
    for xacro in ["cs.ros2_control.xacro", "cs_macro.xacro"]:
        assert _xacro_default(xacro, field) == pytest.approx(sdk_default), xacro


# --- Reverse port ---


def _fresh_script():
    namespace = load_control_script_namespace()
    namespace["initializeControlState"]()
    return namespace


def _feed_reverse(namespace, *reads):
    namespace["socket_read_binary_integer"].side_effect = list(reads)
    for _ in reads:
        namespace["runControlLoopIteration"]()


def test_reverse_read_timeout_follows_the_host_packet():
    namespace = _fresh_script()
    assert namespace["read_timeout"] == 0.0

    _feed_reverse(namespace, reverse_idle(timeout_ms=200))

    assert namespace["read_timeout"] == pytest.approx(0.2)


def test_reverse_idle_from_servoj_stops_the_servo_thread():
    namespace = _fresh_script()
    _feed_reverse(namespace, reverse_servoj([0.0] * 6))
    servo_handle = namespace["move_thread_handle"]

    _feed_reverse(namespace, reverse_idle())

    assert namespace["control_mode"] == namespace["MODE_IDLE"]
    namespace["stop_thread"].assert_called_with(servo_handle)
    assert namespace["move_thread_handle"] == 0
    namespace["stopj"].assert_called_with(namespace["STOPJ_ACCELERATION"])


@pytest.mark.parametrize(
    "builder, mode_name, thread_name, command_name",
    [
        (reverse_speedj, "MODE_SPEEDJ", "speedjThread", "cmd_speed_joint"),
        (reverse_speedl, "MODE_SPEEDL", "speedlThread", "cmd_speedl_tool_speed"),
    ],
)
def test_reverse_velocity_command_starts_its_thread_and_decodes(
        builder, mode_name, thread_name, command_name):
    namespace = _fresh_script()
    velocities = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]

    _feed_reverse(namespace, builder(velocities))

    assert namespace["control_mode"] == namespace[mode_name]
    called_fn, _ = namespace["start_thread"].call_args.args
    assert called_fn is namespace[thread_name]
    assert namespace[command_name] == pytest.approx(velocities)


@pytest.mark.parametrize(
    "mode_name, command_name",
    [("MODE_SPEEDJ", "cmd_speed_joint"), ("MODE_SPEEDL", "cmd_speedl_tool_speed")],
)
def test_entering_a_velocity_mode_starts_at_rest(mode_name, command_name):
    """
    Guard the velocity analogue of the servoj stale-command hazard.

    The new thread may run before the first command of the session lands.
    It must not replay the previous session's last velocity meanwhile.
    """
    namespace = _fresh_script()
    namespace[command_name] = [9.0] * 6

    namespace["handleControlModeSwitch"](namespace[mode_name])

    assert namespace[command_name] == [0, 0, 0, 0, 0, 0]


@pytest.mark.parametrize(
    "iteration_name, builtin_name, command_name",
    [
        ("runSpeedjIteration", "speedj", "cmd_speed_joint"),
        ("runSpeedlIteration", "speedl", "cmd_speedl_tool_speed"),
    ],
)
def test_velocity_iteration_sends_the_shared_command(iteration_name, builtin_name, command_name):
    namespace = _fresh_script()
    namespace[command_name] = [0.1] * 6

    namespace[iteration_name]()

    namespace[builtin_name].assert_called_once_with([0.1] * 6, 40.0, namespace["steptime"])


@pytest.mark.parametrize(
    "function_name, args",
    [
        ("setSpeedj", ([0.1] * 6,)),
        ("setSpeedl", ([0.1] * 6,)),
        ("runSpeedjIteration", ()),
        ("runSpeedlIteration", ()),
    ],
)
def test_velocity_commands_cross_threads_under_the_speed_mutex(function_name, args):
    namespace = _fresh_script()
    mutex = MagicMock()
    namespace["SPEED_DATA_MUTEX"] = mutex

    namespace[function_name](*args)

    mutex.__enter__.assert_called_once()
    mutex.__exit__.assert_called_once()


def test_reverse_stop_control_stops_the_script():
    namespace = _fresh_script()

    _feed_reverse(namespace, reverse_stop_control())

    assert namespace["control_mode"] == namespace["MODE_STOPPED"]
    namespace["textmsg"].assert_any_call("ExternalControl: Script stopped")


def test_freedrive_start_noop_end_then_idle_ends_freedrive_once():
    namespace = _fresh_script()

    _feed_reverse(
        namespace,
        reverse_idle(),
        reverse_freedrive("FREEDRIVE_START"),
        reverse_freedrive("FREEDRIVE_NOOP"),
        reverse_freedrive("FREEDRIVE_END"),
        reverse_idle(),
    )

    namespace["freedrive_mode"].assert_called_once()
    namespace["end_freedrive_mode"].assert_called_once()
    assert namespace["freedrive_active"] is False
    assert namespace["control_mode"] == namespace["MODE_IDLE"]


def test_leaving_freedrive_without_end_still_ends_freedrive():
    namespace = _fresh_script()

    _feed_reverse(namespace, reverse_freedrive("FREEDRIVE_START"), reverse_stop_control())

    namespace["end_freedrive_mode"].assert_called_once()
    assert namespace["control_mode"] == namespace["MODE_STOPPED"]


def test_reverse_trajectory_start_arms_the_trajectory_thread():
    namespace = _fresh_script()

    _feed_reverse(namespace, reverse_trajectory_control("START", 3))

    assert namespace["control_mode"] == namespace["MODE_TRAJECTORY"]
    assert namespace["trajectory_point_num"] == 3
    called_fn, _ = namespace["start_thread"].call_args.args
    assert called_fn is namespace["trajectoryThread"]


def test_reverse_trajectory_noop_leaves_the_trajectory_running():
    namespace = _fresh_script()
    _feed_reverse(namespace, reverse_trajectory_control("START", 3))
    trajectory_handle = namespace["trajectory_thread_handle"]
    namespace["stop_thread"].reset_mock()

    _feed_reverse(namespace, reverse_trajectory_control("NOOP", 0))

    assert namespace["trajectory_thread_handle"] == trajectory_handle
    assert namespace["trajectory_point_num"] == 3
    namespace["stop_thread"].assert_not_called()
    namespace["socket_send_int"].assert_not_called()


def test_reverse_trajectory_cancel_reports_canceled():
    namespace = _fresh_script()
    _feed_reverse(namespace, reverse_trajectory_control("START", 3))
    trajectory_handle = namespace["trajectory_thread_handle"]

    # The three unread points sit buffered on the trajectory socket and drain at once
    namespace["socket_read_binary_integer"].side_effect = (
        [reverse_trajectory_control("CANCEL", 0)] + [trajectory_point([0.0] * 6, 1.0, 0.0)] * 3)
    namespace["runControlLoopIteration"]()

    namespace["stop_thread"].assert_called_with(trajectory_handle)
    namespace["socket_send_int"].assert_called_once_with(
        TRAJECTORY_MOTION_RESULT["CANCELED"], "trajectory_socket")
    assert namespace["trajectory_point_num"] == 0
    assert namespace["trajectory_thread_handle"] == 0


# --- Trajectory port ---


def _trajectory_script(point_num, reads):
    namespace = _fresh_script()
    namespace["trajectory_point_num"] = point_num
    namespace["socket_read_binary_integer"].side_effect = list(reads)
    return namespace


def test_trajectory_points_decode_into_movej_and_movel():
    joint_target = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    pose_target = [0.4, -0.1, 0.3, 3.14, 0.0, 1.57]
    namespace = _trajectory_script(2, [
        trajectory_point(joint_target, 2.5, 0.05),
        trajectory_point(pose_target, 1.5, 0.05, cartesian=True),
    ])

    namespace["trajectoryThread"]()

    movej_args = namespace["movej"].call_args
    assert movej_args.args[0] == pytest.approx(joint_target)
    assert movej_args.kwargs["t"] == pytest.approx(2.5)
    assert movej_args.kwargs["r"] == pytest.approx(0.05)
    movel_args = namespace["movel"].call_args
    assert movel_args.args[0] == pytest.approx(pose_target)
    assert movel_args.kwargs["t"] == pytest.approx(1.5)
    # The last point always stops exactly, whatever blend the host sent
    assert movel_args.kwargs["r"] == 0
    namespace["socket_send_int"].assert_called_once_with(
        TRAJECTORY_MOTION_RESULT["SUCCESS"], "trajectory_socket")


def test_trajectory_point_with_unsupported_motion_type_is_skipped_and_logged():
    point = trajectory_point([0.0] * 6, 1.0, 0.0)
    point[21] = TRAJECTORY_MOTION_TYPE["SPLINE"]
    namespace = _trajectory_script(1, [point])

    namespace["runTrajectoryIteration"]()

    namespace["movej"].assert_not_called()
    namespace["movel"].assert_not_called()
    assert "unsupported motion type" in namespace["textmsg"].call_args.args[0]
    assert namespace["trajectory_point_num"] == 0


def test_trajectory_failed_read_consumes_no_point():
    namespace = _trajectory_script(2, [[0]])
    namespace["trajectory_result"] = TRAJECTORY_MOTION_RESULT["SUCCESS"]
    namespace["trajectory_point_wait"] = 0.0

    namespace["runTrajectoryIteration"]()

    assert namespace["trajectory_point_num"] == 2
    namespace["movej"].assert_not_called()
    assert namespace["trajectory_result"] == TRAJECTORY_MOTION_RESULT["SUCCESS"]


def test_trajectory_late_point_after_short_silence_still_succeeds():
    misses = [[0]] * 10
    namespace = _trajectory_script(1, misses + [trajectory_point([0.1] * 6, 1.0, 0.0)])

    namespace["trajectoryThread"]()

    namespace["movej"].assert_called_once()
    namespace["socket_send_int"].assert_called_once_with(
        TRAJECTORY_MOTION_RESULT["SUCCESS"], "trajectory_socket")


def test_trajectory_silence_past_the_timeout_reports_failure():
    namespace = _fresh_script()
    namespace["trajectory_point_num"] = 3
    # A finite supply turns a missing timeout into a failure instead of a hang
    namespace["socket_read_binary_integer"].side_effect = [[0]] * 1000

    namespace["trajectoryThread"]()

    steptime = namespace["get_steptime"].return_value
    expected_reads = int(namespace["TRAJECTORY_POINT_TIMEOUT"] / steptime) + 1
    read_count = namespace["socket_read_binary_integer"].call_count
    assert read_count == pytest.approx(expected_reads, abs=1)
    namespace["socket_send_int"].assert_called_once_with(
        TRAJECTORY_MOTION_RESULT["FAILURE"], "trajectory_socket")
    # The rest of the trajectory is abandoned, and any blended move is stopped
    assert namespace["trajectory_point_num"] == 0
    namespace["stopj"].assert_called_once_with(namespace["STOPJ_ACCELERATION"])
    assert namespace["trajectory_active"] is False


def test_start_after_a_timed_out_trajectory_keeps_every_new_point():
    """
    Guard review finding #1, a leftover count eating the next trajectory.

    Trajectory A times out with points left. Trajectory B's START must drain
    none of B's own points, so the arm runs B from its first waypoint.
    """
    namespace = _fresh_script()
    _feed_reverse(namespace, reverse_trajectory_control("START", 3))
    namespace["socket_read_binary_integer"].side_effect = [[0]] * 1000
    namespace["trajectoryThread"]()
    namespace["socket_read_binary_integer"].reset_mock()

    b_targets = [[0.1] * 6, [0.2] * 6]
    namespace["socket_read_binary_integer"].side_effect = (
        [reverse_trajectory_control("START", 2)]
        + [trajectory_point(t, 1.0, 0.0) for t in b_targets])
    namespace["runControlLoopIteration"]()
    assert namespace["trajectory_point_num"] == 2
    namespace["trajectoryThread"]()

    moved_to = [c.args[0] for c in namespace["movej"].call_args_list]
    assert moved_to == [pytest.approx(t) for t in b_targets]


@pytest.mark.parametrize("finished_with", ["SUCCESS", "FAILURE"])
def test_leaving_trajectory_mode_after_a_result_reports_nothing_more(finished_with):
    """
    Guard review finding #3, two results for one trajectory.

    A trajectory that already reported its result must not also be reported
    as CANCELED when the script leaves MODE_TRAJECTORY.
    """
    namespace = _fresh_script()
    _feed_reverse(namespace, reverse_trajectory_control("START", 1))
    if finished_with == "SUCCESS":
        reads = [trajectory_point([0.1] * 6, 1.0, 0.0)]
    else:
        reads = [[0]] * 1000
    namespace["socket_read_binary_integer"].side_effect = reads
    namespace["trajectoryThread"]()

    namespace["get_actual_joint_positions"].return_value = [0.0] * 6
    namespace["socket_read_binary_integer"].side_effect = [reverse_servoj([0.0] * 6)]
    namespace["runControlLoopIteration"]()

    namespace["socket_send_int"].assert_called_once_with(
        TRAJECTORY_MOTION_RESULT[finished_with], "trajectory_socket")


def test_leaving_trajectory_mode_without_a_start_reports_nothing():
    namespace = _fresh_script()
    _feed_reverse(namespace, reverse_trajectory_control("NOOP", 0), reverse_idle())

    namespace["socket_send_int"].assert_not_called()


# --- Script command port ---


def _run_script_command(packet):
    namespace = _fresh_script()
    namespace["socket_read_binary_integer"].return_value = packet
    namespace["textmsg"].reset_mock()
    namespace["runScriptCommandIteration"]()
    return namespace


def test_script_command_zero_ftsensor():
    namespace = _run_script_command(script_command_zero_ftsensor())
    namespace["zero_ftsensor"].assert_called_once()


def test_script_command_set_payload_decodes_mass_and_cog():
    namespace = _run_script_command(script_command_set_payload(2.5, [0.01, -0.02, 0.15]))

    mass, cog = namespace["set_payload"].call_args.args
    assert mass == pytest.approx(2.5)
    assert cog == pytest.approx([0.01, -0.02, 0.15])
    namespace["textmsg"].assert_called_once()


@pytest.mark.parametrize("voltage", sorted(TOOL_VOLTAGE.values()))
def test_script_command_tool_voltage_accepts_every_sdk_value(voltage):
    namespace = _run_script_command(script_command_set_tool_voltage(voltage))
    namespace["set_tool_voltage"].assert_called_once_with(voltage)


@pytest.mark.parametrize("voltage", [5, -12, 48])
def test_script_command_tool_voltage_refuses_and_logs_other_values(voltage):
    namespace = _run_script_command(script_command_set_tool_voltage(voltage))

    namespace["set_tool_voltage"].assert_not_called()
    assert "tool voltage refused" in namespace["textmsg"].call_args.args[0]


def test_script_command_start_force_mode_decodes_every_field():
    task_frame = [0.1, 0.2, 0.3, 0.0, 0.0, 1.57]
    # Every axis selected, so scaling any element would show
    selection_vector = [1, 1, 1, 1, 1, 1]
    wrench = [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]
    mode = 2
    limits = [0.1, 0.1, 0.1, 0.17, 0.17, 0.17]

    namespace = _run_script_command(script_command_start_force_mode(
        task_frame, selection_vector, wrench, mode, limits))

    args = namespace["force_mode"].call_args.args
    assert args[0] == pytest.approx(task_frame)
    assert args[1] == selection_vector
    assert args[2] == pytest.approx(wrench)
    assert args[3] == mode
    assert args[4] == pytest.approx(limits)


def test_script_command_end_force_mode():
    namespace = _run_script_command(script_command_end_force_mode())
    namespace["end_force_mode"].assert_called_once()


_SCRIPT_COMMAND_BUILTINS = [
    "zero_ftsensor", "set_payload", "set_tool_voltage", "force_mode", "end_force_mode"]


def test_script_command_failed_read_does_nothing():
    namespace = _run_script_command([0])
    for builtin in _SCRIPT_COMMAND_BUILTINS:
        namespace[builtin].assert_not_called()


def test_script_command_unknown_id_does_nothing():
    packet = script_command_zero_ftsensor()
    packet[1] = max(SCRIPT_COMMAND.values()) + 1
    namespace = _run_script_command(packet)
    for builtin in _SCRIPT_COMMAND_BUILTINS:
        namespace[builtin].assert_not_called()

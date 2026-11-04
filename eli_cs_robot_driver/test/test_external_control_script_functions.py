"""
Tests for external_control.script's functions.

These began as characterization tests, a regression gate for the
loop-body-extraction refactor. They now pin the script's intended behavior,
function by function, and check the script statically. The wire-format tests
drive the same functions with host-built packets.
"""

import ast
import inspect
import threading
from unittest.mock import call, MagicMock

import pytest

from external_control_script_harness import (
    DEFAULT_TEMPLATE_VALUES,
    HEADER_END_MARKER,
    load_control_script_namespace,
    load_header_source,
    make_mock_builtins,
    reverse_idle,
    reverse_servoj,
    SCRIPT_PATH,
    script_command_set_payload,
    trajectory_point,
    TRAJECTORY_MOTION_RESULT,
)


def _stop_after(namespace, mock):
    """
    Make `mock` end a `while control_mode == ...:` loop after one iteration.

    Gives `mock` a side_effect that drops namespace["control_mode"] to
    MODE_STOPPED on its first call, so the loop condition fails on the next
    check.
    """

    def side_effect(*args, **kwargs):
        namespace["control_mode"] = namespace["MODE_STOPPED"]

    mock.side_effect = side_effect


# --- Motion threads ---


def test_servo_thread_sends_cmd_servo_joints_without_reading_actual():
    """
    Pin that each servo cycle sends the shared setpoint and nothing else.

    The start-target check lives in applyModeCommand(), so the servo thread
    never needs the actual position.
    """
    namespace = load_control_script_namespace()
    namespace["control_mode"] = namespace["MODE_SERVOJ"]
    cmd_servo_joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    namespace["cmd_servo_joints"] = cmd_servo_joints
    _stop_after(namespace, namespace["servoj"])

    namespace["servoThread"]()

    namespace["servoj"].assert_called_once()
    assert namespace["servoj"].call_args.args[0] == cmd_servo_joints
    namespace["get_actual_joint_positions"].assert_not_called()
    namespace["stopj"].assert_called_once_with(namespace["STOPJ_ACCELERATION"])


@pytest.mark.parametrize(
    "thread_name, mode_name, builtin_name",
    [("speedjThread", "MODE_SPEEDJ", "speedj"), ("speedlThread", "MODE_SPEEDL", "speedl")],
)
def test_velocity_thread_runs_while_its_mode_holds_then_stops(
        thread_name, mode_name, builtin_name):
    namespace = load_control_script_namespace()
    namespace["initializeControlState"]()
    namespace["control_mode"] = namespace[mode_name]
    _stop_after(namespace, namespace[builtin_name])

    namespace[thread_name]()

    namespace[builtin_name].assert_called_once()
    namespace["stopj"].assert_called_once_with(namespace["STOPJ_ACCELERATION"])


def test_set_servo_setpoint_writes_cmd_servo_joints():
    namespace = load_control_script_namespace()
    namespace["setServoSetpoint"]([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    assert namespace["cmd_servo_joints"] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


@pytest.mark.parametrize("mutex_name", ["SERVOJ_DATA_MUTEX", "SPEED_DATA_MUTEX"])
def test_data_mutexes_are_real_locks(mutex_name):
    namespace = load_control_script_namespace()
    assert isinstance(namespace[mutex_name], type(threading.Lock()))


# --- Trajectory drain and cancel ---


def test_trajectory_clear_points_drains_exact_count():
    namespace = load_control_script_namespace()
    namespace["trajectory_point_num"] = 3
    namespace["socket_read_binary_integer"].return_value = trajectory_point([0.0] * 6, 1.0, 0.0)
    namespace["trajectoryClearPoints"]()
    assert namespace["socket_read_binary_integer"].call_count == 3
    assert namespace["trajectory_point_num"] == 0


def test_trajectory_clear_points_waits_for_a_late_point():
    namespace = load_control_script_namespace()
    namespace["trajectory_point_num"] = 2
    point = trajectory_point([0.0] * 6, 1.0, 0.0)
    # Ten empty reads are about 80 ms of silence, well inside the budget
    namespace["socket_read_binary_integer"].side_effect = [point] + [[0]] * 10 + [point]

    namespace["trajectoryClearPoints"]()

    assert namespace["socket_read_binary_integer"].call_count == 12
    assert namespace["trajectory_point_num"] == 0


def test_trajectory_clear_points_budget_restarts_after_each_point():
    """
    Pin that the silence budget is per gap, not per drain.

    Each gap below is under TRAJECTORY_POINT_TIMEOUT, but together they exceed it.
    Every point must still drain.
    """
    namespace = load_control_script_namespace()
    namespace["trajectory_point_num"] = 3
    point = trajectory_point([0.0] * 6, 1.0, 0.0)
    gap = [[0]] * 100  # 0.8 s at the mocked 8 ms steptime
    namespace["socket_read_binary_integer"].side_effect = [point] + gap + [point] + gap + [point]

    namespace["trajectoryClearPoints"]()

    assert namespace["socket_read_binary_integer"].call_count == 203
    assert namespace["trajectory_point_num"] == 0


def test_trajectory_clear_points_gives_up_after_the_silence_budget():
    namespace = load_control_script_namespace()
    namespace["trajectory_point_num"] = 50
    # A finite supply turns a missing budget into a failure instead of a hang
    namespace["socket_read_binary_integer"].side_effect = (
        [trajectory_point([0.0] * 6, 1.0, 0.0)] + [[0]] * 1000)

    namespace["trajectoryClearPoints"]()

    steptime = namespace["get_steptime"].return_value
    empty_reads = int(namespace["TRAJECTORY_POINT_TIMEOUT"] / steptime) + 1
    read_count = namespace["socket_read_binary_integer"].call_count
    assert read_count == pytest.approx(1 + empty_reads, abs=1)
    assert namespace["trajectory_point_num"] == 0
    # Every drain read carries a timeout, so a missing point cannot block forever
    for read in namespace["socket_read_binary_integer"].call_args_list:
        assert len(read.args) == 3


def test_trajectory_clear_points_noop_when_already_empty():
    namespace = load_control_script_namespace()
    namespace["trajectory_point_num"] = 0
    namespace["trajectoryClearPoints"]()
    namespace["socket_read_binary_integer"].assert_not_called()


def test_cancel_trajectory_stops_motion_before_draining_and_reports_last():
    namespace = load_control_script_namespace()
    manager = MagicMock()
    for name in ["stop_thread", "stopj", "socket_read_binary_integer", "socket_send_int"]:
        namespace[name] = getattr(manager, name)
    manager.socket_read_binary_integer.return_value = trajectory_point([0.0] * 6, 1.0, 0.0)
    namespace["trajectory_thread_handle"] = 7
    namespace["trajectory_point_num"] = 1
    namespace["trajectory_active"] = True

    namespace["cancelTrajectory"]()

    assert [c[0] for c in manager.mock_calls] == [
        "stop_thread", "stopj", "socket_read_binary_integer", "socket_send_int"]
    manager.stop_thread.assert_called_once_with(7)
    manager.socket_send_int.assert_called_once_with(
        TRAJECTORY_MOTION_RESULT["CANCELED"], "trajectory_socket")
    assert namespace["trajectory_thread_handle"] == 0
    assert namespace["trajectory_point_num"] == 0
    assert namespace["trajectory_active"] is False


def test_cancel_trajectory_reports_nothing_for_a_finished_trajectory():
    namespace = load_control_script_namespace()
    namespace["trajectory_thread_handle"] = 7
    namespace["trajectory_point_num"] = 0
    namespace["trajectory_active"] = False

    namespace["cancelTrajectory"]()

    namespace["stop_thread"].assert_called_once_with(7)
    namespace["stopj"].assert_called_once()
    namespace["socket_send_int"].assert_not_called()


# --- Script command loop ---


def _feed_script_commands(namespace, packets):
    """
    Feed `packets` to socket_read_binary_integer in order, then stop.

    After the last packet is delivered, control_mode drops to MODE_STOPPED on
    the next read, ending scriptCommands()'s loop.
    """
    remaining = list(packets)

    def side_effect(*args, **kwargs):
        if remaining:
            return remaining.pop(0)
        namespace["control_mode"] = namespace["MODE_STOPPED"]
        return [0]

    namespace["socket_read_binary_integer"].side_effect = side_effect


def test_script_commands_loop_runs_each_command_until_stopped():
    namespace = load_control_script_namespace()
    namespace["control_mode"] = namespace["MODE_IDLE"]
    _feed_script_commands(namespace, [
        script_command_set_payload(5.0, [0.0, 0.0, 0.1]),
        script_command_set_payload(2.0, [0.0, 0.0, 0.2]),
    ])

    namespace["scriptCommands"]()

    assert namespace["set_payload"].call_count == 2
    mass, cog = namespace["set_payload"].call_args.args
    assert mass == pytest.approx(2.0)
    assert cog == pytest.approx([0.0, 0.0, 0.2])


@pytest.mark.parametrize("mass", [14.5, -0.1, 100.0])
def test_set_payload_out_of_range_mass_is_refused_and_logged(mass):
    namespace = load_control_script_namespace()
    packet = script_command_set_payload(mass, [0, 0, 0])
    namespace["socket_read_binary_integer"].return_value = packet

    namespace["runScriptCommandIteration"]()

    namespace["set_payload"].assert_not_called()
    namespace["textmsg"].assert_called_once()
    assert "payload refused" in namespace["textmsg"].call_args.args[0]


def test_set_payload_cap_comes_from_the_driver_template():
    namespace = load_control_script_namespace(template_overrides={"PAYLOAD_MASS_MAX_REPLACE": 3.0})
    packet = script_command_set_payload(5.0, [0, 0, 0])
    namespace["socket_read_binary_integer"].return_value = packet

    namespace["runScriptCommandIteration"]()

    assert namespace["PAYLOAD_MASS_MAX"] == 3.0
    namespace["set_payload"].assert_not_called()


# --- Main-loop pieces ---


def test_open_control_sockets_opens_all_three():
    namespace = load_control_script_namespace()
    namespace["openControlSockets"]()
    assert namespace["socket_open"].call_count == 3


def test_initialize_control_state_sets_expected_globals():
    namespace = load_control_script_namespace()
    namespace["get_actual_joint_positions"].return_value = [2.0] * 6

    namespace["initializeControlState"]()

    assert namespace["control_mode"] == namespace["MODE_UNINITIALIZED"]
    assert namespace["cmd_servo_joints"] == [2.0] * 6
    assert namespace["trajectory_point_num"] == 0
    assert namespace["move_thread_handle"] == 0
    assert namespace["trajectory_thread_handle"] == 0
    assert namespace["servo_start_check_pending"] is False
    assert namespace["trajectory_active"] is False
    assert namespace["freedrive_active"] is False
    assert namespace["read_timeout"] == 0.0
    namespace["start_thread"].assert_called_once()


def test_handle_control_mode_switch_into_servoj_starts_fresh_from_actual_position():
    namespace = load_control_script_namespace()
    namespace["control_mode"] = namespace["MODE_IDLE"]
    namespace["move_thread_handle"] = 0
    namespace["get_actual_joint_positions"].return_value = [1.0] * 6

    namespace["handleControlModeSwitch"](namespace["MODE_SERVOJ"])

    assert namespace["control_mode"] == namespace["MODE_SERVOJ"]
    assert namespace["cmd_servo_joints"] == [1.0] * 6
    assert namespace["servo_start_check_pending"] is True
    namespace["start_thread"].assert_called_once()
    called_fn, _called_args = namespace["start_thread"].call_args.args
    assert called_fn is namespace["servoThread"]


def test_handle_control_mode_switch_out_of_trajectory_cancels_it():
    namespace = load_control_script_namespace()
    namespace["control_mode"] = namespace["MODE_TRAJECTORY"]
    namespace["trajectory_thread_handle"] = 42
    namespace["trajectory_point_num"] = 0
    namespace["trajectory_active"] = True

    namespace["handleControlModeSwitch"](namespace["MODE_IDLE"])

    namespace["stop_thread"].assert_called_once_with(42)
    namespace["stopj"].assert_called_once()
    namespace["socket_send_int"].assert_called_once_with(
        namespace["TRAJECTORY_RESULT_CANCELED"], "trajectory_socket"
    )
    assert namespace["trajectory_thread_handle"] == 0
    assert namespace["control_mode"] == namespace["MODE_IDLE"]


def test_apply_mode_command_servoj_decodes_zoom_ratio():
    namespace = load_control_script_namespace()
    namespace["control_mode"] = namespace["MODE_SERVOJ"]
    namespace["servo_start_check_pending"] = False
    raw_joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

    namespace["applyModeCommand"](reverse_servoj(raw_joints))

    assert namespace["cmd_servo_joints"] == pytest.approx(raw_joints)


def test_run_control_loop_iteration_mode_switch_and_command_in_one_shot():
    namespace = load_control_script_namespace()
    raw_joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    # The arm already sits at the target, so the start-target check passes.
    namespace["get_actual_joint_positions"].return_value = raw_joints
    namespace["initializeControlState"]()
    namespace["start_thread"].reset_mock()
    namespace["socket_read_binary_integer"].return_value = reverse_servoj(raw_joints)

    namespace["runControlLoopIteration"]()

    assert namespace["control_mode"] == namespace["MODE_SERVOJ"]
    assert namespace["cmd_servo_joints"] == pytest.approx(raw_joints)
    namespace["start_thread"].assert_called_once()


def test_run_control_loop_iteration_timeout_stops_script():
    namespace = load_control_script_namespace()
    namespace["control_mode"] = namespace["MODE_IDLE"]
    namespace["read_timeout"] = 0.0
    namespace["socket_read_binary_integer"].return_value = [0]

    namespace["runControlLoopIteration"]()

    assert namespace["control_mode"] == namespace["MODE_STOPPED"]


def test_read_reverse_command_returns_the_read_and_updates_the_timeout():
    namespace = load_control_script_namespace()
    namespace["read_timeout"] = 0.0
    packet = reverse_idle(timeout_ms=200)
    namespace["socket_read_binary_integer"].return_value = packet

    assert namespace["readReverseCommand"]() == packet
    assert namespace["read_timeout"] == pytest.approx(0.2)


def test_read_reverse_command_keeps_the_timeout_on_a_failed_read():
    namespace = load_control_script_namespace()
    namespace["read_timeout"] = 0.2
    namespace["socket_read_binary_integer"].return_value = [0]

    assert namespace["readReverseCommand"]() == [0]
    assert namespace["read_timeout"] == pytest.approx(0.2)


def test_shutdown_control_threads_call_order():
    namespace = load_control_script_namespace()
    manager = MagicMock()
    namespace["stop_thread"] = manager.stop_thread
    namespace["join_thread"] = manager.join_thread
    namespace["stopj"] = manager.stopj
    namespace["socket_close"] = manager.socket_close

    namespace["script_command_thread_handle"] = 1
    namespace["move_thread_handle"] = 2
    namespace["trajectory_thread_handle"] = 3

    namespace["shutdownControlThreads"]()

    assert manager.mock_calls == [
        call.stop_thread(1),
        call.join_thread(1),
        call.stop_thread(2),
        call.join_thread(2),
        call.stop_thread(3),
        call.join_thread(3),
        call.stopj(namespace["STOPJ_ACCELERATION"]),
        call.socket_close("reverse_socket"),
        call.socket_close("trajectory_socket"),
        call.socket_close("script_command_socket"),
    ]


# --- Decoding ---


def test_decode_vector6_scales_six_words_from_an_offset():
    namespace = load_control_script_namespace()
    words = [99, 1, 1500000, -2500000, 0, 4000000, 5000000, 6000000, 77]

    values = namespace["decodeVector6"](words, 2, 1000000)

    assert values == pytest.approx([1.5, -2.5, 0.0, 4.0, 5.0, 6.0])


def test_decode_vector6_returns_a_fresh_list_each_call():
    """
    Guard against aliasing between decoded setpoints.

    setServoSetpoint() stores the decoded list itself, so a shared list would
    let the next decode rewrite the stored setpoint.
    """
    namespace = load_control_script_namespace()
    first = namespace["decodeVector6"]([0] * 7, 1, 1)
    second = namespace["decodeVector6"]([1] * 7, 1, 1)

    assert first is not second
    assert first == [0.0] * 6


# --- Static checks of the script source ---


def test_control_loop_region_calls_header_functions_with_matching_arity():
    """
    Check the never-exec()'d main-loop region statically.

    Every call there must name a header function and pass the number of
    arguments that function takes. Every name it reads must exist too.
    """
    body = SCRIPT_PATH.read_text().partition(HEADER_END_MARKER)[2]
    for key, value in DEFAULT_TEMPLATE_VALUES.items():
        body = body.replace("{{" + key + "}}", str(value))
    namespace = load_control_script_namespace()
    namespace["initializeControlState"]()

    for node in ast.walk(ast.parse(body)):
        if isinstance(node, ast.Call):
            target = namespace[node.func.id]
            if inspect.isfunction(target):
                expected = len(inspect.signature(target).parameters)
                assert len(node.args) == expected, node.func.id
        elif isinstance(node, ast.Name):
            assert node.id in namespace, node.id


# Python builtins the script may call, because EliteScript provides them too.
# Elite's upstream external_control.script calls each of them.
_SHARED_PYTHON_BUILTINS = {"int", "str", "range"}


def test_header_calls_only_script_functions_mocked_builtins_or_shared_builtins():
    """
    Catch an EliteScript builtin that the harness forgot to mock.

    An unmocked name would silently resolve to a Python builtin, or fail only
    when its code path finally runs.
    """
    header = ast.parse(load_header_source())
    script_functions = {n.name for n in header.body if isinstance(n, ast.FunctionDef)}
    allowed = script_functions | set(make_mock_builtins()) | _SHARED_PYTHON_BUILTINS

    for node in ast.walk(header):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            assert node.func.id in allowed, node.func.id


def test_script_avoids_constructs_unproven_on_the_robot_interpreter():
    """
    Keep the script to constructs Elite's own script relies on.

    Elite's upstream external_control.script uses return values, for/range,
    norm(), str.format(), and the like, so those count as proven. It never
    uses the constructs rejected here, so neither does this script.
    """
    for node in ast.walk(ast.parse(load_header_source())):
        assert not isinstance(node, ast.Not), "'not' is unproven"
        assert not isinstance(node, ast.Break), "'break' is unproven"
        assert not isinstance(node, (ast.ListComp, ast.GeneratorExp, ast.Lambda)), \
            "comprehensions and lambdas are unproven"
        if isinstance(node, ast.Call):
            if isinstance(node.func, ast.Name):
                assert node.func.id != "abs", "abs() is unproven, use norm()"
            if isinstance(node.func, ast.Attribute):
                assert node.func.attr != "append", "list.append() is unproven"

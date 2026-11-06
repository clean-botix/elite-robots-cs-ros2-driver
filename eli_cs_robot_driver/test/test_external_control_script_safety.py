"""
Tests for the servoj start-target safety check.

Entering MODE_SERVOJ arms servo_start_check_pending. The first servoj target
of the session is then compared against a fresh actual-position reading by
verifyServoStartSetpoint(). A target farther than SERVOJ_START_DELTA_LIMIT on
any joint is refused, and the script halts. The refused target never reaches
cmd_servo_joints, so the servo thread keeps holding the actual position. Later
targets in the session skip the check, because they follow the host's own
smooth trajectory.
"""

from unittest.mock import MagicMock

import pytest

from external_control_script_harness import (
    load_control_script_namespace,
    reverse_idle,
    reverse_servoj,
    ScriptHalted,
)

INCIDENT_DELTA = 0.863  # radians, the stale-command snap-back reproduced in sim


def _enter_servo_mode(namespace, actual):
    namespace["control_mode"] = namespace["MODE_IDLE"]
    namespace["move_thread_handle"] = 0
    namespace["trajectory_thread_handle"] = 0
    namespace["get_actual_joint_positions"].return_value = list(actual)
    namespace["handleControlModeSwitch"](namespace["MODE_SERVOJ"])


def _apply_servoj(namespace, target):
    namespace["applyModeCommand"](reverse_servoj(target))


def test_entering_servo_mode_arms_the_check():
    namespace = load_control_script_namespace()
    _enter_servo_mode(namespace, [0.0] * 6)
    assert namespace["servo_start_check_pending"] is True


def test_in_tolerance_start_target_is_accepted_and_disarms_the_check():
    namespace = load_control_script_namespace()
    _enter_servo_mode(namespace, [0.0] * 6)
    target = [0.01] * 6

    _apply_servoj(namespace, target)

    assert namespace["cmd_servo_joints"] == pytest.approx(target)
    assert namespace["servo_start_check_pending"] is False
    namespace["halt"].assert_not_called()
    namespace["popup"].assert_not_called()


@pytest.mark.parametrize("joint_index", [0, 5])
def test_over_limit_start_target_halts_and_keeps_holding(joint_index):
    namespace = load_control_script_namespace()
    actual = [0.0] * 6
    _enter_servo_mode(namespace, actual)
    target = list(actual)
    target[joint_index] = INCIDENT_DELTA

    with pytest.raises(ScriptHalted):
        _apply_servoj(namespace, target)

    assert namespace["cmd_servo_joints"] == actual
    namespace["halt"].assert_called_once()
    namespace["popup"].assert_called_once()
    popup_kwargs = namespace["popup"].call_args.kwargs
    assert popup_kwargs["title"] == "External Control Safety Fault"
    assert popup_kwargs["error"] is True
    assert popup_kwargs["blocking"] is False
    fault_text = namespace["popup"].call_args.args[0]
    assert ("joint " + str(joint_index)) in fault_text
    namespace["textmsg"].assert_called_once_with(fault_text)


def test_fault_message_names_the_first_offending_joint_only():
    namespace = load_control_script_namespace()
    _enter_servo_mode(namespace, [0.0] * 6)

    with pytest.raises(ScriptHalted):
        _apply_servoj(namespace, [0.0, 0.0, 1.0, 1.0, 0.0, 0.0])

    fault_text = namespace["popup"].call_args.args[0]
    assert "joint 2" in fault_text
    assert "joint 3" not in fault_text


def test_boundary_exactly_at_limit_is_accepted():
    namespace = load_control_script_namespace()
    limit = namespace["SERVOJ_START_DELTA_LIMIT"]
    _enter_servo_mode(namespace, [0.0] * 6)

    assert namespace["verifyServoStartSetpoint"]([limit, 0.0, 0.0, 0.0, 0.0, 0.0]) is True
    namespace["halt"].assert_not_called()


def test_just_over_limit_is_refused():
    namespace = load_control_script_namespace()
    limit = namespace["SERVOJ_START_DELTA_LIMIT"]
    _enter_servo_mode(namespace, [0.0] * 6)

    with pytest.raises(ScriptHalted):
        namespace["verifyServoStartSetpoint"]([limit + 0.001, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_negative_direction_delta_is_also_refused():
    namespace = load_control_script_namespace()
    _enter_servo_mode(namespace, [1.0] * 6)

    with pytest.raises(ScriptHalted):
        _apply_servoj(namespace, [0.0] * 6)

    assert namespace["cmd_servo_joints"] == [1.0] * 6


# --- Joint value ranges: the delta is never wrapped at +/-pi ---


@pytest.mark.parametrize("joint_index", [0, 5])
def test_start_target_across_plus_minus_pi_is_refused_as_a_near_full_turn(joint_index):
    """
    Pin that the check uses the plain difference, with no wrap at +/-pi.

    Actual -3.10 rad and target +3.10 rad are 6.20 rad apart on a +/-360
    degree joint. Wrapped, they would look 0.08 rad apart, under the limit.
    """
    namespace = load_control_script_namespace()
    actual = [0.0] * 6
    actual[joint_index] = -3.10
    _enter_servo_mode(namespace, actual)
    target = list(actual)
    target[joint_index] = 3.10

    with pytest.raises(ScriptHalted):
        _apply_servoj(namespace, target)

    assert namespace["cmd_servo_joints"] == actual
    assert "delta=6.2" in namespace["popup"].call_args.args[0]


def test_small_start_target_step_through_plus_minus_pi_is_accepted():
    """Crossing +/-pi is ordinary motion, because the angle is continuous there."""
    namespace = load_control_script_namespace()
    _enter_servo_mode(namespace, [3.13, 0.0, 0.0, 0.0, 0.0, -3.13])

    _apply_servoj(namespace, [3.15, 0.0, 0.0, 0.0, 0.0, -3.15])

    assert namespace["cmd_servo_joints"] == pytest.approx([3.15, 0.0, 0.0, 0.0, 0.0, -3.15])
    namespace["halt"].assert_not_called()


def test_start_target_near_the_two_pi_limit_uses_the_plain_difference():
    """
    Near +/-2*pi, theta and theta - 2*pi are different physical poses.

    A small step at 6.20 rad is accepted. The same pose expressed one turn
    lower, at -0.08 rad, is a full turn away and is refused.
    """
    namespace = load_control_script_namespace()
    actual = [6.20, 0.0, 0.0, 0.0, 0.0, 0.0]
    _enter_servo_mode(namespace, actual)
    assert namespace["verifyServoStartSetpoint"]([6.25, 0.0, 0.0, 0.0, 0.0, 0.0]) is True

    with pytest.raises(ScriptHalted):
        one_turn_lower = 6.20 - 2 * 3.141592653589793
        namespace["verifyServoStartSetpoint"]([one_turn_lower, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_elbow_at_its_plus_minus_pi_limit_uses_the_plain_difference():
    """The elbow's range ends at +/-pi, so its opposite ends are a full turn apart."""
    namespace = load_control_script_namespace()
    _enter_servo_mode(namespace, [0.0, 0.0, 3.10, 0.0, 0.0, 0.0])

    with pytest.raises(ScriptHalted):
        _apply_servoj(namespace, [0.0, 0.0, -3.10, 0.0, 0.0, 0.0])


def test_later_targets_in_the_session_skip_the_check():
    """
    Pin the design choice that only the start target is vetted.

    A mid-session jump is accepted without even reading the actual position.
    """
    namespace = load_control_script_namespace()
    _enter_servo_mode(namespace, [0.0] * 6)
    _apply_servoj(namespace, [0.01] * 6)
    namespace["get_actual_joint_positions"].reset_mock()

    _apply_servoj(namespace, [1.0] * 6)

    assert namespace["cmd_servo_joints"] == pytest.approx([1.0] * 6)
    namespace["get_actual_joint_positions"].assert_not_called()
    namespace["halt"].assert_not_called()


def test_re_entering_servo_mode_re_arms_the_check():
    namespace = load_control_script_namespace()
    _enter_servo_mode(namespace, [0.0] * 6)
    _apply_servoj(namespace, [0.01] * 6)
    namespace["handleControlModeSwitch"](namespace["MODE_IDLE"])

    namespace["get_actual_joint_positions"].return_value = [0.5] * 6
    namespace["handleControlModeSwitch"](namespace["MODE_SERVOJ"])

    assert namespace["servo_start_check_pending"] is True
    with pytest.raises(ScriptHalted):
        _apply_servoj(namespace, [0.01] * 6)


def test_refused_target_is_not_written_even_if_halt_returns():
    """
    Guard against halt() returning on the real interpreter.

    The refusal must not depend on halt() ending the script. The check stays
    armed and the target never reaches cmd_servo_joints.
    """
    namespace = load_control_script_namespace(builtin_overrides={"halt": MagicMock()})
    _enter_servo_mode(namespace, [0.0] * 6)

    _apply_servoj(namespace, [INCIDENT_DELTA] * 6)

    namespace["halt"].assert_called_once()
    assert namespace["cmd_servo_joints"] == [0.0] * 6
    assert namespace["servo_start_check_pending"] is True


def test_verify_returns_false_when_halt_returns():
    namespace = load_control_script_namespace(builtin_overrides={"halt": MagicMock()})
    namespace["get_actual_joint_positions"].return_value = [0.0] * 6

    assert namespace["verifyServoStartSetpoint"]([INCIDENT_DELTA] * 6) is False
    # The first offending joint ends the scan, so only one fault is reported
    namespace["popup"].assert_called_once()


def test_limit_comes_from_the_driver_template():
    namespace = load_control_script_namespace(
        template_overrides={"SERVOJ_START_DELTA_LIMIT_REPLACE": 0.5}
    )
    _enter_servo_mode(namespace, [0.0] * 6)

    _apply_servoj(namespace, [0.3] * 6)

    assert namespace["SERVOJ_START_DELTA_LIMIT"] == 0.5
    namespace["halt"].assert_not_called()


def test_stale_command_after_script_resend_halts_end_to_end():
    """
    Replay the snap-back incident through the real control-loop entry points.

    A freshly started script receives a servoj packet whose target lies far
    from the arm's actual position. The script must halt before moving.
    """
    namespace = load_control_script_namespace()
    actual = [0.1, -1.2, 1.4, -1.7, -1.57, 0.0]
    namespace["get_actual_joint_positions"].return_value = actual
    namespace["initializeControlState"]()
    stale = list(actual)
    stale[1] += INCIDENT_DELTA
    namespace["socket_read_binary_integer"].return_value = reverse_servoj(stale)

    with pytest.raises(ScriptHalted):
        namespace["runControlLoopIteration"]()

    assert namespace["control_mode"] == namespace["MODE_SERVOJ"]
    assert namespace["cmd_servo_joints"] == actual
    namespace["servoj"].assert_not_called()


def test_session_resume_from_idle_then_servoj_checks_again_end_to_end():
    namespace = load_control_script_namespace()
    namespace["get_actual_joint_positions"].return_value = [0.0] * 6
    namespace["initializeControlState"]()
    reads = [reverse_servoj([0.01] * 6), reverse_idle(), reverse_servoj([0.5] * 6)]
    namespace["socket_read_binary_integer"].side_effect = reads

    namespace["runControlLoopIteration"]()
    namespace["runControlLoopIteration"]()
    with pytest.raises(ScriptHalted):
        namespace["runControlLoopIteration"]()

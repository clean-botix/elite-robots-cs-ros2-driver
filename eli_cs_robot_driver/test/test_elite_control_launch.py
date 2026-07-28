from pathlib import Path

LAUNCH_FILE = Path(__file__).resolve().parent.parent / "launch" / "elite_control.launch.py"


def test_controller_stopper_bool_params_are_wrapped():
    # headless_mode and joint_controller_active are declared as bool in
    # controller_stopper_node.cpp (declare_parameter<bool>(...)). A bare
    # LaunchConfiguration always resolves to a string, which throws
    # rclcpp::exceptions::InvalidParameterTypeException and aborts the node at
    # startup. Guard against regressing back to the unwrapped form.
    source = LAUNCH_FILE.read_text()
    assert "ParameterValue(headless_mode, value_type=bool)" in source
    assert "ParameterValue(activate_joint_controller, value_type=bool)" in source
    assert '{"headless_mode": headless_mode}' not in source
    assert '{"joint_controller_active": activate_joint_controller}' not in source


def test_parameter_value_resolves_launch_configuration_to_bool():
    # Confirms the actual fix mechanism: ParameterValue(..., value_type=bool)
    # turns a LaunchConfiguration's string ("true"/"false") into a real Python
    # bool, not a string. Requires launch/launch_ros (not available outside a
    # ROS environment); runs under colcon test / CI.
    from launch import LaunchContext
    from launch.substitutions import LaunchConfiguration
    from launch_ros.parameter_descriptions import ParameterValue

    context = LaunchContext()

    context.launch_configurations["headless_mode"] = "true"
    resolved = ParameterValue(LaunchConfiguration("headless_mode"), value_type=bool).evaluate(context)
    assert resolved is True

    context.launch_configurations["headless_mode"] = "false"
    resolved = ParameterValue(LaunchConfiguration("headless_mode"), value_type=bool).evaluate(context)
    assert resolved is False

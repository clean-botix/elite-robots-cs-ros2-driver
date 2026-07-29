cmake_minimum_required(VERSION 3.8)
project(eli_cs_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(eli_dashboard_interface REQUIRED)
find_package(eli_common_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(backward_ros REQUIRED)
find_package(elite-cs-series-sdk REQUIRED)

set(
  THIS_PACKAGE_DEPENDENCIES
  eli_common_interface
  eli_dashboard_interface
  hardware_interface
  rclcpp_lifecycle
  pluginlib
  tf2_geometry_msgs
  std_srvs
  rclcpp 
  rclcpp_components
  controller_manager_msgs
)

# Add dashboard client component
add_library(dashboard_client_component SHARED src/dashboard_client.cpp)
target_link_libraries(
  dashboard_client_component
  elite-cs-series-sdk
)
target_include_directories(
  dashboard_client_component
  PRIVATE
  include
)
ament_target_dependencies(dashboard_client_component ${THIS_PACKAGE_DEPENDENCIES})
rclcpp_components_register_node(
  dashboard_client_component 
  PLUGIN "ELITE_CS_ROBOT_ROS_DRIVER::DashboardClient" 
  EXECUTABLE dashboard_client
)
ament_export_targets(export_dashboard_client_component)
install(
  TARGETS dashboard_client_component
  EXPORT export_dashboard_client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Add script sender component
add_library(script_node_component SHARED src/script_node.cpp)
target_link_libraries(
  script_node_component
  elite-cs-series-sdk
)
target_include_directories(
  script_node_component
  PRIVATE
  include
)
ament_target_dependencies(script_node_component ${THIS_PACKAGE_DEPENDENCIES})
rclcpp_components_register_node(
  script_node_component 
  PLUGIN "ELITE_CS_ROBOT_ROS_DRIVER::ScriptNode" 
  EXECUTABLE script_node
)
ament_export_targets(export_script_node_component)
install(
  TARGETS script_node_component
  EXPORT export_script_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Add components loader
add_executable(
  eli_components_loader
  src/load_components.cpp
)
target_link_libraries(
  eli_components_loader
  script_node_component
  dashboard_client_component
  elite-cs-series-sdk
)
target_include_directories(
  eli_components_loader
  PRIVATE
  include
)
ament_target_dependencies(eli_components_loader ${THIS_PACKAGE_DEPENDENCIES})
install(
  TARGETS eli_components_loader
  DESTINATION lib/${PROJECT_NAME}
)

# Add hardware interface 
add_library(
  eli_cs_hardware_interface_plugin
  SHARED
  src/hardware_interface.cpp
)
target_link_libraries(
  eli_cs_hardware_interface_plugin
  elite-cs-series-sdk
)
target_include_directories(
  eli_cs_hardware_interface_plugin
  PRIVATE
  include
)
ament_target_dependencies(
  eli_cs_hardware_interface_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_DEPENDENCIES}
)
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)
install(
  TARGETS eli_cs_hardware_interface_plugin
  DESTINATION lib
)


# Add controller stopper node
add_executable(
  controller_stopper_node
  src/controller_stopper.cpp
  src/controller_stopper_node.cpp
)
target_include_directories(
  controller_stopper_node
  PRIVATE
  include
)
ament_target_dependencies(
  controller_stopper_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${THIS_PACKAGE_DEPENDENCIES}
)
install(
  TARGETS controller_stopper_node
  DESTINATION lib/${PROJECT_NAME}
)

#
# eli_ros2_control_node
#
add_executable(
  eli_ros2_control_node
  src/control_node.cpp
  src/rt_memory.cpp
  src/rt_memory_reporting.cpp
)
ament_target_dependencies(
  eli_ros2_control_node
  controller_manager
  rclcpp
)
target_include_directories(
  eli_ros2_control_node
  PRIVATE
  include
)
install(
  TARGETS eli_ros2_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# install resources file
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# export
ament_export_include_directories(
  include
)
ament_export_libraries(
  eli_cs_hardware_interface_plugin
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_DEPENDENCIES}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Unit tests for the ROS-free real-time memory MANAGEMENT helpers (rt_memory.hpp).
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rt_memory test/test_rt_memory.cpp)
  target_include_directories(test_rt_memory PRIVATE include)

  # Unit tests for the real-time memory REPORTING helpers (rt_memory_reporting.hpp).
  ament_add_gtest(test_rt_memory_reporting test/test_rt_memory_reporting.cpp)
  target_include_directories(test_rt_memory_reporting PRIVATE include)

  # Regression test for elite_control.launch.py's bool-typed launch parameters.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_elite_control_launch test/test_elite_control_launch.py)
endif()

ament_package()

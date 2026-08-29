cmake_minimum_required(VERSION 3.8)
project(crane_supervisor)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(crane_msgs REQUIRED)
find_package(epsilon_crane_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# The decision core: an input struct in, a decision struct out. No node handle,
# no message type, no clock, no topic. It is a separate target rather than a
# folder inside the node so that the test binary below can link it with no ROS
# runtime in the process at all, which is what makes every cause reachable one
# at a time (wiki/implementation/style_guide.md §3, §6).
add_library(supervisor_core STATIC src/supervisor_core.cpp src/sway_monitor.cpp)
add_library(${PROJECT_NAME}::supervisor_core ALIAS supervisor_core)
set_target_properties(supervisor_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(supervisor_core PUBLIC cxx_std_17)
target_include_directories(supervisor_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

generate_parameter_library(crane_supervisor_parameters
  src/crane_supervisor_parameters.yaml)

# The adapter, as a library rather than as part of the executable: the stream
# test constructs this node object in process, so that what is asserted is the
# node the deployment runs and not a second one written for the test.
add_library(supervisor_node STATIC src/supervisor_node.cpp)
add_library(${PROJECT_NAME}::supervisor_node ALIAS supervisor_node)
set_target_properties(supervisor_node PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(supervisor_node PUBLIC cxx_std_17)
target_include_directories(supervisor_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(supervisor_node
  PUBLIC
    supervisor_core
    rclcpp::rclcpp
    ${control_msgs_TARGETS}
    ${controller_manager_msgs_TARGETS}
    ${crane_msgs_TARGETS}
    ${epsilon_crane_msgs_TARGETS}
    ${rcl_interfaces_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_srvs_TARGETS}
  PRIVATE
    crane_supervisor_parameters)

add_executable(crane_supervisor src/crane_supervisor_main.cpp)
target_link_libraries(crane_supervisor PRIVATE supervisor_node)

install(
  DIRECTORY include/
  DESTINATION include)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(
  TARGETS supervisor_core supervisor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(
  TARGETS crane_supervisor
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
# The core only. It is the half of this package that has no dependencies at all,
# so exporting it costs a consumer nothing; the adapter is exported by its
# header alone, because a second process linking the node would be a second
# publisher on a contract stream that has exactly one.
ament_export_libraries(supervisor_core)
ament_export_dependencies(
  control_msgs controller_manager_msgs crane_msgs epsilon_crane_msgs rcl_interfaces rclcpp
  sensor_msgs std_srvs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(controller_manager REQUIRED)

  # The seams outside this package: the numbering
  # `crane_msgs/SupervisorStatus` fixes, and which
  # `epsilon_crane_msgs/RemoteCtrlStates` boolean the configured deadman
  # selects. Message structs only -- the adapter is linked for one free
  # function, and no node is constructed, no clock read, no graph joined.
  ament_add_gtest(test_contract test/test_contract.cpp)
  target_link_libraries(test_contract
    supervisor_core
    supervisor_node
    ${crane_msgs_TARGETS}
    ${epsilon_crane_msgs_TARGETS})

  # The decision core, offline. No ROS is linked into this binary.
  ament_add_gtest(test_supervisor_core test/test_supervisor_core.cpp)
  target_link_libraries(test_supervisor_core supervisor_core)

  # The sway bound and the settled predicate, on fixed rate sequences: the
  # crossing, the dwell, the hysteresis and the unknown state, each driven one
  # cycle at a time with the clock as an argument. No ROS is linked here either,
  # which is what lets a two-second dwell be asserted in microseconds.
  ament_add_gtest(test_sway_monitor test/test_sway_monitor.cpp)
  target_link_libraries(test_sway_monitor supervisor_core)

  # The published stream, in process: the node object and an observer in one
  # executor on the isolated domain `ralph/verify.sh` pins. No launch, no
  # controller manager, no simulator.
  ament_add_gtest(test_status_stream test/test_status_stream.cpp TIMEOUT 120)
  target_link_libraries(test_status_stream
    supervisor_node
    ${control_msgs_TARGETS}
    ${crane_msgs_TARGETS}
    ${epsilon_crane_msgs_TARGETS}
    ${std_srvs_TARGETS})
  # The fixture that proves the *file* is the source of the six numbers: it is
  # written in the shape `crane_control/config/tracking_tolerance.yaml` has,
  # wildcard node key and all, so a node that could not read that file cannot
  # read this one either.
  target_compile_definitions(test_status_stream PRIVATE
    CRANE_SUPERVISOR_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/config")

  # The mode, against a real controller manager in this process: the S5 shape
  # `crane_control` already uses (PRD "Seams", S5), with mock hardware and the
  # control loop driven by hand on the isolated domain `ralph/verify.sh` pins.
  # A switch is the one call that must never be issued against the machine by
  # accident, so it is asserted here and never against a launch.
  ament_add_gtest(test_mode_switch test/test_mode_switch.cpp TIMEOUT 300)
  target_link_libraries(test_mode_switch
    supervisor_node
    controller_manager::controller_manager
    ${controller_manager_msgs_TARGETS}
    ${crane_msgs_TARGETS}
    ${epsilon_crane_msgs_TARGETS}
    ${std_srvs_TARGETS})
  target_compile_definitions(test_mode_switch PRIVATE
    CRANE_SUPERVISOR_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/config")

  # PRD §10's sequence, against a controller manager that can be *watched*: the
  # spy counts every switch call, so "the switch was never issued" and "the
  # switch was issued and refused" are two observations here rather than one.
  # It is also where the freshness check's position is proven dynamically --
  # with no manager on the graph at all, the refusal still names the horizon --
  # and where the producer's shadow/active transition and its ordering are
  # asserted. Its own stepped domain, because colcon runs it beside the two
  # binaries above.
  ament_add_gtest(test_handover_sequence test/test_handover_sequence.cpp TIMEOUT 300)
  target_link_libraries(test_handover_sequence
    supervisor_node
    ${controller_manager_msgs_TARGETS}
    ${crane_msgs_TARGETS}
    ${epsilon_crane_msgs_TARGETS}
    ${rcl_interfaces_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_srvs_TARGETS})

  # The absence that is the feature: no publisher on a command topic, and no
  # reach beyond the controller-manager services ROS 2 Interfaces §5 puts
  # here on purpose, and the one parameter call ROS 2 Interfaces §4 does.
  ament_add_pytest_test(test_no_command_path test/test_no_command_path.py)

  # The other absence that is a feature: this package keeps no copy of the one
  # tracking tolerance, so the file that owns it cannot be disagreed with.
  ament_add_pytest_test(test_no_tolerance_of_its_own test/test_no_tolerance_of_its_own.py)

  # The freshness policy, as a shape rather than as a checklist: one
  # `create_subscription` in the package, one policy row and one configured
  # deadline per enumerator of `Input`, and no default margin for an input
  # nobody configured (wiki/control_architecture.md §5.3).
  ament_add_pytest_test(
    test_every_input_has_a_deadline test/test_every_input_has_a_deadline.py)
endif()

ament_package()

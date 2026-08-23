cmake_minimum_required(VERSION 3.8)
project(crane_supervisor)

find_package(ament_cmake REQUIRED)
find_package(crane_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)

# The decision core: an input struct in, a decision struct out. No node handle,
# no message type, no clock, no topic. It is a separate target rather than a
# folder inside the node so that the test binary below can link it with no ROS
# runtime in the process at all, which is what makes every cause reachable one
# at a time (wiki/implementation/style_guide.md §3, §6).
add_library(supervisor_core STATIC src/supervisor_core.cpp)
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
    ${crane_msgs_TARGETS}
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
ament_export_dependencies(crane_msgs rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(crane_model REQUIRED)

  # The seams outside this package: the installed `crane_model` dynamics
  # contract, and the numbering `crane_msgs/SupervisorStatus` fixes. Message
  # structs only -- no node, no clock, no graph.
  ament_add_gtest(test_contract test/test_contract.cpp)
  target_link_libraries(test_contract
    supervisor_core
    crane_model::mock_model
    ${crane_msgs_TARGETS})

  # The decision core, offline. No ROS is linked into this binary.
  ament_add_gtest(test_supervisor_core test/test_supervisor_core.cpp)
  target_link_libraries(test_supervisor_core supervisor_core)

  # The published stream, in process: the node object and an observer in one
  # executor on the isolated domain `ralph/verify.sh` pins. No launch, no
  # controller manager, no simulator.
  ament_add_gtest(test_status_stream test/test_status_stream.cpp TIMEOUT 120)
  target_link_libraries(test_status_stream supervisor_node ${crane_msgs_TARGETS})

  # The absence that is the feature: no publisher on a command topic, no
  # controller-manager client, nothing that can deactivate a controller.
  ament_add_pytest_test(test_no_command_path test/test_no_command_path.py)
endif()

ament_package()

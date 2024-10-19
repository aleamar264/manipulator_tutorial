cmake_minimum_required(VERSION 3.8)
project(cpp_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)




add_executable(simple_parameter src/simple_parameter.cpp)
ament_target_dependencies(simple_parameter rclcpp rcl_interfaces)

add_executable(simple_services_server src/simple_services_server.cpp)
ament_target_dependencies(simple_services_server rclcpp rcl_interfaces robot_msgs)

add_executable(simple_services_client src/simple_services_client.cpp)
ament_target_dependencies(simple_services_client rclcpp rcl_interfaces robot_msgs)

add_executable(simple_moveit_Interface src/simple_moveit_Interface.cpp)
ament_target_dependencies(simple_moveit_Interface rclcpp rcl_interfaces moveit_ros_planning_interface)

add_executable(simple_lifecycle_node src/simple_lifecycle_node.cpp)
ament_target_dependencies(simple_lifecycle_node rclcpp rcl_interfaces rclcpp_lifecycle std_msgs)

add_library(simple_action_server SHARED src/simple_server_action.cpp)

target_include_directories(simple_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(simple_action_server
PRIVATE "SIMPLE_ACTION_SERVER_CPP_BUILDING_DLL")
ament_target_dependencies(simple_action_server
robot_msgs
rclcpp
rclcpp_action
rclcpp_components
)
rclcpp_components_register_node(simple_action_server
PLUGIN "cpp_example::SimpleActionServer"
EXECUTABLE simple_action_server_node)

add_library(simple_action_client SHARED src/simple_action_client.cpp)
target_include_directories(simple_action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(simple_action_client
PRIVATE "SIMPLE_ACTION_CLIENT_CPP_BUILDING_DLL")
ament_target_dependencies(simple_action_client
robot_msgs
rclcpp
rclcpp_action
rclcpp_components
)
rclcpp_components_register_node(simple_action_client
PLUGIN "cpp_example::SimpleActionClient"
EXECUTABLE simple_action_client_node)


install(TARGETS
simple_parameter
simple_services_server
simple_services_client
simple_action_server
simple_action_client
simple_moveit_Interface
simple_lifecycle_node
DESTINATION lib/${PROJECT_NAME})

install(TARGETS
simple_action_server
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin)

install(TARGETS
simple_action_client
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin)


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
endif()

ament_package()

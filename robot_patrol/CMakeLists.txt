cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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

rosidl_generate_interfaces(${PROJECT_NAME} 
"srv/GetDirection.srv"
"action/GoToPose.action"
DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

add_executable(robot_patrol_node src/patrol.cpp)
ament_target_dependencies(robot_patrol_node rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(robot_patrol_client_node src/patrol_with_service.cpp)
ament_target_dependencies(robot_patrol_client_node rclcpp std_msgs sensor_msgs geometry_msgs)
rosidl_get_typesupport_target(TARGET robot_patrol_client_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(robot_patrol_client_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

add_executable(direction_service_node src/direction_service.cpp)
ament_target_dependencies(direction_service_node rclcpp std_msgs sensor_msgs)
# Get the typesupport target for the package
rosidl_get_typesupport_target(TARGET direction_service_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Link the target with the generated interfaces using target_link_libraries
target_link_libraries(direction_service_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

add_executable(test_service_node src/test_service.cpp)
ament_target_dependencies(test_service_node rclcpp std_msgs sensor_msgs geometry_msgs)
rosidl_get_typesupport_target(TARGET test_service_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(test_service_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

add_executable(gotopose_service_node src/go_to_pose_action.cpp)
ament_target_dependencies(gotopose_service_node rclcpp rclcpp_action std_msgs geometry_msgs nav_msgs tf2)
rosidl_get_typesupport_target(TARGET gotopose_service_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(gotopose_service_node ${PROJECT_NAME}__rosidl_typesupport_cpp)


install(TARGETS
   robot_patrol_node
   robot_patrol_client_node
   direction_service_node
   test_service_node
   gotopose_service_node
   DESTINATION lib/${PROJECT_NAME}
 )
# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

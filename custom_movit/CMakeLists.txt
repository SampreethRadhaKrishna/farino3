cmake_minimum_required(VERSION 3.22)
project(custom_movit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fairino_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(ik_move_fr3 src/ik_move_fr3.cpp)
ament_target_dependencies(ik_move_fr3 rclcpp moveit_ros_planning_interface)

add_executable(move_fr3_server src/move_fr3_server.cpp)
ament_target_dependencies(move_fr3_server rclcpp fairino_msgs moveit_ros_planning_interface)

ament_package()

install(TARGETS
  ik_move_fr3
  move_fr3_server
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY src config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

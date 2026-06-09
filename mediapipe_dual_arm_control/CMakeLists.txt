cmake_minimum_required(VERSION 3.8)
project(mediapipe_dual_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_servo REQUIRED)

# Single-arm omniman pose-tracking node
add_executable(omniman_pose_tracking_node src/omniman_pose_tracking_node.cpp)
ament_target_dependencies(omniman_pose_tracking_node
  rclcpp
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  tf2_ros
  tf2_geometry_msgs
  moveit_servo
)

install(TARGETS omniman_pose_tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

# Python nodes (run via ros2 launch / ros2 run)
install(PROGRAMS
  scripts/hand_pose_publisher_node.py
  scripts/hand_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch + config
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
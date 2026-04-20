cmake_minimum_required(VERSION 3.8)
project(omniman_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


add_executable(commander src/commander.cpp)
add_executable(commander2 src/commander2.cpp)
add_executable(position src/position.cpp)
add_executable(quaternion_test src/quaternion_test.cpp)
add_executable(fira_mission_yolo_cpp src/fira_mission_yolo.cpp)

ament_target_dependencies(fira_mission_yolo_cpp
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
)

ament_target_dependencies(commander
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  example_interfaces
  std_msgs
  moveit_task_constructor_core
  tf2_msgs
  tf2_ros
)

ament_target_dependencies(commander2
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  example_interfaces
  std_msgs
  moveit_task_constructor_core
  tf2_msgs
  tf2_ros
)

ament_target_dependencies(position
  rclcpp
  moveit_ros_planning_interface
)

ament_target_dependencies(quaternion_test
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  example_interfaces
  std_msgs
  moveit_task_constructor_core
  tf2_msgs
  tf2_ros
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  commander
  commander2
  position
  quaternion_test
  fira_mission_yolo_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/mission_runner.py
  scripts/traffic_mission.py
  scripts/yolo_target_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/ground_best.pt
  scripts/wood_best.pt
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


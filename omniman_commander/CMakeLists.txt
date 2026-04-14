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


add_executable(commander src/commander.cpp)
add_executable(commander2 src/commander2.cpp)
add_executable(position src/position.cpp)
add_executable(quaternion_test src/quaternion_test.cpp)
add_executable(drive_distance src/drive_distance.cpp)
add_executable(drive_rectangle src/drive_rectangle.cpp)

ament_target_dependencies(drive_distance
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
)

ament_target_dependencies(drive_rectangle
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
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
  drive_distance
  drive_rectangle
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


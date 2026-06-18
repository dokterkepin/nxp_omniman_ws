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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)


# Reusable arm/gripper control library (shared by mission nodes).
add_library(arm_commander src/arm_commander.cpp)
target_include_directories(arm_commander PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(arm_commander
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(commander src/commander.cpp)
add_executable(commander1 src/commander1.cpp)
add_executable(commander2 src/commander2.cpp)
add_executable(commander_aruco src/commander_aruco.cpp)
add_executable(position src/position.cpp)

# commander1 / commander_aruco are mission nodes — they drive a state machine and
# call into the reusable ArmCommander library.
target_link_libraries(commander1 arm_commander)
target_link_libraries(commander_aruco arm_commander)

ament_target_dependencies(commander
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  example_interfaces
  std_msgs
  tf2_msgs
  tf2_ros
)

ament_target_dependencies(commander1
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

ament_target_dependencies(commander_aruco
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  aruco_opencv_msgs
)

ament_target_dependencies(commander2
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  example_interfaces
  std_msgs
  tf2_msgs
  tf2_ros
)

ament_target_dependencies(position
  rclcpp
  moveit_ros_planning_interface
)

install(DIRECTORY
  launch
  parameter
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS arm_commander
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  commander
  commander1
  commander2
  commander_aruco
  position
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


cmake_minimum_required(VERSION 3.8)
project(omniman_vla)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  config
  launch
  description
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  # recorder: data collection
  recorder/replay_arm.py
  recorder/replay_base.py
  recorder/joy_discrete_base.py
  recorder/trim_state.py
  # communication: the control lock and the nodes that take part in it
  communication/control_arbiter.py
  communication/policy_runner.py
  communication/visual_align.py
  # perception: SAM 3 target detector (GPU PC, lerobot_jazzy env)
  perception/sam_detector.py
  # commander: missions
  commander/pick_place_mission.py
  commander/pick_place_shuttle.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.5)
project(tf_frame_example)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(tf_listen_example src/tf_listen_example.cpp)

install(
  DIRECTORY
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/tf_listen_example.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()

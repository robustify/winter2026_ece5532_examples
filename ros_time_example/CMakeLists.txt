cmake_minimum_required(VERSION 3.5)
project(ros_time_example)
set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(time_stamp_example
  src/time_stamp_example.cpp
)

ament_auto_add_executable(timer_example
  src/timer_example.cpp
)

install(
  PROGRAMS
    scripts/time_stamp_example.py
    scripts/timer_example.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()

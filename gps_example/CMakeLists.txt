cmake_minimum_required(VERSION 3.5)
project(gps_example)
set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(gps_example
  src/gps_example.cpp
)

ament_auto_add_executable(enu_utm_comparison
  src/enu_utm_comparison.cpp
)

install(
  PROGRAMS
    scripts/gps_example.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()

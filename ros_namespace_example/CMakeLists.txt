cmake_minimum_required(VERSION 3.5)
project(ros_namespace_example)
set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(namespace_example
  src/namespace_example.cpp
)

install(
  PROGRAMS
    scripts/namespace_example.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()

cmake_minimum_required(VERSION 3.10)
project(piper_cpp_moveit)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch srdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

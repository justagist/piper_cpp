cmake_minimum_required(VERSION 3.10)
project(piper_cpp_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(piper_cpp REQUIRED)

add_library(piper_cpp_ros SHARED
  src/piper_hardware_interface.cpp
)
target_compile_features(piper_cpp_ros PUBLIC cxx_std_17)
target_include_directories(piper_cpp_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(piper_cpp_ros
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)
target_link_libraries(piper_cpp_ros piper_cpp::piper_cpp)

pluginlib_export_plugin_description_file(hardware_interface piper_cpp_ros.xml)

install(TARGETS piper_cpp_ros
  EXPORT export_piper_cpp_ros
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_piper_cpp_ros HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib piper_cpp)

ament_package()

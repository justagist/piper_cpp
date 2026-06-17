
cmake_minimum_required(VERSION 3.10)
project(piper_cpp VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find libsocketcan via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCANSOCKET REQUIRED libsocketcan)

# Library target
add_library(piper_cpp STATIC
    src/can_utils.cpp
    src/types/core.cpp
    src/protocol/piper_parser_v2.cpp
    src/interface/piper_interface_v2.cpp
)
# Built as a static lib but needs to be linkable into shared libs (e.g. ROS plugins).
set_target_properties(piper_cpp PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(piper_cpp
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE ${LIBCANSOCKET_INCLUDE_DIRS}
)

target_link_libraries(piper_cpp
    PUBLIC ${LIBCANSOCKET_LIBRARIES}
)

function(add_piper_executable exe_name src_file)
    add_executable(${exe_name} scripts/${src_file})
    target_link_libraries(${exe_name} PRIVATE piper_cpp)
    target_include_directories(${exe_name} PRIVATE include)
endfunction()

add_piper_executable(piper_cpp_can_utils_demo can_utils_demo.cpp)
add_piper_executable(piper_cpp_std_can_interface_read std_can_interface_read_test.cpp)
add_piper_executable(piper_parser_v2_read_test piper_parser_v2_read_test.cpp)
add_piper_executable(piper_interface_v2_read_test piper_interface_v2_read_test.cpp)
add_piper_executable(can_activate can_activate.cpp)
add_piper_executable(piper_enable piper_enable.cpp)
add_piper_executable(piper_gripper_sweep piper_gripper_sweep.cpp)
add_piper_executable(piper_joint_ctrl_demo piper_joint_ctrl_demo.cpp)
add_piper_executable(piper_joint_sine_demo piper_joint_sine_demo.cpp)
add_piper_executable(piper_end_pose_demo piper_end_pose_demo.cpp)
add_piper_executable(piper_go_zero piper_go_zero.cpp)
add_piper_executable(piper_set_zero piper_set_zero.cpp)
add_piper_executable(piper_set_role piper_set_role.cpp)
add_piper_executable(piper_read_firmware piper_read_firmware.cpp)
add_piper_executable(piper_reset piper_reset.cpp)

include(GNUInstallDirs)
install(TARGETS piper_cpp
    EXPORT piper_cppTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT dev
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(TARGETS piper_cpp_can_utils_demo
        piper_cpp_std_can_interface_read
        piper_parser_v2_read_test
        piper_interface_v2_read_test
        can_activate
        piper_enable
        piper_gripper_sweep
        piper_joint_ctrl_demo
        piper_joint_sine_demo
        piper_end_pose_demo
        piper_go_zero
        piper_set_zero
        piper_set_role
        piper_read_firmware
        piper_reset
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT tools
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT dev)

install(EXPORT piper_cppTargets
    FILE piper_cppTargets.cmake
    NAMESPACE piper_cpp::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/piper_cpp
    COMPONENT dev
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/piper_cppConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/piper_cppConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/piper_cpp
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/piper_cppConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/piper_cppConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/piper_cppConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/piper_cpp
    COMPONENT dev
)

# ------------------------------------------------------------------------------
# Debian packaging (CPack): produces two .deb packages from the install rules above
#   libpiper-cpp-dev -- static library, headers, CMake package config (component: dev)
#   piper-cpp-tools  -- command-line demo / utility executables       (component: tools)
# ------------------------------------------------------------------------------
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "piper-cpp")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "C++ SDK for the AgileX Piper 6-DOF arm")
set(CPACK_PACKAGE_HOMEPAGE_URL "https://github.com/justagist/piper_cpp")
set(CPACK_PACKAGE_CONTACT "Saif Sidhik <mail@saifsidhik.page>")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Saif Sidhik <mail@saifsidhik.page>")

# One .deb per component, named per Debian policy (<name>_<version>_<arch>.deb).
set(CPACK_DEB_COMPONENT_INSTALL ON)
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)   # auto-resolve runtime deps (e.g. libsocketcan2)

# dev component -> libpiper-cpp-dev
set(CPACK_DEBIAN_DEV_PACKAGE_NAME "libpiper-cpp-dev")
set(CPACK_DEBIAN_DEV_PACKAGE_SECTION "libdevel")
set(CPACK_DEBIAN_DEV_PACKAGE_DEPENDS "libsocketcan-dev, pkg-config")
set(CPACK_COMPONENT_DEV_DESCRIPTION
    "Development files (static library, headers, CMake config) for piper_cpp.")

# tools component -> piper-cpp-tools
set(CPACK_DEBIAN_TOOLS_PACKAGE_NAME "piper-cpp-tools")
set(CPACK_DEBIAN_TOOLS_PACKAGE_SECTION "utils")
set(CPACK_DEBIAN_TOOLS_PACKAGE_DEPENDS "libsocketcan2")  # SHLIBDEPS also detects this; explicit = deterministic
set(CPACK_COMPONENT_TOOLS_DESCRIPTION
    "Command-line demo and utility tools for the AgileX Piper arm.")

include(CPack)
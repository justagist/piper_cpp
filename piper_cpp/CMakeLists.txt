
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

target_include_directories(piper_cpp
    PUBLIC include
    PRIVATE ${LIBCANSOCKET_INCLUDE_DIRS}
)

target_link_libraries(piper_cpp
    PRIVATE ${LIBCANSOCKET_LIBRARIES}
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
add_piper_executable(piper_end_pose_demo piper_end_pose_demo.cpp)
add_piper_executable(piper_go_zero piper_go_zero.cpp)
add_piper_executable(piper_set_zero piper_set_zero.cpp)
add_piper_executable(piper_set_role piper_set_role.cpp)
add_piper_executable(piper_read_firmware piper_read_firmware.cpp)

include(GNUInstallDirs)
install(TARGETS piper_cpp
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(TARGETS piper_cpp_can_utils_demo
        piper_cpp_std_can_interface_read
        piper_parser_v2_read_test
        piper_interface_v2_read_test
        can_activate
        piper_enable
        piper_gripper_sweep
        piper_joint_ctrl_demo
        piper_end_pose_demo
        piper_go_zero
        piper_set_zero
        piper_set_role
        piper_read_firmware
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
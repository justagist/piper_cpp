
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

# Helper macro to reduce duplication for test/demo executables
function(add_piper_executable exe_name src_file)
    add_executable(${exe_name} scripts/${src_file})
    target_link_libraries(${exe_name} PRIVATE piper_cpp)
    target_include_directories(${exe_name} PRIVATE include)
endfunction()

# Use macro for all test/demo executables
add_piper_executable(piper_cpp_can_utils_demo can_utils_demo.cpp)
add_piper_executable(piper_cpp_std_can_interface_read std_can_interface_read_test.cpp)
add_piper_executable(piper_parser_v2_read_test piper_parser_v2_read_test.cpp)
add_piper_executable(piper_interface_v2_read_test piper_interface_v2_read_test.cpp)
add_piper_executable(can_activate can_activate.cpp)
add_piper_executable(piper_enable piper_enable.cpp)
add_piper_executable(piper_gripper_sweep piper_gripper_sweep.cpp)

# Install rules
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
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
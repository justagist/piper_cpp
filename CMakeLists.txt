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

# Demo executable for interactive tests
add_executable(piper_cpp_can_utils_demo
    scripts/can_utils_demo.cpp
)
target_link_libraries(piper_cpp_can_utils_demo
    PRIVATE piper_cpp
)

target_include_directories(piper_cpp_can_utils_demo
    PRIVATE include
)
add_executable(piper_cpp_std_can_interface_read
    scripts/std_can_interface_read_test.cpp
)
target_link_libraries(piper_cpp_std_can_interface_read
    PRIVATE piper_cpp
)

target_include_directories(piper_cpp_std_can_interface_read
    PRIVATE include
)

# Add piper_parser_v2 read test executable
add_executable(piper_parser_v2_read_test
    scripts/piper_parser_v2_read_test.cpp
)
target_link_libraries(piper_parser_v2_read_test
    PRIVATE piper_cpp
)

target_include_directories(piper_parser_v2_read_test
    PRIVATE include
)

# Add piper_interface_v2 read test executable
add_executable(piper_interface_v2_read_test
    scripts/piper_interface_v2_read_test.cpp
)
target_link_libraries(piper_interface_v2_read_test
    PRIVATE piper_cpp
)

target_include_directories(piper_interface_v2_read_test
    PRIVATE include
)

add_executable(can_activate
    scripts/can_activate.cpp
)
target_link_libraries(can_activate
    PRIVATE piper_cpp
)

target_include_directories(can_activate
    PRIVATE include
)

add_executable(piper_enable
    scripts/piper_enable.cpp
)
target_link_libraries(piper_enable
    PRIVATE piper_cpp
)

target_include_directories(piper_enable
    PRIVATE include
)

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
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
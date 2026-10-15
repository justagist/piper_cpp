find_package(GTest REQUIRED)
include(GoogleTest)

# A regression in the shutdown path shows up as a hang, so bound every test. GTest::Main is
# the target FindGTest provides on every CMake version this project supports; the newer
# gtest_main target only appeared in 3.20.
function(add_piper_test name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE piper_cpp GTest::Main)
    gtest_discover_tests(${name} PROPERTIES TIMEOUT 30)
endfunction()

add_piper_test(std_can_interface_test)
add_piper_test(piper_interface_v2_test)

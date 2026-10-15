find_package(GTest REQUIRED)
include(GoogleTest)

# A regression in the shutdown path shows up as a hang, so bound every test.
function(add_piper_test name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE piper_cpp GTest::gtest_main)
    gtest_discover_tests(${name} PROPERTIES TIMEOUT 30)
endfunction()

add_piper_test(std_can_interface_test)
add_piper_test(piper_interface_v2_test)

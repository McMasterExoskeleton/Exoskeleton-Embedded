cmake_minimum_required(VERSION 3.14)
project(ExoskeletonFirmware)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable testing
enable_testing()

# Add googletest
add_subdirectory(googletest)

# Motor API library
add_library(motor_api
    motors/motor_api.cpp
)
target_include_directories(motor_api PUBLIC motors)

# Add test for motor API
add_executable(motor_api_test
    tests/motor_api_test.cpp
)
target_link_libraries(motor_api_test
    motor_api
    gtest
    gtest_main
)

add_test(NAME MotorAPITest COMMAND motor_api_test)

# add more components later (sensors, control, etc.)

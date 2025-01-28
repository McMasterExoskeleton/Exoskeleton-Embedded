cmake_minimum_required(VERSION 3.14)
project(MyProject)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Add source files
# add_library(MyLibrary src/my_library.cpp)

# Include GoogleTest
add_subdirectory(googletest)

# Enable testing
enable_testing()

# Add unit tests
add_executable(sampleTest tests/sample_test.cpp)
target_link_libraries(sampleTest PRIVATE gtest gtest_main)

# Register the test with CTest
add_test(NAME sampleTest COMMAND sampleTest)

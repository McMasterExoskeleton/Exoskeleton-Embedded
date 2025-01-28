cmake_minimum_required(VERSION 3.14)
project(MyProject)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Add source files
add_library(MyLibrary src/my_library.cpp)

# Include GoogleTest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/release-1.12.1.zip
)
FetchContent_MakeAvailable(googletest)

# Enable testing
enable_testing()

# Add unit tests
add_executable(MyLibraryTest tests/my_library_test.cpp)
target_link_libraries(MyLibraryTest PRIVATE MyLibrary gtest gtest_main)

# Register the test with CTest
add_test(NAME MyLibraryTest COMMAND MyLibraryTest)

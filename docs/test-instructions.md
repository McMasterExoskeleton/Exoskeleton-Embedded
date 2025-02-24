# How to Add and Run Tests via CLI

This guide explains how to add new tests to the project and run them via the command line.

---

## Adding Tests

### 1. Create a New Test File
- Navigate to the `tests/` directory.
- Create a new test file with a `.cpp` extension, such as `example_test.cpp`.

### 2. Include the GoogleTest Framework
- At the top of your test file, include the GoogleTest header:

    ```cpp
    #include <gtest/gtest.h>
    ```

### 3. Write Your Test Cases
- Use the `TEST` macro to define test cases:

    ```cpp
    int Add(int a, int b) {
        return a + b;
    }

    TEST(ExampleTest, Addition) {
        EXPECT_EQ(Add(2, 3), 5);  // Test that 2 + 3 = 5
        EXPECT_EQ(Add(-1, 1), 0); // Test that -1 + 1 = 0
    }
    ```

### 4. Add Your Test File to CMake
- Open the project's `CMakeLists.txt`.
- Add the new test file to the test executable:

    ```cmake
    add_executable(ExampleTest tests/example_test.cpp) # this builds your test executable
    target_link_libraries(ExampleTest PRIVATE gtest gtest_main) # this links necessary libraries to your test 
    add_test(NAME ExampleTest COMMAND ExampleTest) # this adds your test to CTest
    ```

---

## Building the Tests

### 1. Navigate to the Project Root
- Open a terminal and navigate to the project root directory:

    ```bash
    cd /path/to/project
    ```

### 2. Create and Enter the `build` Directory
- Run the following commands:

    ```bash
    mkdir -p build
    cd build
    ```

### 3. Run CMake to Configure the Build System
- Configure the project:

    ```bash
    cmake ..
    ```

### 4. Build the Project
- Build the project, including all tests:

    ```bash
    cmake --build .
    ```

---

## Running Tests

### 1. Run All Tests
- From the `build/` directory, use `ctest` to run all tests:

    ```bash
    ctest --output-on-failure
    ```

### 2. Run a Specific Test
- Execute a specific test binary directly:

    ```bash
    ./ExampleTest
    ```

### 3. Debugging Test Failures
- If a test fails, you’ll see detailed output with the failure reason and the file/line number where it occurred.

---

## Example Workflow

### Example Test File (`tests/example_test.cpp`)
```cpp
#include <gtest/gtest.h>

int Multiply(int a, int b) {
    return a * b;
}

TEST(MultiplyTest, Basic) {
    EXPECT_EQ(Multiply(2, 3), 6);
    EXPECT_EQ(Multiply(-1, 5), -5);
}

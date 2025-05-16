cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(MainController)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Point to the location of libtorch
set(CMAKE_PREFIX_PATH "/opt/torch")

find_package(Torch REQUIRED)
find_package(nlohmann_json REQUIRED)

# Add the main_controller executable
add_executable(main_controller
    main.cpp
    ../motors/motor_api.cpp
    ../joint_estimator/jointEstimator.cpp
    ../sensors/getSensorData.cpp
    ../models/model.cpp
    clean_angles/clean.cpp
    torque_controller/torqueController.cpp
    ../sensors/data_collection.cpp
    ../sensors/liveSensorData.cpp
    ../sensors/rpi_tca9548a.cpp
    ../sensors/sensor_preprocessing.cpp
    ../sensors/bno055.c
    ../sensors/bno055.h
)

# Link required libraries, including WiringPi
target_link_libraries(main_controller
    "${TORCH_LIBRARIES}"
    nlohmann_json::nlohmann_json
    modbus
    wiringPi
)
# target_include_directories(main_controller PRIVATE ../sensors)
# Set the C++ standard for the target
set_property(TARGET main_controller PROPERTY CXX_STANDARD 17)

/*
 * IMPORTANT: libmodbus is a C library, so we have to be careful and use c-style notation while using C++
 * i.e. use C-style string literals instead of std::string
 * 
 * Q: Why use libmodbus over a C++ library?
 * A: 
 *    - Most commonly used library for modbus RTU in C/C++.
 *    - Amazing Reference guide 
*/

#include <iostream>
#include <modbus/modbus.h>
#include <chrono>
#include <thread>
#include <vector>



//initializeMotor 
void initializeMotor(Motor *motor) {
    // Initialize Modbus connection 
    motor->ctx = modbus_new_rtu(motor->device, 115200, 'E', 8, 1);
    if (motor->ctx == NULL) {
        fprintf(stderr, "Unable to create libmodbus context for %s\n", motor->device);
        return;
    }
    if (modbus_set_slave(motor->ctx, motor->slave_id) == -1) {
        fprintf(stderr, "Invalid slave ID for %s\n", motor->device);
        modbus_free(motor->ctx);
        return;
    }
    if (modbus_connect(motor->ctx) == -1) {
        fprintf(stderr, "Connection failed for %s: %s\n", motor->device, modbus_strerror(errno));
        modbus_free(motor->ctx);
        return;
    }
    printf("Connected to motor on %s (ID: %d)\n", motor->device, motor->slave_id);

    // Now write motor control registers
    write_register(motor->ctx, 0x6600, 0x04, "Profile Torque Mode");
    write_register(motor->ctx, 0x6720, 0x0BB8, "Max Torque (3000‰)"); // values found from documentation
    write_register(motor->ctx, 0x6710, 0x0064 , "Target Torque (100‰)");// set to 100%
    write_register(motor->ctx, 0x6400, 0x0006, "Motor Shutdown");
    write_register(motor->ctx, 0x6400, 0x000F, "On/off Operation");
}

// Function to write a Modbus register
void write_register(modbus_t *ctx, uint16_t reg_addr, uint16_t value, const std::string &desc) {
    if (modbus_write_register(ctx, reg_addr, value) == -1) {
        std::cerr << "Failed to set " << desc << ": " << modbus_strerror(errno) << "\n";
    } else {
        std::cout << desc << " set successfully\n";
    }
}

// motor connection details
struct Motor {
    std::string device;
    int slave_id;
    modbus_t *ctx;

    // Constructor required for emplace_back()
    Motor(const std::string& dev, int id) : device(dev), slave_id(id), ctx(nullptr) {}
};

int main() {

    // Try 1 usb connection first, then connect multiple motors to pi

    // Serial ports + Slave IDs
    std::vector<Motor> motors;
    motors.emplace_back("/dev/ttyUSB0", 1);

    // std::vector<Motor> motors = {
    //     {"/dev/ttyUSB0", 1},
    //     {"/dev/ttyUSB1", 2}
    // };


    // Register addresses
    const uint16_t REG_ADDR_OP_MODE    = 0x6600; 
    const uint16_t REG_ADDR_MAX_TORQUE = 0x6720; 
    const uint16_t REG_ADDR_TRQ_SLOPE  = 0x6870; 
    const uint16_t REG_ADDR_TGT_TORQUE = 0x6710; 
    const uint16_t REG_ADDR_CNTRL_WORD = 0x6400; 

    // Initialize Modbus connections for each motor
    for (auto &motor : motors) {
        motor.ctx = init_modbus(motor.device.c_str(), motor.slave_id);
        if (motor.ctx == nullptr) {
            std::cerr << "Skipping " << motor.device << " due to connection failure.\n";
        }
    }

    // DEBUGGING
    // Check if there is a RS-485 connection issue
    
    uint16_t test_value;
    int rc = modbus_read_registers(motors[0].ctx, REG_ADDR_CNTRL_WORD, 1, &test_value);
    if (rc == -1) {
        std::cerr << "Modbus communication error: " << modbus_strerror(errno) << "\n";
        return -1;
    } else {
        std::cout << "Current Control Word: " << test_value << "\n";
    }

    // Followed simple example from user manual for torque profile mode
    // Run motor in PT torque mode with 10% target torque
    for (auto &motor : motors) {
        if (motor.ctx) {
            write_register(motor.ctx, REG_ADDR_OP_MODE,    0x04,   "Profile Torque Mode");
            write_register(motor.ctx, REG_ADDR_MAX_TORQUE, 0x0BB8, "Max Torque (3000‰)");

            // User manual requires the torque slope to be a uint32_t value. The signature of the libmodbus "write_register" command
            // is uint16_t. Temp solution: Use default value of 3000%
            //write_register(motor.ctx, REG_ADDR_TRQ_SLOPE,  0x03E8, "Torque Slope (1000‰/s)");
            write_register(motor.ctx, REG_ADDR_TGT_TORQUE, 0xA,    "Target Torque (10‰)");
            write_register(motor.ctx, REG_ADDR_CNTRL_WORD, 0x0006, "Motor Shutdown");
            write_register(motor.ctx, REG_ADDR_CNTRL_WORD, 0x000F, "Enable Motor Operation");
        }
    }

    // Run motors for 5 seconds
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Shutdown motors
    for (auto &motor : motors) {
        if (motor.ctx) {
            write_register(motor.ctx, REG_ADDR_CNTRL_WORD, 0x0006, "Motor Shutdown");
        }
    }

    // Close all connections
    for (auto &motor : motors) {
        if (motor.ctx) {
            modbus_close(motor.ctx);
            modbus_free(motor.ctx);
        }
    }

    return 0;
}
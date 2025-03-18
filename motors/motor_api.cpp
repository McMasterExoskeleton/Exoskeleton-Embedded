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

// Initialize Modbus RTU connection
modbus_t* init_modbus(const char* device, int slave_id) {

    // DEBUGGING
    // According to the user manual, the motor expects the data format to have an even parity
    // Old code used     : 'N' (No parity)
    // New code will use : 'E' (Even parity)
    modbus_t *ctx = modbus_new_rtu(device, 115200, 'E', 8, 1);
    if (ctx == nullptr) {
        std::cerr << "Unable to create libmodbus context for " << device << "\n";
        return nullptr;
    }

    // DEBUGGING
    // Use slave_id: 0 instead of 1 (sends commands to ALL connected devices instead of a specific one, think of this as a broadcast address)
    // new code: modbus_set_slave(ctx, 0)
    if (modbus_set_slave(ctx, slave_id) == -1) {
        std::cerr << "Invalid slave ID for " << device << "\n";
        modbus_free(ctx);
        return nullptr;
    }
    if (modbus_connect(ctx) == -1) {
        std::cerr << "Connection failed for " << device << ": " << modbus_strerror(errno) << "\n";
        modbus_free(ctx);
        return nullptr;
    }
    std::cout << "Connected to motor on " << device << " (ID: " << slave_id << ")\n";
    return ctx;
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
/*
 * IMPORTANT: libmodbus is a C library, so we have to be careful and use c-style notation while using C++
 * i.e. use C-style string literals instead of std::string
 * 
 * Q: Why use libmodbus over a C++ library?
 * A: 
 *    - Most commonly used library for modbus RTU in C/C++.
 *    - Amazing Reference guide 
*/

#include "motor_api.h"


// Register addresses
const uint16_t REG_OP_MODE         = 0x6600; 
const uint16_t REG_MAX_TORQUE      = 0x6720; 
const uint16_t REG_TRQ_SLOPE       = 0x6870; 
const uint16_t REG_TARGET_TORQUE   = 0x6710; 
const uint16_t REG_CONTROL_WORD    = 0x6400;
const uint16_t REG_TARGET_SPEED    = 0x6FF0;

const uint16_t OPERATION_ENABLE    = 0x000F;
const uint16_t OPERATION_DISABLE   = 0x0006;

const int16_t PP_MODE             = 1;   //PROFILE POSITION MODE 
const int16_t PV_MODE             = 3;   //PROFILE VELOCITY MODE 
const int16_t PT_MODE             = 4;   //PROFILE TORQUE MODE 

Motor::Motor(const std::string& dev, int id) : device(dev), slave_id(id), ctx(nullptr), connected(false) {}

// Initialize and connect motor
bool Motor::initializeMotor() {
    ctx = modbus_new_rtu(device.c_str(), 115200, 'E', 8, 1);
    if (!ctx) {
        std::cerr << "Failed to create Modbus context for " << device << "\n";
        return false;
    }
    if (modbus_set_slave(ctx, slave_id) == -1) {
        std::cerr << "Invalid slave ID: " << slave_id << "\n";
        modbus_free(ctx);
        return false;
    }
    if (modbus_connect(ctx) == -1) {
        std::cerr << "Failed to connect to " << device << ": " << modbus_strerror(errno) << "\n";
        modbus_free(ctx);
        return false;
    }
    connected = true;
    std::cout << "Connected to motor on " << device << " (ID: " << slave_id << ")\n";

    setMaxTorque(3000);
    writeRegister(REG_CONTROL_WORD, OPERATION_ENABLE); // Enable motor on startup

    return true;
}

// Write a register
bool Motor::writeRegister(uint16_t reg, uint16_t value) {
    if (!connected) return false;
    if (modbus_write_register(ctx, reg, value) == -1) {
        std::cerr << "Modbus write error: " << modbus_strerror(errno) << "\n";
        return false;
    }
    return true;
}

// Read a register
int Motor::readRegister(uint16_t reg) {
    if (!connected) return -1;
    uint16_t value;
    if (modbus_read_registers(ctx, reg, 1, &value) == -1) {
        std::cerr << "Modbus read error: " << modbus_strerror(errno) << "\n";
        return -1;
    }
    return value;
}

// Set motor speed
bool Motor::setSpeed(int speed) {
    return writeRegister(REG_TARGET_SPEED, static_cast<uint16_t>(speed));
}

// Set motor torque
bool Motor::setTorque(int torque) {
    return writeRegister(REG_TARGET_TORQUE, static_cast<uint16_t>(torque));
}

// Set max motor torque
bool Motor::setMaxTorque(int max_torque) {
    return writeRegister(REG_MAX_TORQUE, static_cast<uint16_t>(max_torque));
}

// Set motor operation mode
bool Motor::setOperationMode(int16_t mode) {
    return writeRegister(REG_OP_MODE, static_cast<uint16_t>(mode));
}

// Get motor status
int Motor::getStatus() {
    return readRegister(REG_CONTROL_WORD);
}

// Stop motor
bool Motor::stopMotor() {
    return writeRegister(REG_CONTROL_WORD, OPERATION_DISABLE);
}

// Disconnect motor
void Motor::disconnectMotor() {
    if (!connected) return;    
    stopMotor();
    modbus_close(ctx);
    modbus_free(ctx);
    ctx = nullptr;
    connected = false;
}

// Destructor
Motor::~Motor() {
    disconnectMotor();
}

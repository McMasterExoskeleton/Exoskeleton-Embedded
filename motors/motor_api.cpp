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
#include <thread>

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
    clearError();
    writeRegister(REG_CONTROL_WORD, OPERATION_ENABLE);
    setOperationMode(PT_MODE);
    setMaxTorque(3000);
     // Enable motor on startup

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
bool Motor::readRegister(uint16_t reg, uint16_t &value) {
    if (!connected) {
        std::cerr << "[readRegister] Not connected to motor.\n";
        return false;
    }

    if (modbus_read_registers(ctx, reg, 1, &value) == -1) {
        std::cerr << "[readRegister] Modbus read error at 0x" 
                  << std::hex << reg << ": " 
                  << modbus_strerror(errno) << "\n";
        return false;
    }

    return true;
}

void Motor::logErrorStatus() {
    uint16_t error_status = 0;
    if (!readRegister(REG_ERROR_STATUS, error_status)) {
        std::cerr << "Failed to read Error Status register.\n";
        return;
    }

    std::cout << "=== Motor Error Status (0x20BA) ===\n";
    if (error_status == 0) {
        std::cout << "No errors detected.\n";
        return;
    }

    if (error_status & (1 << 0)) std::cout << "OCP: Over-current protection triggered.\n";
    if (error_status & (1 << 1)) std::cout << "UVLO: Under-voltage lockout triggered.\n";
    if (error_status & (1 << 2)) std::cout << "LOCK: Rotor lock detected.\n";
    if (error_status & (1 << 3)) std::cout << "VIN_LIMIT: DC link voltage protection active.\n";
    if (error_status & (1 << 4)) std::cout << "OVERLOAD: Overload protection triggered.\n";
    if (error_status & (1 << 5)) std::cout << "POS_LIMIT: Position limit exceeded.\n";
    if (error_status & (1 << 6)) std::cout << "SENSOR_ERR: Position sensor error.\n";
    std::cout << "====================================\n";
}


// Write to a register of data type INT32
bool Motor::writeRegister32(uint16_t start_addr, int32_t value) {
    if (!connected) return false;
    uint16_t regs[2] = {
        static_cast<uint16_t>(value & 0xFFFF),
        static_cast<uint16_t>((value >> 16) & 0xFFFF)
    };
    int rc = modbus_write_registers(ctx, start_addr, 2, regs);
    if (rc != 2) {
        std::cerr << "Failed to write 32-bit register: " << modbus_strerror(errno) << "\n";
        return false;
    }
    return true;
}
bool Motor::clearError() {
    // Step 1: Make sure bit 7 has a falling edge first
    if (!writeRegister(REG_CONTROL_WORD, 0x0000)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Step 2: Write 0x0080 to trigger fault reset
    if (!writeRegister(REG_CONTROL_WORD, 0x0080)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Step 3: Shutdown command (get to "Ready to switch on")
    if (!writeRegister(REG_CONTROL_WORD, 0x0006)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return true;
}

// Read a register of data type INT32
bool Motor::readRegister32(uint16_t start_addr, int32_t &out) {
    if (!connected) {
        std::cerr << "[readRegister32] Not connected to motor.\n";
        return false;
    }

    uint16_t regs[2];
    if (modbus_read_registers(ctx, start_addr, 2, regs) != 2) {
        std::cerr << "[readRegister32] Modbus read error at 0x"
                  << std::hex << start_addr << ": "
                  << modbus_strerror(errno) << "\n";
        return false;
    }

    out = (static_cast<int32_t>(regs[1]) << 16) | regs[0];
    return true;
}


bool Motor::setOTPThreshold(int16_t temperature_C){
    return writeRegister(REG_OTP_THRESHOLD, temperature_C);
}



bool Motor::setMotorAdrress(uint16_t adr){
    return writeRegister(REG_MODBUS_ADDRESS, adr);
}

bool Motor::getMotorAddress(uint16_t &adr){
    uint16_t address = 0;
    if (!readRegister(REG_MODBUS_ADDRESS, address)) {
        return false; 
    }
    adr = address;
    return true;
}

bool Motor::getMotorTemperature(int16_t &temperature_C){
    int16_t temp = 0;
    if (!readRegister(REG_TEMPERATURE, temp)) {
        return false;  
    }
    temperature_C = temp;
    return true;
}

bool Motor::storeParameters(){
    return writeRegister32(REG_RELATED_REGISTERS, STORE_PARAMETERS);
}

bool Motor::updateControlParameters(){
    return writeRegister32(REG_RELATED_REGISTERS, UPDATE_CONTROL_PARAMETERS);
}

bool Motor::restoreDefaultParameters(){
    return writeRegister32(REG_RELATED_REGISTERS, RESTORE_DEFAULT_PARAMETERS);
}

bool Motor::systemReset(){
    return writeRegister32(REG_RELATED_REGISTERS, SYSTEM_RESET);
}

void Motor::printMotorAddress() {
    uint16_t address = 0;
    if (getMotorAddress(address)) {
        std::cout << "Motor on " << device << " has address: " << address << std::endl;
    } else {
        std::cerr << "Failed to retrieve motor address for " << device << std::endl;
    }
}

void Motor::printMotorTemperature() {
    int16_t temperature = 0;
    if (getMotorTemperature(temperature)) {
        std::cout << "Motor on " << device << " has temperature: " << temperature << "°C" << std::endl;
    } else {
        std::cerr << "Failed to retrieve motor temperature for " << device << std::endl;
    }
}

// Read raw current in ‰ (thousandths of rated torque)
int16_t Motor::getActualCurrent() {
    uint16_t raw_current;
    if (!readRegister(REG_ACTUAL_CURRENT, raw_current)) {
        return 0;
    }
    return static_cast<int16_t>(raw_current);  // safe cast to signed
}

// TODO: DOUBLE CHECK BELOW CONSTANTS
// Get actual current in milliamps 
double Motor::getActualCurrent_mA() {
    // Ritvik: Current approach causes infinite reccursion
    // I'm assuming this is a typo and getActualCurrent() was meant to be called
    int16_t current_per_mille = getActualCurrent();

    // Select rated torque based on motor model (update accordingly)
    const double rated_torque = 0.64;  // for 200W motor. in Nm
    const double torque_constant = 0.056;  // 56 mNm/A from user guide

    // Convert current value from per-mille of rated torque to Nm
    double actual_torque = (current_per_mille / 1000.0) * rated_torque;

    double current_mA = actual_torque * 1000.0 / torque_constant;
    return current_mA;
}

// Set motor position
bool Motor::setPosition(int32_t position) {
    return writeRegister32(REG_TARGET_POSITION, position);
}

// Get actual motor position
int32_t Motor::getActualPosition() {
    int32_t position;
    if (!readRegister32(REG_ACTUAL_POSITION, position)) {
        return 0;
    }
    return position;
}

// Get actual motor position in degrees
double Motor::getDegrees(){
    int32_t  actual_position_in_INC = Motor::getActualPosition();
    double degrees = (actual_position_in_INC * 360.0) / 65536.0;
    return degrees;
}

// Set motor target velocity
bool Motor::setTargetVelocity(int32_t velocity) {
    return writeRegister32(REG_TARGET_VELOCITY, velocity);
}

// Get actual motor velocity
int32_t Motor::getActualVelocity() {
    int32_t velocity;
    if (!readRegister32(REG_ACTUAL_VELOCITY, velocity)) {
        return 0;
    }
    return velocity;
}

// Set motor target torque
bool Motor::setTargetTorque(int16_t torque_permille) {
    if (torque_permille < -3000 || torque_permille > 3000) {
        std::cerr << "Torque value " << torque_permille << "‰ out of range [-3000, 3000]\n";
        return false;
    }
    uint16_t raw = static_cast<uint16_t>(*reinterpret_cast<uint16_t*>(&torque_permille));
    return writeRegister(REG_TARGET_TORQUE, raw);
}

// Get actual motor torque
int16_t Motor::getActualTorque() {
    uint16_t torque = 0;
    if (!readRegister(REG_ACTUAL_TORQUE, torque)) {
        return 0;
    }
    return static_cast<int16_t>(torque);
}

// Set max motor torque
bool Motor::setMaxTorque(uint16_t max_torque) {
    if (max_torque > 3000) {
        std::cerr << "Max torque exceeds 3000‰ limit.\n";
        return false;
    }
    return writeRegister(REG_MAX_TORQUE, max_torque);
}

// Set motor operation mode
bool Motor::setOperationMode(int16_t mode) {
    return writeRegister(REG_OP_MODE, static_cast<uint16_t>(mode));
}

// Get motor status
uint16_t Motor::getStatus() {
    uint16_t status = 0;
    if (!readRegister(REG_CONTROL_WORD, status)) {
        return 0;
    }
    return status;
}



// Stop motor
bool Motor::stopMotor() {
    return writeRegister(REG_CONTROL_WORD, OPERATION_DISABLE);
}

// Disconnect motor
bool Motor::disconnectMotor() {
    std::cout << "Disconnecting motor on " << device << " (ID: " << slave_id << ")\n";
    if (connected) {
        stopMotor();
        modbus_close(ctx);
        modbus_free(ctx);
        ctx = nullptr;
        connected = false;
        return true;
    }
    return false;
}

// Destructor
Motor::~Motor() {
    disconnectMotor();
}
#ifndef MOTOR_API_H
#define MOTOR_API_H

#include <modbus/modbus.h>
#include <iostream>
#include <vector>

// ─────────────────────────────────────────────────────────────
// Control Modes
constexpr int16_t PP_MODE = 1;  // Profile Position
constexpr int16_t PV_MODE = 3;  // Profile Velocity
constexpr int16_t PT_MODE = 4;  // Profile Torque

// ─────────────────────────────────────────────────────────────
// Control Words
constexpr uint16_t OPERATION_ENABLE  = 0x000F;
constexpr uint16_t OPERATION_DISABLE = 0x0006;

// ─────────────────────────────────────────────────────────────
// Modbus Register Addresses
constexpr uint16_t REG_OP_MODE          = 0x6600;
constexpr uint16_t REG_CONTROL_WORD     = 0x6400;

constexpr uint16_t REG_TARGET_TORQUE    = 0x6710;
constexpr uint16_t REG_TARGET_VELOCITY  = 0x6FF0;
constexpr uint16_t REG_TARGET_POSITION  = 0x67A0;

constexpr uint16_t REG_MAX_TORQUE       = 0x6720;
constexpr uint16_t REG_TRQ_SLOPE        = 0x6870;

constexpr uint16_t REG_ACTUAL_TORQUE    = 0x6770;
constexpr uint16_t REG_ACTUAL_VELOCITY  = 0x66C0;
constexpr uint16_t REG_ACTUAL_POSITION  = 0x6640;
constexpr uint16_t REG_ACTUAL_CURRENT   = 0x6780;

constexpr uint16_t REG_RATED_CURRENT    = 0x2017;



// ─────────────────────────────────────────────────────────────
// Motor Class
class Motor {
public:
    // Constructor / Destructor
    Motor(const std::string& device, int slave_id);
    ~Motor();

    // Initialization
    bool initializeMotor();
    void disconnectMotor();

    // Motor Control
    bool setOperationMode(int16_t mode);
    bool stopMotor();

    bool setMaxTorque(uint16_t max_torque);
    bool setTargetTorque(int16_t torque_permille);
    int16_t  getActualTorque();

    bool setTargetVelocity(int32_t velocity);
    int32_t getActualVelocity();

    bool setPosition(int32_t position);
    int32_t getActualPosition();
    double getDegrees();
    double getDegrees();

    bool setRatedCurrent(uint32_t current_mA);
    uint32_t getRatedCurrent();
    int16_t getActualCurrent();
    double getActualCurrent_mA();

    // Status
    uint16_t getStatus();
    uint16_t getStatus();
    bool isConnected() const { return connected; }

private:
    std::string device;
    int slave_id;
    modbus_t* ctx;
    bool connected;

    // Register Access Helpers
    bool writeRegister(uint16_t reg, uint16_t value);
    bool readRegister(uint16_t reg, uint16_t &value);

    // For 32-bit register access
    bool writeRegister32(uint16_t start_addr, int32_t value);
    bool readRegister32(uint16_t start_addr, int32_t &out);
    bool readRegister32(uint16_t start_addr, int32_t &out);
};

#endif // MOTOR_API_H

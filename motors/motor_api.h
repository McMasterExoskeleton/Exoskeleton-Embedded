#ifndef MOTOR_API_H
#define MOTOR_API_H

#include <modbus/modbus.h>
#include <iostream>
#include <vector>

class Motor {
public:
    // Constructor
    Motor(const std::string& device, int slave_id);
    
    // Initialization and connection
    bool initializeMotor();
    void disconnectMotor();

    // Control functions
    bool setPosition(int32_t position);
    int32_t getActualPosition();

    bool setSpeed(int speed);
    int  getActualSpeed()

    bool setTorque(int torque);
    int  getActualTorque()
    bool setMaxTorque(int torque);
    
    bool setOperationMode(int16_t mode);
    bool stopMotor();
    
    // Error handling
    int getStatus();

    ~Motor(); // Destructor to clean up

private:
    std::string device;
    int slave_id;
    modbus_t* ctx;
    bool connected;

    // Private helper functions
    bool writeRegister(uint16_t reg, uint16_t value);
    bool writeRegister32(uint16_t start_addr, int32_t value);
    int readRegister(uint16_t reg);
    int32_t readRegister32(uint16_t start_addr);
};

#endif // MOTOR_API_H

#include <iostream>
#include <cassert>

class MotorControl {
public:
    MotorControl() : currentSpeed(0) {}

    void setSpeed(int speed) {
        if (speed < 0) speed = 0;
        if (speed > 100) speed = 100;
        currentSpeed = speed;
    }

    int getSpeed() const {
        return currentSpeed;
    }

private:
    int currentSpeed;
};

void testMotorControl() {
    MotorControl motor;

    // Test case 1: Set speed to 50
    motor.setSpeed(50);
    assert(motor.getSpeed() == 50);
    std::cout << "Test case 1 passed: Speed set to 50." << std::endl;

    // Test case 2: Set speed to 150 (should clamp to 100)
    motor.setSpeed(150);
    assert(motor.getSpeed() == 100);
    std::cout << "Test case 2 passed: Speed clamped to 100." << std::endl;

    // Test case 3: Set speed to -10 (should clamp to 0)
    motor.setSpeed(-10);
    assert(motor.getSpeed() == 0);
    std::cout << "Test case 3 passed: Speed clamped to 0." << std::endl;
}

int main() {
    testMotorControl();
    std::cout << "All test cases passed!" << std::endl;
    return 0;
}
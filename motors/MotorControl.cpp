#include <iostream>

class MotorControl {
public:
    MotorControl() : currentSpeed(0) {}

    void setSpeed(int speed) {
        if (speed < 0) speed = 0;
        if (speed > 100) speed = 100; // Clamp speed between 0 and 100
        currentSpeed = speed;
    }

    int getSpeed() const {
        return currentSpeed;
    }

private:
    int currentSpeed;
};
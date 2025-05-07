#include "../motors/motor_api.h"
#include <gtest/gtest.h>
#include <chrono>
#include <thread>

#define HARDWARE_AVAILABLE true  

class MotorTest : public ::testing::Test {
protected:
    Motor* motor;

    void SetUp() override {
        if (HARDWARE_AVAILABLE) {
            motor = new Motor("/dev/ttyUSB0", 1);
            ASSERT_TRUE(motor->initializeMotor()); // Ensure connection is successful
        } else {
            motor = nullptr;
            GTEST_SKIP() << "Skipping test: Hardware not available.";
        }
    }

    void TearDown() override {
        if (motor) {
            std::this_thread::sleep_for(std::chrono::seconds(3)); // Wait only if motor exists
            motor->disconnectMotor();
            delete motor;
        }
    }
};

// Test Initialization
// TEST_F(MotorTest, Initialization) {
//     EXPECT_NE(motor, nullptr);  // Ensure motor was created
// }

// // Test Setting Speed
// TEST_F(MotorTest, SetSpeed) {
//     ASSERT_NE(motor, nullptr);
//     EXPECT_TRUE(motor->setOperationMode(PV_MODE)); // Set to Profile Velocity Mode
//     int speed = 1000;
//     EXPECT_TRUE(motor->setTargetVelocity(speed));
//     // std::this_thread::sleep_for(std::chrono::seconds(2));
//     ASSERT_EQ(speed, motor->getActualVelocity()) << "Target Velocity has not been reached";
// }

// Test Setting Torque
TEST_F(MotorTest, SetTorque) {
    ASSERT_NE(motor, nullptr);
    EXPECT_TRUE(motor->setOperationMode(PT_MODE)); // Set to Profile Torque Mode
    // EXPECT_TRUE(motor->setTargetTorque(-200));
    int torque = 100;
    EXPECT_TRUE(motor->setTargetTorque(torque));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_EQ(torque, motor->getActualTorque()) << "Target Torque has not been reached";
    
}

// Test Stopping Motor
TEST_F(MotorTest, StopMotor) {
    ASSERT_NE(motor, nullptr);
    EXPECT_TRUE(motor->stopMotor());
}

// Test Disconnecting Motor
TEST_F(MotorTest, DisconnectMotor) {
    if (motor) {
        motor->disconnectMotor();
        EXPECT_FALSE(motor->isConnected()); // Ensure motor is disconnected
    } else {
        GTEST_SKIP() << "Skipping test: Motor was not initialized.";
    }
}

// Main function for Google Test
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#include "driver.h"
#include <iostream>

// #define BNO055_I2C_ADDR1 0x29  // Default I2C address

s8 I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    int fd = wiringPiI2CSetup(dev_addr);
    if (fd == -1) {
        std::cerr << "Failed to initialize I2C device" << std::endl;
        return BNO055_ERROR;
    }
    for (int i = 0; i < cnt; i++) {
        if (wiringPiI2CWriteReg8(fd, reg_addr + i, reg_data[i]) == -1) {
            std::cerr << "I2C write failed" << std::endl;
            return BNO055_ERROR;
        }
    }
    return BNO055_SUCCESS;
}

s8 I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    int fd = wiringPiI2CSetup(dev_addr);
    if (fd == -1) {
        std::cerr << "Failed to initialize I2C device" << std::endl;
        return BNO055_ERROR;
    }
    for (int i = 0; i < cnt; i++) {
        reg_data[i] = wiringPiI2CReadReg8(fd, reg_addr + i);
        if (reg_data[i] == -1) {
            std::cerr << "I2C read failed" << std::endl;
            return BNO055_ERROR;
        }
    }
    return BNO055_SUCCESS;
}

void delay_msec(u32 msec) {
    usleep(msec * 1000);
}

// Initialize the IMU sensor (Assign function pointers and set device address)
s8 initialize_imu(bno055_t* imu, u8 dev_addr) {
    imu->bus_write  = I2C_bus_write;
    imu->bus_read   = I2C_bus_read;
    imu->delay_msec = delay_msec;
    imu->dev_addr   = dev_addr;

    if (bno055_init(imu) != BNO055_SUCCESS) {
        std::cerr << "BNO055 initialization failed!" << std::endl;
        return BNO055_ERROR;
    }
    return BNO055_SUCCESS;
}

// Configure the IMU operation mode
s8 setup_imu(bno055_t* imu) {
    if (bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF) != BNO055_SUCCESS) {
        std::cerr << "Failed to set operation mode!" << std::endl;
        return BNO055_ERROR;
    }
    delay_msec(BNO055_MODE_SWITCHING_DELAY);
    return BNO055_SUCCESS;
}

// Helper functions for reading sensor data
s8 read_euler_angles(bno055_t* imu, bno055_euler_double_t* euler) {
    if (bno055_convert_double_euler_hpr_deg(euler) != BNO055_SUCCESS) {
        std::cerr << "Failed to read Euler angles!" << std::endl;
        return BNO055_ERROR;
    }
    return BNO055_SUCCESS;
}

s8 read_acceleration(bno055_t* imu, bno055_accel_double_t* accel) {
    if (bno055_convert_double_accel_xyz_msq(accel) != BNO055_SUCCESS) {
        std::cerr << "Failed to read acceleration data!" << std::endl;
        return BNO055_ERROR;
    }
    return BNO055_SUCCESS;
}

s8 read_linear_acceleration(bno055_t* imu, bno055_linear_accel_double_t* linear_accel) {
    if (bno055_convert_double_linear_accel_xyz_msq(linear_accel) != BNO055_SUCCESS) {
        std::cerr << "Failed to read linear acceleration data!" << std::endl;
        return BNO055_ERROR;
    }
    return BNO055_SUCCESS;
}

s8 read_gravity(bno055_t* imu, bno055_gravity_double_t* gravity) {
    if (bno055_convert_double_gravity_xyz_msq(gravity) != BNO055_SUCCESS) {
        std::cerr << "Failed to read gravity data!" << std::endl;
        return BNO055_ERROR;
    }
    return BNO055_SUCCESS;
}

s8 read_gyro(bno055_t* imu, bno055_gyro_double_t* gyro) {
    if (bno055_convert_double_gyro_xyz_dps(gyro) != BNO055_SUCCESS) {
        std::cerr << "Failed to read gyro data!" << std::endl;
        return BNO055_ERROR;
    }
    return BNO055_SUCCESS;
}

s8 read_magnetic_field(bno055_t* imu, bno055_mag_double_t* mag) {
    if (bno055_convert_double_mag_xyz_uT(mag) != BNO055_SUCCESS) {
        std::cerr << "Failed to read magnetic field data!" << std::endl;
        return BNO055_ERROR;
    }
    return BNO055_SUCCESS;
}

s8 read_quaternion(bno055_t* imu, bno055_quaternion_t* quat) {
    if (bno055_read_quaternion_wxyz(quat) != BNO055_SUCCESS) {
        std::cerr << "Failed to read quaternion data!" << std::endl;
        return BNO055_ERROR;
    }
    return BNO055_SUCCESS;
}

// Helper function for printing sensor data
void print_sensor_data(
    const bno055_euler_double_t& euler, 
    const bno055_accel_double_t& accel, 
    const bno055_linear_accel_double_t& linear_accel, 
    const bno055_gravity_double_t& gravity,
    const bno055_gyro_double_t& gyro,
    const bno055_mag_double_t& mag,
    const bno055_quaternion_t& quat
) {
    std::cout << "\n====== Sensor Readings ======\n";
    std::cout << "Euler Angles (deg): Heading=" << euler.h << ", Roll=" << euler.r << ", Pitch=" << euler.p << std::endl;
    std::cout << "Acceleration (m/s²): X=" << accel.x << ", Y=" << accel.y << ", Z=" << accel.z << std::endl;
    std::cout << "Linear Acceleration (m/s²): X=" << linear_accel.x << ", Y=" << linear_accel.y << ", Z=" << linear_accel.z << std::endl;
    std::cout << "Gravity (m/s²): X=" << gravity.x << ", Y=" << gravity.y << ", Z=" << gravity.z << std::endl;
    std::cout << "Gyroscope (°/s): X=" << gyro.x << ", Y=" << gyro.y << ", Z=" << gyro.z << std::endl;
    std::cout << "Magnetic Field (uT): X=" << mag.x << ", Y=" << mag.y << ", Z=" << mag.z << std::endl;
    std::cout << "Quaternion: W=" << quat.w << ", X=" << quat.x << ", Y=" << quat.y << ", Z=" << quat.z << std::endl;
    std::cout << "=============================\n\n";
}

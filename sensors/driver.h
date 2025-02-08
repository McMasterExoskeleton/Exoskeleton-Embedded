#ifndef DRIVER_H
#define DRIVER_H

#include <wiringPiI2C.h>
#include <unistd.h>
#include "bno055.h"

// Function prototypes
s8 I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 initialize_imu(bno055_t* imu, u8 dev_addr);
s8 setup_imu(bno055_t* imu);
void delay_msec(u32 msec);
void print_sensor_data(
    const bno055_euler_double_t& euler, 
    const bno055_accel_double_t& accel, 
    const bno055_linear_accel_double_t& linear_accel, 
    const bno055_gravity_double_t& gravity,
    const bno055_gyro_double_t& gyro,
    const bno055_mag_double_t& mag,
    const bno055_quaternion_t& quat
);

s8 read_euler_angles(bno055_t* imu, bno055_euler_double_t* euler);
s8 read_acceleration(bno055_t* imu, bno055_accel_double_t* accel);
s8 read_linear_acceleration(bno055_t* imu, bno055_linear_accel_double_t* linear_accel);
s8 read_gravity(bno055_t* imu, bno055_gravity_double_t* gravity);
s8 read_gyro(bno055_t* imu, bno055_gyro_double_t* gyro);
s8 read_magnetic_field(bno055_t* imu, bno055_mag_double_t* mag);
s8 read_quaternion(bno055_t* imu, bno055_quaternion_t* quat);

#endif // DRIVER_H

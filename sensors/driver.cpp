#include <iostream>
#include <wiringPiI2C.h>
#include <unistd.h>
#include "bno055.h"

// Define the I2C address for the BNO055 sensor
#define BNO055_I2C_ADDR1 0x29

// I2C communication functions using Raspberry Pi's I2C
s8 I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    int fd = wiringPiI2CSetup(dev_addr);  // Initialize I2C device with the given address
    if (fd == -1) {
        std::cerr << "Failed to initialize I2C device" << std::endl;
        return BNO055_ERROR;
    }

    // Write data to the register
    for (int i = 0; i < cnt; i++) {
        if (wiringPiI2CWriteReg8(fd, reg_addr + i, reg_data[i]) == -1) {
            std::cerr << "I2C write failed" << std::endl;
            return BNO055_ERROR;
        }
    }
    return BNO055_SUCCESS;
}

s8 I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    int fd = wiringPiI2CSetup(dev_addr);  // Initialize I2C device with the given address
    if (fd == -1) {
        std::cerr << "Failed to initialize I2C device" << std::endl;
        return BNO055_ERROR;
    }

    // Read data from the register
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
    usleep(msec * 1000);  // Convert milliseconds to microseconds
}

int main() { //initialize the variables for all of the different data types, note that we are initialziing the double types which are the values that have units (not just raw)
    bno055_t bno055;
    bno055_euler_double_t euler_angles;
    bno055_accel_double_t accel_xyz;
    bno055_linear_accel_double_t linear_accel_xyz;
    bno055_gravity_double_t gravity_xyz;
    bno055_gyro_double_t angular_velocity_xyz;
    bno055_mag_double_t mag_field_xyz; //magnetic field
    bno055_quaternion_t quater_wxyz; //quarternion coords

    // Initialize wiringPi for I2C setup
   // if (wiringPiI2CSetup() == -1) {
     //   std::cerr << "Failed to initialize wiringPi" << std::endl;
      //  return -1;
    //}

    // Configure I2C communication functions
    bno055.bus_write = I2C_bus_write;
    bno055.bus_read = I2C_bus_read;
    bno055.delay_msec = delay_msec;
    bno055.dev_addr = BNO055_I2C_ADDR1;  // Use the appropriate I2C address

    // Initialize BNO055 
    if (bno055_init(&bno055) != BNO055_SUCCESS) {
        std::cerr << "BNO055 initialization failed!" << std::endl;
        return -1;
    }

    // Set the operation mode to NDOF (absolute orientation)
    if (bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF) != BNO055_SUCCESS) {
        std::cerr << "Failed to set operation mode!" << std::endl;
        return -1;
    }

    // Delay to allow sensor to switch modes
    delay_msec(BNO055_MODE_SWITCHING_DELAY);
    while(1){
        // Read Euler angles
      if (bno055_convert_double_euler_hpr_deg(&euler_angles) != BNO055_SUCCESS) {
            std::cerr << "Failed to read Euler angles!" << std::endl;
            return -1;
    }   
    
    if (bno055_convert_double_accel_xyz_msq(&accel_xyz) != BNO055_SUCCESS) {
            std::cerr << "Failed to read acceleration data!" << std::endl;
            return -1;
    }  
    
     if (bno055_convert_double_linear_accel_xyz_msq(&linear_accel_xyz) != BNO055_SUCCESS) {
            std::cerr << "Failed to read linear acceleration data!" << std::endl;
            return -1;
    }  
    
    if (bno055_convert_double_gravity_xyz_msq(&gravity_xyz) != BNO055_SUCCESS) {
            std::cerr << "Failed to read gravity data!" << std::endl;
            return -1;
    } 
    
    if (bno055_convert_double_gyro_xyz_dps(&angular_velocity_xyz) != BNO055_SUCCESS) {
            std::cerr << "Failed to read gravity data!" << std::endl;
            return -1;
    } 
    
    if (bno055_convert_double_mag_xyz_uT(&mag_field_xyz) != BNO055_SUCCESS) {
            std::cerr << "Failed to read gravity data!" << std::endl;
            return -1;
    } 
    
    if (bno055_read_quaternion_wxyz(&quater_wxyz) != BNO055_SUCCESS) {
            std::cerr << "Failed to read gravity data!" << std::endl;
            return -1;
    }
    

        // print euler angles
        std::cout << "Heading: " << euler_angles.h << std::endl;
        std::cout << "Roll: " << euler_angles.r << std::endl;
        std::cout << "Pitch: " << euler_angles.p << std::endl;
        
        std::cout << "                  "<< std::endl;
       
        //print acceleration data
        std::cout << "x accel: " << accel_xyz.x << std::endl;
        std::cout << "y accel: " << accel_xyz.y << std::endl;
        std::cout << "z accel: " << accel_xyz.z << std::endl;
        
        std::cout << "                  "<< std::endl;
        
        //print linear acceleration data
        std::cout << "x linear accel: " << linear_accel_xyz.x << std::endl;
        std::cout << "y linear accel: " << linear_accel_xyz.y << std::endl;
        std::cout << "z linea accel: " << linear_accel_xyz.z << std::endl;
        
        std::cout << "                  "<< std::endl;
        
        //print gravity data
        std::cout << "x gravity: " << gravity_xyz.x << std::endl;
        std::cout << "y gravity: " << gravity_xyz.y << std::endl;
        std::cout << "z gravity: " << gravity_xyz.z << std::endl;
        
         std::cout << "                  "<< std::endl;
        
         //print angular velocity data
        std::cout << "x angular velocity: " << angular_velocity_xyz.x << std::endl;
        std::cout << "y angular velocity: " << angular_velocity_xyz.y << std::endl;
        std::cout << "z angular velocity: " << angular_velocity_xyz.z << std::endl;
        
        std::cout << "                  "<< std::endl;
        
         //print magnetic field data
        std::cout << "x mag field: " <<mag_field_xyz.x << std::endl;
        std::cout << "y mag field: " << mag_field_xyz.y << std::endl;
        std::cout << "z mag field: " << mag_field_xyz.z << std::endl;
        

        std::cout << "                  "<< std::endl;
        
         //print quaternion coordinates
        std::cout << "w quater: " <<quater_wxyz.w << std::endl;
        std::cout << "x quater: " <<quater_wxyz.x << std::endl;
        std::cout << "y quater: " << quater_wxyz.y << std::endl;
        std::cout << "z quater: " << quater_wxyz.z << std::endl;
       
        std::cout << " ------------- "<< std::endl;
        
        usleep(1000000);
}
    return 0;
}

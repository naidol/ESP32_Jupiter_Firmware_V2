//#######################################################################################################
// Name:             imu_bno055.cpp
// Purpose:          Bosch IMU bno055 module setup and read sensor data
// Description:      to be used with esp32 firmware and micro-ros
// Related Files:    uses the onboard oled (oled_ssd1306.h) to display imu data     
// Author:           logan naidoo, south africa, 2024
//########################################################################################################

#include "imu_bno055.h"
// BNO055 IMU pre-built OEM files
#include <Adafruit_BNO055.h>
#include <Adafruit_BusIO_Register.h>
#include <sensor_msgs/msg/imu.h>

// Define IMU object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup_imu(sensor_msgs__msg__Imu* imu_msg) {
    // Initialize I2C communication
    Wire.begin(21, 22);

    // Initialize the BNO055 sensor
    if (!bno.begin()) {
        Serial.println("No BNO055 detected. Check your wiring or I2C ADDR!");
        while (1);
    }

    // Set up the BNO055 sensor
    bno.setExtCrystalUse(true);

    imu_msg->orientation_covariance[0] = -1;

    imu_msg->orientation_covariance[0] = -1; // Indicates orientation covariance is not provided
    imu_msg->angular_velocity_covariance[0] = -1; // Indicates angular velocity covariance is not provided
    imu_msg->linear_acceleration_covariance[0] = -1; // Indicates linear acceleration covariance is not provided
}

// Get IMU data and update the &imu_msg and refresh the Oled display with latest IMU data 
void get_imu_data(sensor_msgs__msg__Imu* imu_msg) {
    // Get the BNO055 calibration status
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    // Read the 9DOF sensor data
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    /* Read the current temperature from BNO055 */
    int8_t temp = bno.getTemp();

    // Read the BNO055 Quaternion
    imu::Quaternion quat = bno.getQuat();

    // Fill the ROS2 message for IMU data
    imu_msg->header.stamp.nanosec = (uint32_t)(millis() * 1e6);
    imu_msg->header.stamp.sec = (uint32_t)(millis() / 1000);
    imu_msg->header.frame_id.data = (char*)("imu_link");

    imu_msg->orientation.x = quat.x();
    imu_msg->orientation.y = quat.y();
    imu_msg->orientation.z = quat.z();
    imu_msg->orientation.w = quat.w(); 

    imu_msg->angular_velocity.x = angVelocityData.gyro.x;
    imu_msg->angular_velocity.y = angVelocityData.gyro.y;
    imu_msg->angular_velocity.z = angVelocityData.gyro.z;

    imu_msg->linear_acceleration.x = linearAccelData.acceleration.x;
    imu_msg->linear_acceleration.y = linearAccelData.acceleration.y;
    imu_msg->linear_acceleration.z = linearAccelData.acceleration.z;

    // Display IMU data on OLED
    display_oled_imu_data(temp, orientationData.acceleration.heading,
                          orientationData.acceleration.pitch,
                          orientationData.acceleration.roll,
                          gravityData.gyro.x,
                          gravityData.gyro.y,
                          gravityData.gyro.z,
                          system, gyro, accel, mag);
    
}

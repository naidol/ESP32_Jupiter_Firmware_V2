#include "kinematics.h"

// Constructor to initialize wheel radius and wheel separation
Kinematics::Kinematics(float wheel_radius, float wheel_separation)
    : wheel_radius_(wheel_radius), wheel_separation_(wheel_separation) {}

// Function to calculate wheel speeds (m/s) based on input linear and angular velocities
Kinematics::WheelSpeeds Kinematics::computeWheelSpeeds(float linear_velocity_x, float angular_velocity_z) {
    WheelSpeeds speeds;

    // Calculate the individual wheel speeds based on the robot's linear and angular velocity
    speeds.front_left_speed = linear_velocity_x - (angular_velocity_z * wheel_separation_ / 2.0);
    speeds.front_right_speed = linear_velocity_x + (angular_velocity_z * wheel_separation_ / 2.0);
    speeds.back_left_speed = speeds.front_left_speed;
    speeds.back_right_speed = speeds.front_right_speed;

    return speeds;
}

// Function to calculate wheel RPMs based on input linear and angular velocities
Kinematics::MotorRPM Kinematics::calculateRPM(float linear_velocity_x, float angular_velocity_z) {
    MotorRPM rpm;

    // First calculate the wheel speeds using the computeWheelSpeeds function
    WheelSpeeds speeds = computeWheelSpeeds(linear_velocity_x, angular_velocity_z);

    // Convert the wheel speeds (m/s) to RPM
    rpm.front_left_rpm = (speeds.front_left_speed / (2.0 * M_PI * wheel_radius_)) * 60.0;
    rpm.front_right_rpm = (speeds.front_right_speed / (2.0 * M_PI * wheel_radius_)) * 60.0;
    rpm.back_left_rpm = (speeds.back_left_speed / (2.0 * M_PI * wheel_radius_)) * 60.0;
    rpm.back_right_rpm = (speeds.back_right_speed / (2.0 * M_PI * wheel_radius_)) * 60.0;

    return rpm;
}

// Function to calculate robot velocities based on wheel RPMs
Kinematics::Velocities Kinematics::getVelocities(float front_left_rpm, float front_right_rpm, float back_left_rpm, float back_right_rpm) {
    // Convert RPM to linear velocity (m/s) for each wheel
    float front_left_velocity = (front_left_rpm * 2.0f * M_PI * wheel_radius_) / 60.0f;
    float front_right_velocity = (front_right_rpm * 2.0f * M_PI * wheel_radius_) / 60.0f;
    float back_left_velocity = (back_left_rpm * 2.0f * M_PI * wheel_radius_) / 60.0f;
    float back_right_velocity = (back_right_rpm * 2.0f * M_PI * wheel_radius_) / 60.0f;

    // Calculate the average linear velocity (assuming differential drive, so left and right side)
    float linear_velocity_x = (front_left_velocity + back_left_velocity + front_right_velocity + back_right_velocity) / 4.0f;

    // Calculate the angular velocity around the z-axis (yaw)
    // (left wheels velocity - right wheels velocity) / wheel separation gives angular velocity in rad/s
    float angular_velocity_z = ((back_right_velocity + front_right_velocity) - (back_left_velocity + front_left_velocity)) / wheel_separation_;

    // Return the computed velocities
    Velocities velocities;
    velocities.linear_velocity_x = linear_velocity_x;
    velocities.linear_velocity_y = 0.0f;  // Assuming no lateral movement (sideways velocity)
    velocities.angular_velocity_z = angular_velocity_z;
    
    return velocities;
}

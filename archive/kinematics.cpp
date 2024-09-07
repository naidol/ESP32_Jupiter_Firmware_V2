#include "kinematics.h"

// Constructor: initialize with wheel radius and separation
Kinematics::Kinematics(double wheel_radius, double wheel_separation)
    : wheel_radius_(wheel_radius), wheel_separation_(wheel_separation) {}

// Function to compute wheel speeds from linear and angular velocities
void Kinematics::computeWheelSpeeds(double linear_velocity, double angular_velocity, double& front_left_wheel_speed, double& front_right_wheel_speed, double& back_left_wheel_speed, double& back_right_wheel_speed) {
    
    // Calculate the linear velocity contribution for each side (left and right)
    double linear_velocity_left = linear_velocity - (angular_velocity * wheel_separation_ / 2.0);
    double linear_velocity_right = linear_velocity + (angular_velocity * wheel_separation_ / 2.0);

    // Convert the linear velocities to wheel speeds (angular velocity in radians per second)
    front_left_wheel_speed = linear_velocity_left / wheel_radius_;
    front_right_wheel_speed = linear_velocity_right / wheel_radius_;
    back_left_wheel_speed = front_left_wheel_speed;  // For a four-wheel robot with differential drive, rear wheels follow the front wheels
    back_right_wheel_speed = front_right_wheel_speed;
}

#ifndef KINEMATICS_H
#define KINEMATICS_H
#include <Arduino.h>

class Kinematics {
public:
    // Constructor: initialize with wheel radius and separation
    Kinematics(double wheel_radius, double wheel_separation);

    // Function to compute wheel speeds from linear and angular velocities
    void computeWheelSpeeds(double linear_velocity, double angular_velocity, double& front_left_wheel_speed, double& front_right_wheel_speed, double& back_left_wheel_speed, double& back_right_wheel_speed);

private:
    double wheel_radius_;
    double wheel_separation_;
};

#endif // KINEMATICS_H

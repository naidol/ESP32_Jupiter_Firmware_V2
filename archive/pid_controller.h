#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    float calculate(float setpoint, float measured_value, float dt);

private:
    float kp, ki, kd;
    float previous_error;
    float integral;
};

#endif // PID_CONTROLLER_H

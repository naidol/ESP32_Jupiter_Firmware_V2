//#######################################################################################################
// Name:             jupiter_config.h
// Purpose:          Main configuration header file for Jupiter Robot
// Description:      ESP32 pins and Robot properties can be user defined here 
// Related Files:         
// Author:           logan naidoo, south africa, 2024
//########################################################################################################

// Motor Pins
#define MOTOR1_PWM  32
#define MOTOR1_DIR  33
#define MOTOR1_ENC_B  25
#define MOTOR1_ENC_A  26

#define MOTOR2_PWM  23 //19
#define MOTOR2_DIR  19 //23
#define MOTOR2_ENC_A  18
#define MOTOR2_ENC_B  5

#define MOTOR3_PWM  27
#define MOTOR3_DIR  14
#define MOTOR3_ENC_B  12
#define MOTOR3_ENC_A  13

#define MOTOR4_PWM  17 //16
#define MOTOR4_DIR  16 //17
#define MOTOR4_ENC_A  4
#define MOTOR4_ENC_B  15 

// Define Onboard LED
#define ESP32_LED 2

// Pin assignments for motors and encoders
// #define FRONT_LEFT_MOTOR_PIN_A  32      // PWM
// #define FRONT_LEFT_MOTOR_PIN_B  33      // DIR
// #define FRONT_LEFT_ENCODER_PIN_A 25     
// #define FRONT_LEFT_ENCODER_PIN_B 26

// #define FRONT_RIGHT_MOTOR_PIN_A 23      // PWM
// #define FRONT_RIGHT_MOTOR_PIN_B 19      // DIR
// #define FRONT_RIGHT_ENCODER_PIN_A 18
// #define FRONT_RIGHT_ENCODER_PIN_B 5

// #define BACK_LEFT_MOTOR_PIN_A   27      // PWM
// #define BACK_LEFT_MOTOR_PIN_B   14      // DIR
// #define BACK_LEFT_ENCODER_PIN_A  12     
// #define BACK_LEFT_ENCODER_PIN_B  13

// #define BACK_RIGHT_MOTOR_PIN_A  17      // PWM
// #define BACK_RIGHT_MOTOR_PIN_B  16      // DIR
// #define BACK_RIGHT_ENCODER_PIN_A 4
// #define BACK_RIGHT_ENCODER_PIN_B 15

// Wheel and robot parameters
// #define WHEEL_RADIUS 0.045
// #define WHEEL_BASE_WIDTH 0.400

constexpr double WHEEL_RADIUS = 0.045; // meters
constexpr double WHEEL_SEPARATION = 0.4; // meters


// PID and Control Loop parameters
// #define PID_KP  1.0     //1.0
// #define PID_KI  0.2     //0.5
// #define PID_KD  0.1     //0.1
#define CONTROL_LOOP_HZ 50
#define PWM_FREQ 20000 // 5000
#define PWM_BITS 10 // 8

// ENCODER specifications
#define ENCODER_CPR 1320 // ticks or counts per revolution

// FROM LINO FOR PID
#define K_P 2.0 //0.6                             // P constant
#define K_I 0.0 //0.8                             // I constant
#define K_D 0.0 //0.5                             // D constant
#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX
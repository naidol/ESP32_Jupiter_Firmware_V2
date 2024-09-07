//#######################################################################################################
// Name:             main.cpp
// Purpose:          Jupiter Robot ESP32 firmware
// Description:      Robot motor drivers, IMU, LED are controlled using PID and communicates to Host PC
//                   using Micro-ROS.  This firmware reads cmd_vel msgs from ROS2 host and publishes
//                   imu/data and odom/unfiltered msgs back to the host so that ROS2 Navigation can compute
//                   the robots position and orientation and determine velocity feedback to the ESP32
//                   Also included are other modules that drive the attached OLED display and Onboard LED
//                   to indicate when the Robot is listening to voice commands.
// Related Files:    this firmware is built to compile on VS CODE using the PLATFORMIO plugin     
// Author:           logan naidoo, south africa, 2024
//########################################################################################################

#include <Arduino.h>                            // needed if using Platformio and VS Code IDE
#include <micro_ros_platformio.h>               // use if using platformio, otherwise #include <miro_ros_arduino.h>
#include <Wire.h>
#include <esp32-hal-ledc.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <utility/imumaths.h>
#include <nav_msgs/msg/odometry.h>

// Include the local header files
#include "jupiter_config.h"
#include "imu_bno055.h"
#include "encoder.h"
#include "kinematics.h"
#include "odometry.h"
#include "pid.h"

#include "motor.h"


// Declare the cmd_vel subscriber and twist message variable
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

// Declare the IMU publisher
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;  

// Declare the encoder publisher oject and message array
rcl_publisher_t encoder_publisher;
std_msgs__msg__Int32MultiArray encoder_msg;

// Declare the ROS & micro-ROS node interfaces
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

// Setup Motor Instances
Motor motor1(MOTOR1_PWM, MOTOR1_DIR, 0, PWM_FREQ, PWM_BITS); // attach to pwm chan 0
Motor motor2(MOTOR2_PWM, MOTOR2_DIR, 1, PWM_FREQ, PWM_BITS); // attach to pwm chan 1
Motor motor3(MOTOR3_PWM, MOTOR3_DIR, 2, PWM_FREQ, PWM_BITS); // attach to pwm chan 2
Motor motor4(MOTOR4_PWM, MOTOR4_DIR, 3, PWM_FREQ, PWM_BITS); // attach to pwm chan 3


// Setup Wheel Ecoders for each Motor
Encoder motor1_encoder(MOTOR1_ENC_A, MOTOR1_ENC_B, ENCODER_CPR);
Encoder motor2_encoder(MOTOR2_ENC_A, MOTOR2_ENC_B, ENCODER_CPR);
Encoder motor3_encoder(MOTOR3_ENC_A, MOTOR3_ENC_B, ENCODER_CPR);
Encoder motor4_encoder(MOTOR4_ENC_A, MOTOR4_ENC_B, ENCODER_CPR);



// Setup the robot kinematics based on its wheel location and radius
// This object (Kinematics) is used to provide differential drive mechanics to compute left and right wheel speeds
Kinematics kinematics(WHEEL_RADIUS, WHEEL_SEPARATION);

// From LINO ROBOT -----------------------------------------------------------------------------------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(ESP32_LED, HIGH);
        delay(150);
        digitalWrite(ESP32_LED, LOW);
        delay(150);
    }
    delay(1000);
}
void rclErrorLoop() 
{
    while(true)
    {
        flashLED(2);
    }
}
// For Odometry
rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;
unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
Odometry odometry;

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis(); 
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

// Setup the PID controllers for each motor
PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

// End LINO ROBOT STUFF --------------------------------------------------------------------

// PIDController pid_front_left(PID_KP, PID_KI, PID_KD);
// PIDController pid_front_right(PID_KP, PID_KI, PID_KD);
// PIDController pid_back_left(PID_KP, PID_KI, PID_KD);
// PIDController pid_back_right(PID_KP, PID_KI, PID_KD);

float target_linear_velocity = 0;
float target_angular_velocity = 0;

void setMotorSpeed(int fl_speed, int fr_speed, int bl_speed, int br_speed) {
    // Set all motor PWM outputs to zero to stop the motors
    motor1.setSpeed(fl_speed);
    motor2.setSpeed(fr_speed);
    motor3.setSpeed(bl_speed);
    motor4.setSpeed(br_speed);
    if (fl_speed == 0 && fr_speed == 0 && bl_speed == 0 && br_speed == 0)
        digitalWrite(ESP32_LED, LOW); // ESP32 LED turns off when all wheels motors stop
    else
        digitalWrite(ESP32_LED, HIGH);
}

void cmdVelCallback(const void *msg_in) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
    target_linear_velocity = msg->linear.x;
    target_angular_velocity = msg->angular.z;

    // double front_left_speed, front_right_speed, back_left_speed, back_right_speed;
    
    // Compute the wheel speeds based on the cmd_vel input
    Kinematics::WheelSpeeds wheel_speeds = kinematics.computeWheelSpeeds(target_linear_velocity, target_angular_velocity);
   
    // Set motor speeds for all four motors
    setMotorSpeed(wheel_speeds.front_left_speed, wheel_speeds.front_right_speed, wheel_speeds.back_left_speed, wheel_speeds.back_right_speed); 

}

void moveBase()
{
    // brake if there's no command received, or when it's only the first command sent
    // if(((millis() - prev_cmd_time) >= 200)) 
    // {
    //     cmd_vel_msg.linear.x = 0.0;
    //     cmd_vel_msg.linear.y = 0.0;
    //     cmd_vel_msg.angular.z = 0.0;

    //     digitalWrite(ESP32_LED, HIGH);
    // }
    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::MotorRPM req_rpm = kinematics.calculateRPM(
        target_linear_velocity, 
        // cmd_vel_msg.linear.y, 
        target_angular_velocity
    );

    // get the current speed of each motor
    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();
    float current_rpm3 = motor3_encoder.getRPM();
    float current_rpm4 = motor4_encoder.getRPM();

    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1.setSpeed(motor1_pid.compute(req_rpm.front_left_rpm, current_rpm1));
    motor2.setSpeed(motor1_pid.compute(req_rpm.front_right_rpm, current_rpm2));
    motor3.setSpeed(motor1_pid.compute(req_rpm.back_left_rpm, current_rpm3));
    motor4.setSpeed(motor1_pid.compute(req_rpm.back_right_rpm, current_rpm4));

    Kinematics::Velocities current_vel = kinematics.getVelocities(
        current_rpm1, 
        current_rpm2, 
        current_rpm3, 
        current_rpm4
    );

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt, 
        current_vel.linear_velocity_x, 
        current_vel.linear_velocity_y, 
        current_vel.angular_velocity_z
    );
}

void timerCallback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    
    // Get & Publish the IMU message
    get_imu_data(&imu_msg);
    rcl_ret_t ret_imu_ok = rcl_publish(&imu_publisher, &imu_msg, NULL);

    // ---------------Encoder pub test ------------------------------
    // Update encoder message
    encoder_msg.data.data[0] = int32_t (motor1_encoder.getCount());
    encoder_msg.data.data[1] = int32_t (motor2_encoder.getCount());
    encoder_msg.data.data[2] = int32_t (motor3_encoder.getCount());
    encoder_msg.data.data[3] = int32_t (motor4_encoder.getCount());
    // Publish encoder data
    rcl_ret_t ret_enc_ok = rcl_publish(&encoder_publisher, &encoder_msg, NULL);
    // -------------- Encoder pub test end ----------------------------

    // LINO
    // moveBase();
    // END LINO

}



void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);

    // Set initial PWM values to zero to prevent motors from moving at startup
    setMotorSpeed(0, 0, 0, 0);

    // Initialise Encoders and attach interrupts and reset the counters
    motor1_encoder.begin();
    motor2_encoder.begin();
    motor3_encoder.begin();
    motor4_encoder.begin();
    delay(3000);            // delay to allow wheels spin on startup to stop & reset counters
    motor1_encoder.reset();
    motor2_encoder.reset();
    motor3_encoder.reset();
    motor4_encoder.reset();


    // Set Micro-ROS transport
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();

    // Create init_options and support
    rclc_support_init(&support, 0, NULL, &allocator);

    // ---------------------  NEW GPT CODE -------------------------------
    rcl_node_t node = rcl_get_zero_initialized_node();
    // ---------------------- END NEW GPT CODE ---------------------------

    // Create node
    rclc_node_init_default(&node, "esp32_node", "", &support);


    // Create IMU publisher
    rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data");

    // Create cmd_vel subscriber
    rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    // Create odometry publisher
    rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom/unfiltered");

    // ---------------------------Encoder TEST --------------------------
    
    // Create encoder publisher
    rclc_publisher_init_default(
        &encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "wheel_encoders");

    // Initialize encoder message
    encoder_msg.data.size = 4;
    encoder_msg.data.capacity = 4;
    encoder_msg.data.data = (int32_t *)malloc(encoder_msg.data.capacity * sizeof(int32_t));
    // ___________________ end encoder test ______________________________
    
    // Create timer
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100), // The timer callback will execute every 100 ms
        timerCallback);
    
    // Create executor which only handles timer and subscriber callbacks
    rclc_executor_init(&executor, &support.context, 2, &allocator);  // adjust the number of handles for each callback added
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA);

    // Initialise (setup) IMU and OLED
    setup_imu(&imu_msg);   // built for Bosch BNO055 imu. Replace imu_bno055.h and .cpp if needed
    setup_oled_display();  // comment this out, if OLED SSD1306 on controller board not connected

    // Set up LED pin
    pinMode(ESP32_LED, OUTPUT);
    digitalWrite(ESP32_LED, LOW);

    // synchronize time with the agent
    syncTime();
    flashLED(5);

}

void loop() {
    float dt = 1.0 / CONTROL_LOOP_HZ;

    // Spin the ROS 2 executor to handle callbacks
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); // was 100

    // Stop motors if both target velocities are zero
    // if (target_linear_velocity == 0 && target_angular_velocity == 0) {
    //     setMotorSpeed(0, 0, 0, 0);
    // } 

 
    delay(1000 / CONTROL_LOOP_HZ);

    // delay(50);
}
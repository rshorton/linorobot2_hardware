// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float32.h>

#include "config.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#if defined(NO_ENCODER)
#include "encoder_none.h"
#else
#include "encoder.h"
#endif
#include "diagnostics.h"
#include "util.h"

#define PUBLISH_MOTOR_DIAGS         // Define to publish the PID status for plotting via RQT
#define TUNE_PID_LOOP               // Allow tweaking of PID parameters via topic write

#define ERR_BLINK_GENERAL   2
#define ERR_BLINK_IMU       3

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop(ERR_BLINK_GENERAL);}}
#define RCCHECK_WITH_BLINK_CODE(blink_code, fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop(blink_code);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_subscription_t twist_subscriber;

#if defined(TUNE_PID_LOOP)
rcl_subscription_t pid_kp_subscriber;
rcl_subscription_t pid_kd_subscriber;
rcl_subscription_t pid_ki_subscriber;

std_msgs__msg__Float32 pid_kp_msg;
std_msgs__msg__Float32 pid_kd_msg;
std_msgs__msg__Float32 pid_ki_msg;
#endif

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
bool micro_ros_init_successful = false;

Kinematics::rpm req_rpm;
Kinematics::rpm last_rpm = {0.0, 0.0, 0.0, 0.0};

int m1_dir_status = 0;
int m2_dir_status = 0;
int m3_dir_status = 0;
int m4_dir_status = 0;

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV, &m1_dir_status);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV, &m2_dir_status);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV, &m3_dir_status);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV, &m4_dir_status);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B, MOTOR1_CURRENT, &m1_dir_status);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B, MOTOR2_CURRENT, &m2_dir_status);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B, MOTOR3_CURRENT, &m3_dir_status);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B, MOTOR4_CURRENT, &m4_dir_status);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(
    Kinematics::LINO_BASE, 
    MOTOR_MAX_RPM, 
    MAX_RPM_RATIO, 
    MOTOR_OPERATING_VOLTAGE, 
    MOTOR_POWER_MAX_VOLTAGE, 
    WHEEL_DIAMETER, 
    LR_WHEELS_DISTANCE
);

Odometry odometry;
IMU imu;

float current_rpm1 = 0.0;
float current_rpm2 = 0.0;
float current_rpm3 = 0.0;
float current_rpm4 = 0.0;

MotorDiags motor1_diags;
MotorDiags motor2_diags;
MotorDiags motor3_diags;
MotorDiags motor4_diags;

bool pwr_relay_on = false;

void setup() 
{
    pinMode(LED_PIN, OUTPUT);

    pinMode(MOTOR_RELAY_PWR_OUT, OUTPUT);
    pinMode(MOTOR_RELAY_PWR_IN, INPUT);


    bool imu_ok = imu.init();
    if(!imu_ok)
    {
        while(1)
        {
            flashLED(3);
        }
    }

    micro_ros_init_successful = false;
    
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
}

void loop() 
{
    static unsigned long prev_connect_test_time;
    // check if the agent got disconnected at 10Hz
    if(millis() - prev_connect_test_time >= 100)
    {
        prev_connect_test_time = millis();
        // check if the agent is connected
        if(RMW_RET_OK == rmw_uros_ping_agent(10, 2))
        {
            // reconnect if agent got disconnected or haven't at all
            if (!micro_ros_init_successful) 
            {
                createEntities();
            } 
        } 
        else if(micro_ros_init_successful)
        {
            // stop the robot when the agent got disconnected
            fullStop();
            // clean up micro-ROS components
            destroyEntities();
        }
    }
    
    if(micro_ros_init_successful)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
       moveBase();
       publishData();
    }
}

void twistCallback(const void * msgin) 
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    prev_cmd_time = millis();
}

#if defined(TUNE_PID_LOOP)
void pidKpCallback(const void * msgin) 
{
    motor1_pid.updateKp(pid_kp_msg.data);
    motor2_pid.updateKp(pid_kp_msg.data);
    motor3_pid.updateKp(pid_kp_msg.data);
    motor4_pid.updateKp(pid_kp_msg.data);
}

void pidKdCallback(const void * msgin) 
{
    motor1_pid.updateKd(pid_kd_msg.data);
    motor2_pid.updateKd(pid_kd_msg.data);
    motor3_pid.updateKd(pid_kd_msg.data);
    motor4_pid.updateKd(pid_kd_msg.data);
}

void pidKiCallback(const void * msgin) 
{
    motor1_pid.updateKi(pid_ki_msg.data);
    motor2_pid.updateKi(pid_ki_msg.data);
    motor3_pid.updateKi(pid_ki_msg.data);
    motor4_pid.updateKi(pid_ki_msg.data);
}
#endif

void createEntities()
{
    allocator = rcl_get_default_allocator();
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "linorobot_base_node", "", &support));
    // create odometry publisher
    RCCHECK(rclc_publisher_init_default( 
        &odom_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"
    ));
    // create IMU publisher
    RCCHECK(rclc_publisher_init_default( 
        &imu_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    ));

#if defined(PUBLISH_MOTOR_DIAGS)
    // create diagnostics publisher
    motor1_diags.create(node, 1);
    motor2_diags.create(node, 2);
    motor3_diags.create(node, 3);
    motor4_diags.create(node, 4);
#endif

#if defined(TUNE_PID_LOOP)
    RCCHECK_WITH_BLINK_CODE(3, rclc_subscription_init_default( 
        &pid_kp_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "pid_kp"
    ));
    RCCHECK(rclc_subscription_init_default( 
        &pid_kd_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "pid_kd"
    ));

    RCCHECK(rclc_subscription_init_default( 
        &pid_ki_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "pid_ki"
    ));
#endif

    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default( 
        &twist_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));

    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default( 
        &control_timer, 
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback
    ));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2 + 3, & allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &twist_subscriber, 
        &twist_msg, 
        &twistCallback, 
        ON_NEW_DATA
    ));
#if defined(TUNE_PID_LOOP)
    RCCHECK_WITH_BLINK_CODE(4, rclc_executor_add_subscription(
        &executor, 
        &pid_kp_subscriber, 
        &pid_kp_msg, 
        &pidKpCallback, 
        ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &pid_kd_subscriber, 
        &pid_kd_msg, 
        &pidKdCallback, 
        ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &pid_ki_subscriber, 
        &pid_ki_msg, 
        &pidKiCallback, 
        ON_NEW_DATA
    ));
#endif
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_PIN, HIGH);
    micro_ros_init_successful = true;
}

void destroyEntities()
{
    digitalWrite(LED_PIN, LOW);

#if defined(PUBLISH_MOTOR_DIAGS)
    motor1_diags.destroy(node);
    motor2_diags.destroy(node);
    motor3_diags.destroy(node);
    motor4_diags.destroy(node);
#endif

    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
#if defined(TUNE_PID_LOOP)
    rcl_subscription_fini(&pid_kp_subscriber, &node);
    rcl_subscription_fini(&pid_kd_subscriber, &node);
    rcl_subscription_fini(&pid_ki_subscriber, &node);
#endif
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    micro_ros_init_successful = false;
}

void fullStop()
{
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    motor1_controller.brake();
    motor2_controller.brake();
    motor3_controller.brake();
    motor4_controller.brake();
}

bool directionChange(float cur_rpm, float req_rpm)
{
    return abs(cur_rpm) > 0.1 && sgn(cur_rpm) != sgn(req_rpm);
}

int encoder_read_cnt = 0;

void moveBase()
{
    // brake if there's no command received, or when it's only the first command sent
    if(((millis() - prev_cmd_time) >= 200)) 
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        digitalWrite(LED_PIN, HIGH);
    }
    // get the required rpm for each motor based on required velocities, and base used
    req_rpm = kinematics.getRPM(
        twist_msg.linear.x, 
        twist_msg.linear.y, 
        twist_msg.angular.z
    );

    // get the current speed of each motor
    current_rpm1 = motor1_encoder.getRPM();
    current_rpm2 = motor2_encoder.getRPM();
    current_rpm3 = motor3_encoder.getRPM();
    current_rpm4 = motor4_encoder.getRPM();

    // Don't drive motor in the opposite direction until it stops
    if (directionChange(current_rpm1, req_rpm.motor1)) {
        req_rpm.motor1 = 0.0;
    }

    if (directionChange(current_rpm2, req_rpm.motor2)) {
        req_rpm.motor2 = 0.0;
    }

    if (directionChange(current_rpm3, req_rpm.motor3)) {
        req_rpm.motor3 = 0.0;
    }

    if (directionChange(current_rpm4, req_rpm.motor4)) {
        req_rpm.motor4 = 0.0;
    }

    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
    motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));

    // If power relay not on, then wait until we actually want to move
    // before turning on motor power
    if (!pwr_relay_on &&
        (motor1_pid.getOutputConstrained() != 0 ||
         motor2_pid.getOutputConstrained() != 0 ||
         motor3_pid.getOutputConstrained() != 0 ||
         motor4_pid.getOutputConstrained() != 0)) {
        digitalWrite(MOTOR_RELAY_PWR_OUT, HIGH);
        pwr_relay_on = true;
    }

    Kinematics::velocities current_vel = kinematics.getVelocities(
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
        current_vel.linear_x, 
        current_vel.linear_y, 
        current_vel.angular_z
    );
}

void publishData()
{
    odom_msg = odometry.getData();
    imu_msg = imu.getData();

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

#if defined(PUBLISH_MOTOR_DIAGS)
    motor1_diags.publish(time_stamp, req_rpm.motor1, current_rpm1, motor1_controller.getCurrent(), motor1_pid);
    motor2_diags.publish(time_stamp, req_rpm.motor2, current_rpm2, motor2_controller.getCurrent(), motor2_pid);
    motor3_diags.publish(time_stamp, req_rpm.motor3, current_rpm3, motor3_controller.getCurrent(), motor3_pid);
    motor4_diags.publish(time_stamp, req_rpm.motor4, current_rpm4, motor4_controller.getCurrent(), motor4_pid);
#endif
}

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

void rclErrorLoop(int n_times) 
{
    while(true)
    {
        flashLED(n_times);
    }
}

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}

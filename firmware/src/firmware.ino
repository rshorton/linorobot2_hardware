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
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/joy.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

#include "config.h"
#include "logger.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#define ENCODER_USE_INTERRUPTS
#include "encoder.h"
#include "encoder_single_phase.h"
#include "motor_diagnostics.h"
#include "servo_diagnostics.h"
#include "util.h"

#include "motor_speed_controller.h"
#include "linear_actuator.h"
#include "steering_using_linear_actuator.h"
#include "steering_angle_to_actuator_mapper_ebot_ackerman.h"

#include "hc_sr04.h"
#include "serial_bus_servo.h"
#include "ros_range_sensor.h"
#include "ros_rot_range_sensor.h"

#define TUNE_PID_LOOP               // Allow tweaking of PID parameters via topic write

#undef FAIL_ON_UROS_LINK_LOST

// Game controller buttons
const int JOY_BUTTON_LB = 4; // left side, closest to top
const int JOY_BUTTON_X = 2;  // X
const int JOY_BUTTON_Y = 3;  // Y
const int JOY_BUTTON_A = 0;  // A
const int JOY_BUTTON_B = 1;  // B

const int JOY_AXIS_LEFT_STICK_LR = 0;
const int JOY_AXIS_LEFT_STICK_UD = 1;
const int JOY_AXIS_RIGHT_STICK_LR = 2;
const int JOY_AXIS_RIGHT_STICK_UD = 3;

const int JOY_AXIS_RIGHT_TRIGGER_BUTTON = 4;
const int JOY_AXIS_LEFT_TRIGGER_BUTTON = 5;

const int JOY_AXIS_DPAD_LR = 6;
const int JOY_AXIS_DPAD_UD = 7;

#define ERR_BLINK_GENERAL   2
#define ERR_BLINK_IMU       3
#define ERR_BLINK_STEERING  4
#define ERR_BLINK_UROS_LOST 5

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop(ERR_BLINK_GENERAL);}}
#define RCCHECK_WITH_BLINK_CODE(blink_code, fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop(blink_code);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t imu_mag_field_publisher;
rcl_subscription_t twist_subscriber;
rcl_subscription_t joy_subscriber;

#if defined(TUNE_PID_LOOP)
rcl_subscription_t pid_kp_subscriber;
rcl_subscription_t pid_kd_subscriber;
rcl_subscription_t pid_ki_subscriber;
rcl_subscription_t pid_type_subscriber;
rcl_subscription_t range_scan_enable_subscriber;

std_msgs__msg__Float32 pid_kp_msg;
std_msgs__msg__Float32 pid_kd_msg;
std_msgs__msg__Float32 pid_ki_msg;
std_msgs__msg__Int32 pid_type_msg;
#endif

std_msgs__msg__Bool range_scan_enable_msg;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_field_msg;
geometry_msgs__msg__Twist twist_msg;

sensor_msgs__msg__Joy joy_msg;
int32_t button_data[9];
float axes_data[8];

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_timer_t sensor_timer;
rcl_timer_t dist_sensor_timer;
rcl_timer_t sync_time_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long prev_joy_cmd_time = 0;
bool new_twist_msg = false;
bool micro_ros_init_successful = false;

enum class tune_pid_type_t { WheelMotors, SteeringAct, SteeringMotor };
tune_pid_type_t tune_pid_type = tune_pid_type_t::WheelMotors;

const float MIN_MOVING_RPM_THRESH = 1.0f;

const float SPEED_SCALE_TURTLE = 0.15;
const float SPEED_SCALE_SLOW = 0.30;
const float SPEED_SCALE_NORMAL = 0.65;

bool ackermann_teleop = true;
float speed_scale = SPEED_SCALE_SLOW;
float speed_x_in = 0.0;
float steering_angle_in = 0.0;

Kinematics::rpm req_rpm;
Kinematics::rpm last_rpm = {0.0f, 0.0f, 0.0f, 0.0f};

// Range sensors

const int DIST_SENSOR_UPDATE_PERIOD_MS = 20;
HCSR04 dist_sensor_front(0, HCSR04_TRIG_FRONT_OUT, HCSR04_ECHO_FRONT_IN, 6);
HCSR04 dist_sensor_back(1, HCSR04_TRIG_BACK_OUT, HCSR04_ECHO_BACK_IN, 6);

RosRangeSensor range_sensor_front(dist_sensor_front, "hcsr04_front", "ebot/range/front");
RosRangeSensor range_sensor_back(dist_sensor_back, "hcsr04_back", "ebot/range/back");

SerialServo range_servo_front(Serial8, 1, 240, 1000, true);
SerialServo range_servo_back(Serial8, 2, 240, 1000, true);

const float front_ranging_angles[] = {45.0f, 22.5f, 0.0f, -22.5f, -45.0f, -22.5f, 0.0f, 22.5f};
//const float front_ranging_angles[] = {40.0f, 30.0f, 20.0f, 10.0f, 0.0f, -10.0f, -20.0f, -30.0f, -40.0f, -30.0f, -20.0f, -10.0f, 0.0f, 10.0f, 20.0f, 30.0f};
//const float front_ranging_angles[] = {45.0f, 0.0f, -45.0f, 0.0f};
//const float front_ranging_angles[] = {0.0f};
const float back_ranging_angles[] = {45.0f, 0.0f, -45.0f, 0.0f};
//const float back_ranging_angles[] = {60.0f, 30.0f, 0.0f, -30.0f, -60.0f, -30.0f, 0.0f, 30.0f};
RosRotatingRangeSensor front_rotating_range_sensor("hcsr04_pan_joint_front", range_sensor_front, range_servo_front, front_ranging_angles,
                                                   sizeof(front_ranging_angles)/sizeof(float), 120.0f, 10);
RosRotatingRangeSensor back_rotating_range_sensor("hcsr04_pan_joint_back", range_sensor_back, range_servo_back, back_ranging_angles,
                                                   sizeof(back_ranging_angles)/sizeof(float), 120.0f, 300);

//////////////////////////////////
// Wheel related
//////////////////////////////////

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B, -1);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B, -1);

#if NUM_BASE_MOTORS == 4
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B, -1);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B, -1);
#endif

// Encoders

EncoderSinglePhase motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV, motor1_controller);
EncoderSinglePhase motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV, motor2_controller);

#if NUM_BASE_MOTORS == 4
EncoderSinglePhase motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV, motor3_controller);
EncoderSinglePhase motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV, motor4_controller);
#endif

// Speed controllers

const float MOTOR_DIR_CHANGE_HOLD_OFF_RPM = 30.0f;
PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
MotorSpeedController motor1_speed_controller(motor1_controller, motor1_encoder, motor1_pid, MOTOR_DIR_CHANGE_HOLD_OFF_RPM);

PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
MotorSpeedController motor2_speed_controller(motor2_controller, motor2_encoder, motor2_pid, MOTOR_DIR_CHANGE_HOLD_OFF_RPM);

#if NUM_BASE_MOTORS == 4
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
MotorSpeedController motor3_speed_controller(motor3_controller, motor3_encoder, motor3_pid, MOTOR_DIR_CHANGE_HOLD_OFF_RPM);

PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
MotorSpeedController motor4_speed_controller(motor4_controller, motor4_encoder, motor4_pid, MOTOR_DIR_CHANGE_HOLD_OFF_RPM);
#endif

//////////////////////////////////
// Steering  - see comments in lino_base_config file.
//////////////////////////////////

// Motor
Motor motor_str_controller(PWM_FREQUENCY, PWM_BITS, MOTOR_STR_INV, MOTOR_STR_PWM, MOTOR_STR_IN_A, MOTOR_STR_IN_B, -1);

// Motor/shaft encoder
EncoderQuadrature str_motor_enc(STEERMTR_ENCODER_A, STEERMTR_ENCODER_B, STR_MOTOR_ENC_TICKS_PER_REV, MOTOR_STR_ENCODER_INV);
EncoderNull str_wheel_enc;

// Motor speed controller
PID str_motor_spd_pid(STR_SPD_PWM_MIN, STR_SPD_PWM_MAX, STR_SPD_PID_P, STR_SPD_PID_I, STR_SPD_PID_D);
MotorSpeedController str_motor_speed_controller(motor_str_controller, str_motor_enc, str_motor_spd_pid);


PID str_act_pid(STR_ACT_RPM_MIN, STR_ACT_RPM_MAX, STR_ACT_PID_P, STR_ACT_PID_I, STR_ACT_PID_D);
LinearActuator steering_actuator(LinearActuator::HomeDetection::kSwitch, STR_LEFT_LIMIT_IN,
                                 str_motor_speed_controller, str_motor_enc, str_act_pid, STR_ACT_HOMING_RPM,
                                  STR_ACT_MAX_POS, STR_ACT_POS_THRESH);

SteeringAngleToActuatorMapperEbotAckerman steering_angle_to_lin_actuator_mapper;
SteeringUsingLinearActuator steering(steering_actuator, steering_angle_to_lin_actuator_mapper);

Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    FR_WHEELS_DISTANCE,
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
#if NUM_BASE_MOTORS == 4
MotorDiags motor3_diags;
MotorDiags motor4_diags;
#endif

MotorDiags steering_motor_diags;
ServoDiags steering_servo_diags;

int connection_drop_cnt = 0;

bool estopAsserted()
{
    return digitalRead(ESTOP_IN) == 0;
}

bool ackermannSteeringEnabled()
{
    return digitalRead(ENABLE_ACKERMANN);
}

void configureSteeringMode()
{
    auto use_ackermann = ackermannSteeringEnabled();

    enum Kinematics::base platform = Kinematics::ACKERMANN;
    if (!use_ackermann)
    {
        platform = Kinematics::DIFFERENTIAL_DRIVE;
    }
    if (kinematics.getBasePlatform() != platform) {
        kinematics.setBasePlatform(platform);
        Logger::log_message(Logger::LogLevel::Info, "Using ackermann %d", use_ackermann);        
    }
}

extern "C" void setup()
{
    pinMode(LED_PIN, OUTPUT);

    pinMode(MOTOR_RELAY_PWR_OUT, OUTPUT);
    pinMode(MOTOR_RELAY_PWR_IN, INPUT);

    pinMode(ENABLE_ACKERMANN, INPUT_PULLUP);

    digitalWrite(MOTOR_RELAY_PWR_OUT, LOW);

    bool imu_ok = imu.init();
    if (!imu_ok)
    {
        while (1)
        {
            flashLED(3);
        }
    }

    micro_ros_init_successful = false;

    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    configureSteeringMode();

    flashLED(2);
}

extern "C" void loop()
{
    static unsigned long prev_connect_test_time = 0;
    // check if the agent got disconnected at 10Hz
    if (millis() - prev_connect_test_time >= 100)
    {
        prev_connect_test_time = millis();
        // check if the agent is connected
        if (RMW_RET_OK == rmw_uros_ping_agent(10, 50))
        {
            // reconnect if agent got disconnected or first time
            if (!micro_ros_init_successful)
            {
                createEntities();
                Logger::log_message(Logger::LogLevel::Info, "Micro ROS initialized, connection drop cnt: %d",
                    connection_drop_cnt);
                
                front_rotating_range_sensor.init(node);
                front_rotating_range_sensor.start(false);

                back_rotating_range_sensor.init(node);
                back_rotating_range_sensor.start(false);

                // Enable the power relay.  Still requires the wireless switch to be
                // enabled and the E-switch to be On before power is applied to motor drive. 
                digitalWrite(MOTOR_RELAY_PWR_OUT, HIGH);
            }
        }
        else if (micro_ros_init_successful)
        {
            connection_drop_cnt++;

            // Disable power relay
            digitalWrite(MOTOR_RELAY_PWR_OUT, LOW);

            // stop the robot when the agent is disconnected
            fullStop();
            // clean up micro-ROS components
            destroyEntities();
#if defined(FAIL_ON_UROS_LINK_LOST)
            rclErrorLoop(ERR_BLINK_UROS_LOST);
#endif            
        }
    }

    if (micro_ros_init_successful)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
}

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        if (kinematics.getBasePlatform() == Kinematics::ACKERMANN) {
            if (steering.get_state() == SteeringUsingLinearActuator::State::kInit)
            {
                digitalWrite(MOTOR_RELAY_PWR_OUT, HIGH);
                fullStop();
                steering.home();
                return;
            }
            else if (steering.get_state() == SteeringUsingLinearActuator::State::kHoming)
            {
                if (!is_moving() && !estopAsserted())
                {
                    steering.update();
                }
                return;
            }
            else if (steering.get_state() == SteeringUsingLinearActuator::State::kHomingFailure)
            {
                rclErrorLoop(ERR_BLINK_STEERING);
                return;
            }
        }

        if (estopAsserted())
        {
            fullStop();
        }
        else
        {
            moveBase();
        }
        publishData();
    }
}

void syncTimeCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
        syncTime();
    }
}

void sensorCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
       publishSensorData();
    }
}

void distSensorCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        front_rotating_range_sensor.update();
        back_rotating_range_sensor.update();
    }
}

void twistCallback(const void *msgin)
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    prev_cmd_time = millis();
    new_twist_msg = true;
}

#if defined(TUNE_PID_LOOP)
void pidKpCallback(const void * msgin) 
{
    Logger::log_message(Logger::LogLevel::Info, "Tune Pid set Kp: %f, type: %d", 
                        pid_kp_msg.data, tune_pid_type);

    if (tune_pid_type == tune_pid_type_t::WheelMotors)
    {
        motor1_pid.updateKp(pid_kp_msg.data);
        motor2_pid.updateKp(pid_kp_msg.data);
#if NUM_BASE_MOTORS == 4
        motor3_pid.updateKp(pid_kp_msg.data);
        motor4_pid.updateKp(pid_kp_msg.data);
#endif
    }
    else if (tune_pid_type == tune_pid_type_t::SteeringAct)
    {
        str_act_pid.updateKp(pid_kp_msg.data);
    }
    else if (tune_pid_type == tune_pid_type_t::SteeringMotor)
    {
        str_motor_spd_pid.updateKp(pid_kp_msg.data);
    }
}

void pidKdCallback(const void * msgin)
{
    Logger::log_message(Logger::LogLevel::Info, "Tune Pid set Kd: %f, type: %d",
                        pid_kd_msg.data,  tune_pid_type);

    if (tune_pid_type == tune_pid_type_t::WheelMotors)
    {
        motor1_pid.updateKd(pid_kd_msg.data);
        motor2_pid.updateKd(pid_kd_msg.data);
#if NUM_BASE_MOTORS == 4
        motor3_pid.updateKd(pid_kd_msg.data);
        motor4_pid.updateKd(pid_kd_msg.data);
#endif
    }
    else if (tune_pid_type == tune_pid_type_t::SteeringAct)
    {
        str_act_pid.updateKd(pid_kd_msg.data);
    }
    else if (tune_pid_type == tune_pid_type_t::SteeringMotor)
    {
        str_motor_spd_pid.updateKd(pid_kd_msg.data);
    }
}

void pidKiCallback(const void * msgin)
{
    Logger::log_message(Logger::LogLevel::Info, "Tune Pid set Ki: %f, type: %d", 
                        pid_ki_msg.data, tune_pid_type);

    if (tune_pid_type == tune_pid_type_t::WheelMotors)
    {
        motor1_pid.updateKi(pid_ki_msg.data);
        motor2_pid.updateKi(pid_ki_msg.data);
#if NUM_BASE_MOTORS == 4
        motor3_pid.updateKi(pid_ki_msg.data);
        motor4_pid.updateKi(pid_ki_msg.data);
#endif
    }
    else if (tune_pid_type == tune_pid_type_t::SteeringAct)
    {
        str_act_pid.updateKi(pid_ki_msg.data);
    }
    else if (tune_pid_type == tune_pid_type_t::SteeringMotor)
    {
        str_motor_spd_pid.updateKi(pid_ki_msg.data);
    }
}

void pidTypeCallback(const void * msgin)
{
    tune_pid_type = static_cast<tune_pid_type_t>(pid_type_msg.data);
    Logger::log_message(Logger::LogLevel::Info, "Tune Pid set type: %d", tune_pid_type);
}
#endif

void rangeScanEnableCallback(const void * msgin)
{
    auto scan = range_scan_enable_msg.data;
    front_rotating_range_sensor.start(scan);
    back_rotating_range_sensor.start(scan);
}

void setSpeedScale(float scale)
{
    if (scale >= SPEED_SCALE_TURTLE &&
        scale <= SPEED_SCALE_NORMAL)
    {
        speed_scale = scale;
    }
}

void joyCallback(const void *msgin)
{
    RCLC_UNUSED(msgin);

    if (joy_msg.buttons.data[JOY_BUTTON_X])
    {
        setSpeedScale(SPEED_SCALE_SLOW);
    }
    else if (joy_msg.buttons.data[JOY_BUTTON_A])
    {
        setSpeedScale(SPEED_SCALE_NORMAL);
    }

    if (kinematics.getBasePlatform() == Kinematics::ACKERMANN) {
        ackermann_teleop = joy_msg.axes.data[JOY_AXIS_LEFT_TRIGGER_BUTTON] == -1;
        if (ackermann_teleop)
        {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            prev_joy_cmd_time = millis();

            speed_x_in = joy_msg.axes.data[JOY_AXIS_LEFT_STICK_UD] * speed_scale;

            // fix - use steering mapper to determine.
            const float STEERING_FULL_RANGE_DEG = 60.0f;
            steering_angle_in = static_cast<float>(joy_msg.axes.data[JOY_AXIS_RIGHT_STICK_LR]) * STEERING_FULL_RANGE_DEG / 2.0f / 180.0f * M_PI;
            return;
        }
    }
    speed_x_in = 0.0;
    steering_angle_in = 0.0;
}


void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
    Logger::log_message(Logger::LogLevel::Info, "Local time diff: %ld", time_offset);
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

class LogTimeProvider: public Logger::TimeProvider
{
    struct timespec get_time() {
        return getTime();
    }
} log_time_provider;

void createEntities()
{
    allocator = rcl_get_default_allocator();
    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "linorobot_base_node", "", &support));
    // create odometry publisher
    RCCHECK(rclc_publisher_init_default( 
        &odom_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "ebot/odom"));

    // create IMU publisher
    RCCHECK(rclc_publisher_init_default( 
        &imu_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "ebot/imu/data"));

    // create IMU Magnetic Field publisher
    RCCHECK(rclc_publisher_init_default( 
        &imu_mag_field_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        "ebot/imu/mag"));

    Logger::create_logger(node, log_time_provider);          

#if defined(PUBLISH_MOTOR_DIAGS)
    // create diagnostics publisher
    motor1_diags.create(node, 1);
    motor2_diags.create(node, 2);
#if NUM_BASE_MOTORS == 4
    motor3_diags.create(node, 3);
    motor4_diags.create(node, 4);
#endif
    steering_motor_diags.create(node, 5);
#endif

#if defined(PUBLISH_SERVO_DIAGS)
    steering_servo_diags.create(node, "steering");
#endif

#if defined(TUNE_PID_LOOP)
    RCCHECK_WITH_BLINK_CODE(3, rclc_subscription_init_default(
        &pid_kp_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "ebot/tune_pid_kp"));
    RCCHECK(rclc_subscription_init_default(
        &pid_kd_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "ebot/tune_pid_kd"));

    RCCHECK(rclc_subscription_init_default(
        &pid_ki_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "ebot/tune_pid_ki"));

    RCCHECK(rclc_subscription_init_default(
        &pid_type_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "ebot/tune_pid_type"));
#endif

    RCCHECK(rclc_subscription_init_default(
        &range_scan_enable_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "ebot/range_scan_enable"));

    RCCHECK(rclc_subscription_init_default(
        &joy_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
        "joy"));

    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel/muxed"));

    // create timer for actuating the motors at 50 Hz
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback));

    // create timer for reading and publishing sensor data 20 Hz
    const unsigned int sensor_timeout = 50;
    RCCHECK(rclc_timer_init_default(
        &sensor_timer,
        &support,
        RCL_MS_TO_NS(sensor_timeout),
        sensorCallback));

    // create timer for updating distance measurements
    const unsigned int dist_sensor_timeout = DIST_SENSOR_UPDATE_PERIOD_MS;
    RCCHECK(rclc_timer_init_default(
        &dist_sensor_timer,
        &support,
        RCL_MS_TO_NS(dist_sensor_timeout),
        distSensorCallback));

    // create timer for periodically syncing the local time with the main CPU
    const unsigned int sync_time_timeout = 5000;
    RCCHECK(rclc_timer_init_default(
        &sync_time_timer,
        &support,
        RCL_MS_TO_NS(sync_time_timeout),
        syncTimeCallback));

    executor = rclc_executor_get_zero_initialized_executor();

    // WATCHOUT - Update the number of handles if more subscriptions/timers added.
    // Also make sure the micro_ros.meta specifies enough allocations for subs and pubs.
    // If this is too small, you should see the ERR_BLINK_GENERAL blink pattern.
    const int num_handles = 8 /* subscriptions */ + 4 /* timers */;
    RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, & allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twistCallback,
        ON_NEW_DATA));

#if defined(TUNE_PID_LOOP)
    RCCHECK_WITH_BLINK_CODE(4, rclc_executor_add_subscription(
        &executor,
        &pid_kp_subscriber,
        &pid_kp_msg,
        &pidKpCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &pid_kd_subscriber,
        &pid_kd_msg,
        &pidKdCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &pid_ki_subscriber,
        &pid_ki_msg,
        &pidKiCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &pid_type_subscriber,
        &pid_type_msg,
        &pidTypeCallback,
        ON_NEW_DATA));
#endif

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &range_scan_enable_subscriber,
        &range_scan_enable_msg,
        &rangeScanEnableCallback,
        ON_NEW_DATA));

    joy_msg.buttons.data = button_data;
    joy_msg.buttons.size = 0;
    joy_msg.buttons.capacity = sizeof(button_data);
    joy_msg.axes.data = axes_data;
    joy_msg.axes.size = 0;
    joy_msg.axes.capacity = sizeof(axes_data);

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &joy_subscriber,
        &joy_msg,
        &joyCallback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &dist_sensor_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &sync_time_timer));

    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_PIN, HIGH);
    micro_ros_init_successful = true;
}

void destroyEntities()
{
    micro_ros_init_successful = false;

    digitalWrite(LED_PIN, LOW);

    Logger::destroy_logger(node);

#if defined(PUBLISH_MOTOR_DIAGS)
    motor1_diags.destroy(node);
    motor2_diags.destroy(node);
#if NUM_BASE_MOTORS == 4
    motor3_diags.destroy(node);
    motor4_diags.destroy(node);
#endif
    steering_motor_diags.destroy(node);
#endif

#if defined(PUBLISH_SERVO_DIAGS)
    steering_servo_diags.destroy(node);
#endif

    front_rotating_range_sensor.destroy(node);
    back_rotating_range_sensor.destroy(node);

    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_publisher_fini(&imu_mag_field_publisher, &node);
    rcl_subscription_fini(&twist_subscriber, &node);

#if defined(TUNE_PID_LOOP)
    rcl_subscription_fini(&pid_kp_subscriber, &node);
    rcl_subscription_fini(&pid_kd_subscriber, &node);
    rcl_subscription_fini(&pid_ki_subscriber, &node);
    rcl_subscription_fini(&pid_type_subscriber, &node);
#endif

    rcl_subscription_fini(&range_scan_enable_subscriber, &node);

    rcl_subscription_fini(&joy_subscriber, &node);
    
    rcl_node_fini(&node);

    rcl_timer_fini(&control_timer);
    rcl_timer_fini(&sensor_timer);
    rcl_timer_fini(&dist_sensor_timer);
    rcl_timer_fini(&sync_time_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
}

void fullStop()
{
    twist_msg.linear.x = 0.0f;
    twist_msg.linear.y = 0.0f;
    twist_msg.angular.z = 0.0f;

    motor1_speed_controller.stop();
    motor2_speed_controller.stop();
#if NUM_BASE_MOTORS == 4
    motor3_speed_controller.stop();
    motor4_speed_controller.stop();
#endif    
    odometry.update(0.0f, 0.0f, 0.0f, 0.0f);
}

bool is_moving()
{
    return abs(motor1_encoder.getRPM()) > MIN_MOVING_RPM_THRESH ||
           abs(motor2_encoder.getRPM()) > MIN_MOVING_RPM_THRESH;
}

// For converting twist msg to Ackermann x vel and steering angle (for bicycle model, where
// there is one wheel in the center of the front axle).
//
// See Car-Like (Bicycle) Model, Double-Traction Axle, and Ackermann Steering sections here:
// https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html

float rot_and_linear_vel_to_steering_angle(float x_vel, float w_vel, float wheelbase)
{
    if (x_vel == 0.0f || w_vel == 0.0f)
    {
        return 0.0f;
    }
    float radius = x_vel / w_vel;
    return atan(wheelbase/radius);
}

void moveBase()
{
    float speed_x = 0.0f;
    float speed_y = 0.0f;
    float speed_z = 0.0f;
    float steering_angle = 0.0f;

    if (kinematics.getBasePlatform() == Kinematics::ACKERMANN && 
        ackermann_teleop)
    {
        if (((millis() - prev_joy_cmd_time) > 200))
        {
            digitalWrite(LED_PIN, HIGH);
        }
        else
        {
            speed_x = speed_x_in;
            steering_angle = steering_angle_in;
        }
    }
    else
    {
        // Handle twist msg input (driven by the twist telop or a
        // navigation controller)

        // brake if there's no command received, or when it's only the first command sent
        if(((millis() - prev_cmd_time) >= 200)) 
        {
            twist_msg.linear.x = 0.0f;
            twist_msg.linear.y = 0.0f;
            twist_msg.angular.z = 0.0f;

            digitalWrite(LED_PIN, HIGH);
        }

        speed_x = twist_msg.linear.x;
        speed_y = twist_msg.linear.y;
        speed_z = twist_msg.angular.z;

        if (kinematics.getBasePlatform() == Kinematics::ACKERMANN)
        {
            // Calculate steering angle (bicycle car model) from x velocity, twist and wheelbase
            // http://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20car-like%20robots
            // (Positive angle when moving forward turns left)
            steering_angle = rot_and_linear_vel_to_steering_angle(twist_msg.linear.x, twist_msg.angular.z, FR_WHEELS_DISTANCE);
            
            // Limit to steerable range
            steering_angle = steering_angle_to_lin_actuator_mapper.actuator_setting_to_angle_fast(
                            steering_angle_to_lin_actuator_mapper.angle_to_actuator_setting_fast(steering_angle));

            if (new_twist_msg)
            {
                new_twist_msg = false;
                Logger::log_message(Logger::LogLevel::Debug, "Steering angle %f, xve: %f, zvel: %f",
                    steering_angle*180.0/M_PI, twist_msg.linear.x, twist_msg.angular.z);
            }
        }
    }

    if (kinematics.getBasePlatform() == Kinematics::ACKERMANN)
    {
        req_rpm = kinematics.getRPMAckermann(speed_x, steering_angle);
    }
    else
    {        
        // get the required rpm for each motor based on required velocities, and base used
        req_rpm = kinematics.getRPM(
            speed_x, 
            speed_y, 
            speed_z);
    }

    // get the current speed of each motor
    current_rpm1 = motor1_speed_controller.get_current_rpm();
    current_rpm2 = motor2_speed_controller.get_current_rpm();
#if NUM_BASE_MOTORS == 4
    current_rpm3 = motor3_speed_controller.get_current_rpm();
    current_rpm4 = motor4_speed_controller.get_current_rpm();
#endif

    motor1_speed_controller.set_target_rpm(req_rpm.motor1);
    motor2_speed_controller.set_target_rpm(req_rpm.motor2);
#if NUM_BASE_MOTORS == 4
    motor3_speed_controller.set_target_rpm(req_rpm.motor3);
    motor4_speed_controller.set_target_rpm(req_rpm.motor4);
#endif    

    if (kinematics.getBasePlatform() == Kinematics::ACKERMANN)
    {
        steering.set_angle(steering_angle);
    }

    Kinematics::velocities current_vel;
    if (kinematics.getBasePlatform() == Kinematics::ACKERMANN)
    {
        current_vel = kinematics.getVelocities(steering.get_current_angle(), current_rpm1, current_rpm2);
    }
    else
    {
        current_vel = kinematics.getVelocities(
            current_rpm1, 
            current_rpm2, 
            current_rpm3, 
            current_rpm4);

        //Logger::log_message(Logger::LogLevel::Info, "in, spd_x: %f spd_z: %f | mtr1, cur: %f, req: %f | mtr2, cur: %f, req: %f",
        //    speed_x, speed_z, current_rpm1, req_rpm.motor1, current_rpm2, req_rpm.motor2);
    }

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt,
        current_vel.linear_x,
        current_vel.linear_y,
        current_vel.angular_z);

        
    if (kinematics.getBasePlatform() == Kinematics::ACKERMANN)
    {
        steering.update();
    }        
    motor1_speed_controller.update();
    motor2_speed_controller.update();
#if NUM_BASE_MOTORS == 4
    motor3_speed_controller.update();
    motor4_speed_controller.update();
#endif    
}

void publishSensorData()
{
    imu_msg = imu.getData();
    mag_field_msg = imu.getMagneticField();

    struct timespec time_stamp = getTime();

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    mag_field_msg.header.stamp.sec = time_stamp.tv_sec;
    mag_field_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&imu_mag_field_publisher, &mag_field_msg, NULL));
}

void publishData()
{
    odom_msg = odometry.getData();
    
    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

#if defined(PUBLISH_MOTOR_DIAGS)
    motor1_diags.publish(time_stamp, req_rpm.motor1, current_rpm1, motor1_controller.getCurrent(), motor1_pid, motor1_encoder);
    motor2_diags.publish(time_stamp, req_rpm.motor2, current_rpm2, motor2_controller.getCurrent(), motor2_pid, motor2_encoder);
#if NUM_BASE_MOTORS == 4
    motor3_diags.publish(time_stamp, req_rpm.motor3, current_rpm3, motor3_controller.getCurrent(), motor3_pid, motor3_encoder);
    motor4_diags.publish(time_stamp, req_rpm.motor4, current_rpm4, motor4_controller.getCurrent(), motor4_pid, motor4_encoder);
#endif

    steering_motor_diags.publish(time_stamp, str_motor_speed_controller.get_target_rpm(), str_motor_speed_controller.get_current_rpm(), 0.0f,
                                 str_motor_speed_controller.get_pid(), str_motor_speed_controller.get_encoder());    
#endif
#if defined(PUBLISH_SERVO_DIAGS)
    steering_servo_diags.publish(time_stamp, steering_actuator.get_target_position(), steering_actuator.get_current_position(),
                                 steering_actuator.get_pid(), steering_actuator.get_encoder());
#endif
}

void rclErrorLoop(int n_times)
{
    // Disable power relay
    digitalWrite(MOTOR_RELAY_PWR_OUT, LOW);

    fullStop();
    if (micro_ros_init_successful) {
        Logger::log_message(Logger::LogLevel::Error, "Fail code %d", n_times);
    }


    while (true)
    {
        flashLED(n_times);
    }
}

void flashLED(int n_times)
{
    for (int i = 0; i < n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}

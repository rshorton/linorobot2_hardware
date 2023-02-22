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
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/joy.h>

#include "config.h"
#include "logger.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#define ENCODER_USE_INTERRUPTS
//#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder_none.h"
#include "encoder.h"
#include "motor_diagnostics.h"
#include "battery_diagnostics.h"
#include "util.h"
#include "steering.h"
#include "accel_pedal.h"
#include "INA226.h"
#include "hc_sr04.h"
#include "serial_bus_servo.h"
#include "ros_range_sensor.h"

#define TUNE_PID_LOOP // Allow tweaking of PID parameters via topic write
#define STEERING_PID_TWEAKS
#define DRIVER_CONTROL
#define JOY_BUTTON_ENABLES_DRIVER_CONTROL

const int ONE_SEC_IN_US = 1000000;
const int ONE_SEC_IN_MS = 1000;

const float MIN_MOVING_RPM_THRESH = 1.0;

const int BATTERY_DIAG_PUBLISH_PERIOD_MS = 10000;

const int DIST_SENSOR_UPDATE_PERIOD_MS = 100;

const float SPEED_SCALE_TURTLE = 0.25;
const float SPEED_SCALE_SLOW = 0.35;
const float SPEED_SCALE_NORMAL = 0.75;
const float SPEED_SCALE_LUDICROUS = 2.0;
const float SPEED_SCALE_CRAZY_LUDICROUS = 2.5;

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

const int ERR_BLINK_GENERAL = 2;
const int ERR_BLINK_IMU = 3;

#define RCCHECK(fn)                          \
    {                                        \
        rcl_ret_t temp_rc = fn;              \
        if ((temp_rc != RCL_RET_OK))         \
        {                                    \
            rclErrorLoop(ERR_BLINK_GENERAL); \
        }                                    \
    }
#define RCCHECK_WITH_BLINK_CODE(blink_code, fn) \
    {                                           \
        rcl_ret_t temp_rc = fn;                 \
        if ((temp_rc != RCL_RET_OK))            \
        {                                       \
            rclErrorLoop(blink_code);           \
        }                                       \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_subscription_t twist_subscriber;

#if defined(DRIVER_CONTROL)
rcl_subscription_t driver_control_subscriber;
rcl_subscription_t joy_subscriber;

std_msgs__msg__Bool driver_control_msg;
sensor_msgs__msg__Joy joy_msg;
int32_t button_data[9];
float axes_data[8];
#endif

rcl_subscription_t speed_scale_subscriber;
std_msgs__msg__Float32 speed_scale_msg;

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
rcl_timer_t battery_timer;
rcl_timer_t dist_sensor_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_joy_cmd_time = 0;
unsigned long prev_odom_update = 0;
bool micro_ros_init_successful = false;

// true to enable child driver using steering wheel and accel pedal
bool driver_control = false; 
float speed_scale = SPEED_SCALE_SLOW;

bool ackermann_teleop = true;
float speed_x_in = 0.0;
float steering_angle_in = 0.0;

Kinematics::rpm req_rpm;

int m1_dir_status = 0;
int m2_dir_status = 0;
int mstr_dir_status = 0;

// Steering wheel encoder
Encoder str_wheel_enc(STEERWHL_ENCODER_A, STEERWHL_ENCODER_B, 1, false);
// Steering motor/shaft encoder
Encoder str_motor_enc(STEERMTR_ENCODER_A, STEERMTR_ENCODER_B, 1, true);

// Rear wheel pulse counter
EncoderNone motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV, &m1_dir_status);
EncoderNone motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV, &m2_dir_status);

// Rear motors
Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B, &m1_dir_status);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B, &m2_dir_status);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

// Steering motor
Motor motor_str_controller(PWM_FREQUENCY, PWM_BITS, MOTOR_STR_INV, MOTOR_STR_PWM, MOTOR_STR_IN_A, MOTOR_STR_IN_B, &mstr_dir_status);

const int STR_PWM_MIN = -1000;
const int STR_PWM_MAX = 1000;
PID motor_str_pid(STR_PWM_MIN, STR_PWM_MAX, STR_PID_P, STR_PID_I, STR_PID_D);

Steering steering(STEER_RIGHT_LIMIT_IN, STEERING_RIGHT_SENSOR_POS, STEERING_FULL_RANGE_STEPS, STEERING_FULL_RANGE_DEG, WHEEL_SCALING_FACTOR,
                  motor_str_controller, str_motor_enc, str_wheel_enc, motor_str_pid);

AccelPedal accel_pedal(ACCEL_SW_IN, 0, ACCEL_LEVEL_IN, MIN_ACCEL_IN, MAX_ACCEL_IN, ONE_SEC_IN_MS / 20, 0.5);

INA226 pwr_mon_ctrl_bat;
INA226 pwr_mon_drive_bat;

HCSR04 dist_sensor0(0, HCSR04_TRIG1_OUT, HCSR04_ECHO1_IN, 3);
HCSR04 dist_sensor1(1, HCSR04_TRIG2_OUT, HCSR04_ECHO2_IN, 3);

RosRangeSensor range_sensor0(dist_sensor0, "hcsr04_back_left", "range/back_left");
RosRangeSensor range_sensor1(dist_sensor1, "hcsr04_back_right", "range/back_right");

bool range0_active = false;

BatteryDiags control_battery_diags("control", pwr_mon_ctrl_bat);
BatteryDiags drive_battery_diags("drive", pwr_mon_drive_bat);

Kinematics kinematics(
    Kinematics::ACKERMANN,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    FR_WHEELS_DISTANCE,
    LR_WHEELS_DISTANCE);

Odometry odometry;
IMU imu;

float current_rpm1 = 0.0;
float current_rpm2 = 0.0;
float current_rpm3 = 0.0;
float current_rpm4 = 0.0;

MotorDiags motor1_diags;
MotorDiags motor2_diags;

MotorDiags steering_diags;


bool pwr_relay_on = false;

bool estopAsserted()
{
    return digitalRead(ESTOP_IN) == 0;
}

void setup()
{
    pinMode(LED_PIN, OUTPUT);

    // FIX Move pin setup to classes which use them..
    pinMode(MOTOR_RELAY_PWR_OUT, OUTPUT);
    digitalWrite(MOTOR_RELAY_PWR_OUT, HIGH);
    pinMode(MOTOR_RELAY_PWR_IN, INPUT);
    pinMode(STEER_RIGHT_LIMIT_IN, INPUT_PULLUP);
    pinMode(ACCEL_SW_IN, INPUT_PULLUP);
    pinMode(FORW_REV_SW_IN, INPUT_PULLUP);
    pinMode(ESTOP_IN, INPUT_PULLUP);

    pwr_mon_ctrl_bat.begin(IIC_ADDR_INA226_CTRL_BAT);
    pwr_mon_ctrl_bat.configure();
    pwr_mon_ctrl_bat.calibrate(0.1, 4);
    pwr_mon_drive_bat.begin(IIC_ADDR_INA226_DRIVE_BAT);
    pwr_mon_drive_bat.configure();
    pwr_mon_drive_bat.calibrate(0.1, 20);

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

    flashLED(2);
}

void loop()
{
    static unsigned long prev_connect_test_time;
    // check if the agent got disconnected at 10Hz
    if (millis() - prev_connect_test_time >= 100)
    {
        prev_connect_test_time = millis();
        // check if the agent is connected
        if (RMW_RET_OK == rmw_uros_ping_agent(10, 2))
        {
            // reconnect if agent got disconnected or first time
            if (!micro_ros_init_successful)
            {
                createEntities();
                publishBatteryState();
                Logger::log_message(Logger::LogLevel::Error, "Micro ROS initialized");
            }
        }
        else if (micro_ros_init_successful)
        {
            // stop the robot when the agent is disconnected
            fullStop();
            // clean up micro-ROS components
            destroyEntities();
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
        if (steering.get_state() == Steering::State::kInit)
        {
            fullStop();
            steering.home();
            return;
        }
        else if (steering.get_state() == Steering::State::kHoming)
        {
            if (!is_moving())
            {
                steering.update(!estopAsserted());
                if (steering.get_state() != Steering::State::kHoming)
                {
                    steering.enable_external_control();
                }
            }
            return;
        }

        moveBase();
        publishData();
    }
}

void batteryCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        publishBatteryState();
    }
}

void twistCallback(const void *msgin)
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    prev_cmd_time = millis();
}

void setSpeedScale(float scale)
{
    if (scale >= SPEED_SCALE_TURTLE &&
        scale <= SPEED_SCALE_CRAZY_LUDICROUS)
    {
        speed_scale = scale;
    }
}

#if defined(DRIVER_CONTROL)
void setDriverControl(bool enable)
{
    if (enable)
    {
        steering.enable_steering_wheel();
    }
    else
    {
        steering.enable_external_control();
    }
}

void driverControlCallback(const void *msgin)
{
    if (driver_control != driver_control_msg.data)
    {
        driver_control = driver_control_msg.data;
        setDriverControl(driver_control);
    }
}
#endif

void distSensorCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        bool active = false;
        if (range0_active) {
            active = range_sensor0.update();
        } else {
            active = range_sensor1.update();
        }            
        if (!active) {
            range0_active = !range0_active;
        }
    }
}

void joyCallback(const void *msgin)
{
    RCLC_UNUSED(msgin);

#if defined(JOY_BUTTON_ENABLES_DRIVER_CONTROL)
    bool enable = (bool)joy_msg.buttons.data[JOY_BUTTON_LB];
    if (enable != driver_control)
    {
        driver_control = enable;
        setDriverControl(enable);
    }
#endif

    if (joy_msg.buttons.data[JOY_BUTTON_X])
    {
        setSpeedScale(SPEED_SCALE_SLOW);
    }
    else if (joy_msg.buttons.data[JOY_BUTTON_A])
    {
        setSpeedScale(SPEED_SCALE_NORMAL);
    }
    else if (joy_msg.buttons.data[JOY_BUTTON_B])
    {
        setSpeedScale(SPEED_SCALE_LUDICROUS);
    }

    ackermann_teleop = joy_msg.axes.data[JOY_AXIS_LEFT_TRIGGER_BUTTON] == -1;
    if (ackermann_teleop)
    {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        prev_joy_cmd_time = millis();

        speed_x_in = joy_msg.axes.data[JOY_AXIS_LEFT_STICK_UD] * speed_scale;
        steering_angle_in = joy_msg.axes.data[JOY_AXIS_RIGHT_STICK_LR] * STEERING_FULL_RANGE_DEG / 2 / 180.0 * M_PI;
    }
    else
    {
        speed_x_in = 0.0;
        steering_angle_in = 0.0;
    }
}

void speedScaleCalback(const void *msgin)
{
    setSpeedScale(speed_scale_msg.data);
}

#if defined(TUNE_PID_LOOP)
void pidKpCallback(const void *msgin)
{
#if defined(STEERING_PID_TWEAKS)
    motor_str_pid.updateKp(pid_kp_msg.data);
#else
    motor1_pid.updateKp(pid_kp_msg.data);
    motor2_pid.updateKp(pid_kp_msg.data);
    motor3_pid.updateKp(pid_kp_msg.data);
    motor4_pid.updateKp(pid_kp_msg.data);
#endif
}

void pidKdCallback(const void *msgin)
{
#if defined(STEERING_PID_TWEAKS)
    motor_str_pid.updateKp(pid_kd_msg.data);
#else
    motor1_pid.updateKd(pid_kd_msg.data);
    motor2_pid.updateKd(pid_kd_msg.data);
    motor3_pid.updateKd(pid_kd_msg.data);
    motor4_pid.updateKd(pid_kd_msg.data);
#endif
}

void pidKiCallback(const void *msgin)
{
#if defined(STEERING_PID_TWEAKS)
    motor_str_pid.updateKp(pid_ki_msg.data);
#else
    motor1_pid.updateKi(pid_ki_msg.data);
    motor2_pid.updateKi(pid_ki_msg.data);
    motor3_pid.updateKi(pid_ki_msg.data);
    motor4_pid.updateKi(pid_ki_msg.data);
#endif
}
#endif

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
        "odom/unfiltered"));

    // create IMU publisher
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"));

    control_battery_diags.create(node);
    drive_battery_diags.create(node);
    range_sensor0.create(node);
    range_sensor1.create(node);
    Logger::create_logger(node);   

#if defined(PUBLISH_MOTOR_DIAGS)
    // create diagnostics publisher
    motor1_diags.create(node, 1);
    motor2_diags.create(node, 2);
    steering_diags.create(node, 3);
#endif

    RCCHECK_WITH_BLINK_CODE(3, rclc_subscription_init_default(
                                   &speed_scale_subscriber,
                                   &node,
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                   "speed_scale"));

#if defined(TUNE_PID_LOOP)
    RCCHECK_WITH_BLINK_CODE(3, rclc_subscription_init_default(
                                   &pid_kp_subscriber,
                                   &node,
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                   "pid_kp"));
    RCCHECK(rclc_subscription_init_default(
        &pid_kd_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "pid_kd"));

    RCCHECK(rclc_subscription_init_default(
        &pid_ki_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "pid_ki"));
#endif

#if defined(DRIVER_CONTROL)
    RCCHECK(rclc_subscription_init_default(
        &driver_control_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "driver_control"));

    joy_msg.buttons.data = button_data;
    joy_msg.buttons.size = 0;
    joy_msg.buttons.capacity = sizeof(button_data);
    joy_msg.axes.data = axes_data;
    joy_msg.axes.size = 0;
    joy_msg.axes.capacity = sizeof(axes_data);

    RCCHECK(rclc_subscription_init_default(
        &joy_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
        "joy"));
#endif

    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback));

    // create timer for monitoring the batteries
    const unsigned int battery_status_timeout = BATTERY_DIAG_PUBLISH_PERIOD_MS;
    RCCHECK(rclc_timer_init_default(
        &battery_timer,
        &support,
        RCL_MS_TO_NS(battery_status_timeout),
        batteryCallback));

    // create timer for updating distance measurements
    const unsigned int dist_sensor_timeout = DIST_SENSOR_UPDATE_PERIOD_MS;
    RCCHECK(rclc_timer_init_default(
        &dist_sensor_timer,
        &support,
        RCL_MS_TO_NS(dist_sensor_timeout),
        distSensorCallback));

    executor = rclc_executor_get_zero_initialized_executor();
    // 9 handles = 7 subscriptions + 3 timers  (including ifdef'ed subs)
    RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator));

#if defined(DRIVER_CONTROL)
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &driver_control_subscriber,
        &driver_control_msg,
        &driverControlCallback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &joy_subscriber,
        &joy_msg,
        &joyCallback,
        ON_NEW_DATA));
#endif

    RCCHECK_WITH_BLINK_CODE(4, rclc_executor_add_subscription(
                                   &executor,
                                   &speed_scale_subscriber,
                                   &speed_scale_msg,
                                   &speedScaleCalback,
                                   ON_NEW_DATA));

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
#endif
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &battery_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &dist_sensor_timer));

    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_PIN, HIGH);
    micro_ros_init_successful = true;
}

void destroyEntities()
{
    digitalWrite(LED_PIN, LOW);

    control_battery_diags.destroy(node);
    drive_battery_diags.destroy(node);
    range_sensor0.destroy(node);
    range_sensor1.destroy(node);
    Logger::destroy_logger(node);   

#if defined(PUBLISH_MOTOR_DIAGS)
    motor1_diags.destroy(node);
    motor2_diags.destroy(node);
    steering_diags.destroy(node);
#endif

    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&imu_publisher, &node);

#if defined(DRIVER_CONTROL)
    rcl_subscription_fini(&driver_control_subscriber, &node);
    rcl_subscription_fini(&joy_subscriber, &node);
#endif
    rcl_subscription_fini(&speed_scale_subscriber, &node);
    rcl_subscription_fini(&twist_subscriber, &node);

#if defined(TUNE_PID_LOOP)
    rcl_subscription_fini(&pid_kp_subscriber, &node);
    rcl_subscription_fini(&pid_kd_subscriber, &node);
    rcl_subscription_fini(&pid_ki_subscriber, &node);
#endif

    rcl_node_fini(&node);

    rcl_timer_fini(&control_timer);
    rcl_timer_fini(&battery_timer);
    rcl_timer_fini(&dist_sensor_timer);

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
}

bool is_moving()
{
    return abs(motor1_encoder.getRPM()) > MIN_MOVING_RPM_THRESH ||
           abs(motor2_encoder.getRPM()) > MIN_MOVING_RPM_THRESH;
}

float steer(float steering_angle)
{
    float angle_deg = -steering_angle * 180.0 / M_PI;
    angle_deg = steering.set_position_deg(angle_deg);
    return -angle_deg * M_PI / 180.0;
}

float getSteeringPos()
{
    return -steering.get_actual_pos_deg() * M_PI / 180.0;
}

bool directionChange(float cur_rpm, float req_rpm)
{
    return abs(cur_rpm) > 1.0 && sgn(cur_rpm) != sgn(req_rpm);
}

// For converting twist msg to Ackermann x vel and steering angle
// See convertTransRotVelToSteeringAngle() in 
// https://github.com/rst-tu-dortmund/teb_local_planner/blob/melodic-devel/src/teb_local_planner_ros.cpp
float rotational_vel_to_steering_angle(float x_vel, float w_vel, float wheelbase)
{
    if (x_vel == 0.0 || w_vel == 0.0)
    {
        return 0.0;
    }
    float radius = x_vel / w_vel;

// FIX move to config (or calculate based on max steering angle and wheelbase)
const float MIN_TURNING_RADIUS = 1.5;

    if (fabs(radius) < MIN_TURNING_RADIUS)
    {
        radius = MIN_TURNING_RADIUS * (radius < 0? -1: 1);
    }

    // FIX - why is this factor needed?  Without it the steering angle is
    // either maxed to the left/right, or center.
    radius *= 1.75;
    return atan(wheelbase/radius);
}

void moveBase()
{
    // get the current speed of each motor
    current_rpm1 = motor1_encoder.getRPM();
    current_rpm2 = motor2_encoder.getRPM();

    bool estop = estopAsserted();

#if defined(DRIVER_CONTROL)
    if (driver_control)
    {
        accel_pedal.update();
        float level = accel_pedal.get_level();
        bool forward = digitalRead(FORW_REV_SW_IN);

        float req_rpm = 0.0;
        // Must press left-top joystick button and estop/rf sw must allow control
        if (joy_msg.buttons.data[JOY_BUTTON_LB] && !estop)
        {
            req_rpm = level;
            if (forward)
            {
                req_rpm *= MAX_MANUAL_RPM_FORWARD * speed_scale;
            }
            else
            {
                req_rpm *= -MAX_MANUAL_RPM_REVERSE * SPEED_SCALE_SLOW;
            }
        }

        // Don't drive motor in the opposite direction until it stops
        if (directionChange(current_rpm1, req_rpm))
        {
            req_rpm = 0.0;
        }
        motor1_controller.spin(motor1_pid.compute(req_rpm, current_rpm1));
        motor2_controller.spin(motor2_pid.compute(req_rpm, current_rpm2));
    }
    else
#endif
    {
        float speed_x = 0.0;
        float steering_angle = 0.0;

        if (ackermann_teleop)
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
            // navigation planner)

            // brake if there's no command received, or when it's only the first command sent
            if (((millis() - prev_cmd_time) > 200))
            {
                digitalWrite(LED_PIN, HIGH);
            }
            else
            {
                speed_x = twist_msg.linear.x;
                // Calculate steering angle from x velocity, twist and wheelbase
                // http://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20car-like%20robots
                steering_angle = rotational_vel_to_steering_angle(twist_msg.linear.x, twist_msg.angular.z, FR_WHEELS_DISTANCE);
            }
        }

        // get the required rpm for each motor based on required velocities
        req_rpm = kinematics.getRPM(
            speed_x,
            0,
            steering_angle);

        // Don't drive motor in the opposite direction until it stops
        if (directionChange(current_rpm1, req_rpm.motor1) || estop)
        {
            req_rpm.motor1 = 0.0;
        }

        if (directionChange(current_rpm2, req_rpm.motor2) || estop)
        {
            req_rpm.motor2 = 0.0;
        }

        // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
        // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
        motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
        motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));

        if (kinematics.getBasePlatform() == Kinematics::ACKERMANN)
        {
            steer(steering_angle);
        }
    }

    Kinematics::velocities current_vel;
    if (kinematics.getBasePlatform() == Kinematics::ACKERMANN)
    {
        current_vel = kinematics.getVelocities(getSteeringPos(), current_rpm1, current_rpm2);
    }
    else
    {
        current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    }

    //Logger::log_message(Logger::LogLevel::Error, "Steering pos %f", steering.get_actual_pos_deg());

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt,
        current_vel.linear_x,
        current_vel.linear_y,
        current_vel.angular_z);

    steering.update(!estop);
}

void publishBatteryState()
{
    control_battery_diags.publish();
    drive_battery_diags.publish();
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
    steering_diags.publish(time_stamp, steering.get_position(), steering.get_actual_pos(), 0.0, motor_str_pid);
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

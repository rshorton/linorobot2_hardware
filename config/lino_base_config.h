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

#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

// Define to enable motor diag support for publishing
// motor status for plotting
#define PUBLISH_MOTOR_DIAGS

#define LED_PIN 13 //used for debugging status

//uncomment the base you're building
//#define LINO_BASE DIFFERENTIAL_DRIVE     // 2WD and Tracked robot w/ 2 motors
//#define LINO_BASE SKID_STEER             // 4WD robot
// #define LINO_BASE MECANUM               // Mecanum drive robot

//uncomment the motor driver you're using
//#define USE_GENERIC_2_IN_MOTOR_DRIVER    // Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE) pin ie. L298, L293, VNH5019
//#define USE_GENERIC_1_IN_MOTOR_DRIVER    // Motor drivers with 1 Direction Pin(INA) and 1 PWM(ENABLE) pin.
#define USE_BTS7960_MOTOR_DRIVER           // BTS7970 Motor Driver
// #define USE_ESC_MOTOR_DRIVER            // Motor ESC for brushless motors

//uncomment the IMU you're using
#define USE_GY85_IMU
// #define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU

#define K_P 0.6                             // P constant
#define K_I 0.3                             // I constant
#define K_D 1.5                             // D constant

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)  
         BACK
*/

#define MOTOR_MAX_RPM 150                   // motor's max RPM          
#define MAX_RPM_RATIO 0.95                  // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
#define MOTOR_OPERATING_VOLTAGE 12          // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 18          // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 18     // current voltage reading of the power connected to the motor (used for calibration)

                                            // Max wheel RPM when in manual (human) drive mode
#define MAX_MANUAL_RPM_FORWARD 70           //  Forward
#define MAX_MANUAL_RPM_REVERSE 40           //  Reverse

#define COUNTS_PER_REV1 192                 // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 192                 // wheel2 encoder's no of ticks per rev

#define WHEEL_DIAMETER 0.280                // wheel's diameter in meters
#define FR_WHEELS_DISTANCE 0.708            // distance between front and back wheels
#define LR_WHEELS_DISTANCE 0.660            // distance between left and right wheels
#define PWM_BITS 10                         // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 20000                 // PWM Frequency

#define STR_PID_P   40.0                    // Steering motor PID values 
#define STR_PID_I   10.0
#define STR_PID_D   70.0

#define STEERING_MAX_RANGE_STEPS  42        // Steering max range in steering encoder units
#define STEERING_FULL_RANGE_STEPS 40        // Steering usable range in steering encoder units
#define STEERING_FULL_RANGE_DEG   48        // Steering range in degrees
#define STEERING_HALF_RANGE_DEG   (STEERING_FULL_RANGE_DEG/2)
                                            // Left sensor position in encoder units (0 centered, - => left, + => right)
#define STEERING_LEFT_SENSOR_POS  (-STEERING_MAX_RANGE_STEPS/2)    

#define MIN_ACCEL_IN 750                    // Accel pedal analog input value when not pressed
#define MAX_ACCEL_IN 300                    // Accel pedal analog input value when fully pressed

// INVERT ENCODER COUNTS
#define MOTOR1_ENCODER_INV true
#define MOTOR2_ENCODER_INV true

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV false
#define MOTOR2_INV false
#define MOTOR_STR_INV false

// ENCODER PINS
// Only one Hall effect sensor for detecting rate, but not direction for wheel motors
#define MOTOR1_ENCODER_A 8
#define MOTOR1_ENCODER_B -1

#define MOTOR2_ENCODER_A 9
#define MOTOR2_ENCODER_B -1

// Quadrature encoder used for steering sensors

#define STEERWHL_ENCODER_A 30
#define STEERWHL_ENCODER_B 31

#define STEERMTR_ENCODER_A 26
#define STEERMTR_ENCODER_B 27

// MOTOR PINS
#ifdef USE_GENERIC_2_IN_MOTOR_DRIVER
  #define MOTOR1_PWM 21 //Pin no 21 is not a PWM pin on Teensy 4.x, you can swap it with pin no 1 instead.
  #define MOTOR1_IN_A 20
  #define MOTOR1_IN_B 1 

  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A 6
  #define MOTOR2_IN_B 8

  #define MOTOR3_PWM 22
  #define MOTOR3_IN_A 23
  #define MOTOR3_IN_B 0

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A 3
  #define MOTOR4_IN_B 2

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 

#ifdef USE_GENERIC_1_IN_MOTOR_DRIVER
  #define MOTOR1_PWM  4   //Pin no 21 is not a PWM pin on Teensy 4.x, you can use pin no 1 instead.
  #define MOTOR1_IN_A 3
  #define MOTOR1_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_CURRENT A12

  #define MOTOR2_PWM  5
  #define MOTOR2_IN_A 6
  #define MOTOR2_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_CURRENT A13

  #define MOTOR3_PWM  1
  #define MOTOR3_IN_A 20
  #define MOTOR3_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_CURRENT A11

  #define MOTOR4_PWM  22
  #define MOTOR4_IN_A 23
  #define MOTOR4_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_CURRENT A10

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 

#ifdef USE_BTS7960_MOTOR_DRIVER
  #define MOTOR1_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_A 11
  #define MOTOR1_IN_B 12

  #define MOTOR2_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_A 28
  #define MOTOR2_IN_B 29

  #define MOTOR_STR_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR_STR_IN_A 24
  #define MOTOR_STR_IN_B 25
  
  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif

#ifdef USE_ESC_MOTOR_DRIVER
  #define MOTOR1_PWM 21 //Pin no 21 is not a PWM pin on Teensy 4.x. You can use pin no 1 instead.
  #define MOTOR1_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR3_PWM 22 
  #define MOTOR3_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define PWM_MAX 400
  #define PWM_MIN -PWM_MAX
#endif

// Motor power relay control related
// Active high output is connected thru
// emergency stop switch and wireless switch
// to the control input of the relay.
#define MOTOR_RELAY_PWR_OUT 6
// The control input of the relay is monitored
// using this input.
#define MOTOR_RELAY_PWR_IN  7

// Misc Inputs
#define FORW_REV_SW_IN      32  // Active hi (forw) down position
#define ACCEL_SW_IN         5   // Active low (when pressed)
#define ACCEL_LEVEL_IN      A17 // Accelerator level ~2.5 (none)-> ~0(max)
#define IGN_SW_IN           10  // Active hi (switch pressed)
#define STEER_LEFT_LIMIT_IN 3   // Active low left limit Hall sensor
#define ESTOP_IN            7   // Active low (emergency stop button pressed or RF switch is off)

// I2C addresses
#define IIC_ADDR_INA226_CTRL_BAT    0x40  // INA226 connected to control circuitry battery
#define IIC_ADDR_INA226_DRIVE_BAT   0x41  // INA226 connected to motor battery (shunt not connected)

#endif

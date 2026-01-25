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
//#define LINO_BASE SKID_STEER               // 4WD robot
//#define LINO_BASE MECANUM                // Mecanum drive robot
#define LINO_BASE ACKERMANN                // Front steering with rear motors

//#define NUM_BASE_MOTORS 4
#define NUM_BASE_MOTORS 2

//uncomment the motor driver you're using
//#define USE_GENERIC_2_IN_MOTOR_DRIVER    // Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE) pin ie. L298, L293, VNH5019
#define USE_GENERIC_1_IN_MOTOR_DRIVER      // Motor drivers with 1 Direction Pin(INA) and 1 PWM(ENABLE) pin.
// #define USE_BTS7960_MOTOR_DRIVER        // BTS7970 Motor Driver
// #define USE_ESC_MOTOR_DRIVER            // Motor ESC for brushless motors

//uncomment the IMU you're using
#define USE_GY85_IMU
// #define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU

// Support approx 4rpm min speed with 24V powerwheels motors powered with Ryobi 18V Li battery (18v-13v)
#define K_P 0.5                            // P constant
#define K_I 0.3                            // I constant
#define K_D 2.8                            // D constant

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)  
         BACK
*/

//define your robot' specs here
#define MOTOR_MAX_RPM 186                  // motor's max RPM          
#define MAX_RPM_RATIO 0.95                 // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
#define MOTOR_OPERATING_VOLTAGE 18         // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 18         // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 18    // current voltage reading of the power connected to the motor (used for calibration)

#define COUNTS_PER_REV1 (120)               // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 (120)               // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 (120)               // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 (120)               // wheel4 encoder's no of ticks per rev

#define WHEEL_DIAMETER (6.0*0.0254)         // wheel's diameter in meters
#define FR_WHEELS_DISTANCE 0.366            // distance between front and back wheels
#define LR_WHEELS_DISTANCE 0.436            // distance between left and right wheels
#define PWM_BITS 10                         // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 20000                 // PWM Frequency

// INVERT ENCODER COUNTS
#define MOTOR1_ENCODER_INV true
#define MOTOR2_ENCODER_INV false
#define MOTOR3_ENCODER_INV true 
#define MOTOR4_ENCODER_INV false

#define MOTOR_STR_ENCODER_INV false

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV true
#define MOTOR2_INV false
#define MOTOR3_INV true
#define MOTOR4_INV false

#define MOTOR_STR_INV false

// ENCODER PINS

// Quadrature encoder used for steering sensors
#define STEERMTR_ENCODER_A 30
#define STEERMTR_ENCODER_B 31

#if NUM_BASE_MOTORS == 4
#define MOTOR1_ENCODER_A 9
#define MOTOR1_ENCODER_B -1

#define MOTOR2_ENCODER_A 12
#define MOTOR2_ENCODER_B -1

#define MOTOR3_ENCODER_A 14
#define MOTOR3_ENCODER_B -1

#define MOTOR4_ENCODER_A 17
#define MOTOR4_ENCODER_B -1
#else
#define MOTOR1_ENCODER_A 14
#define MOTOR1_ENCODER_B -1

#define MOTOR2_ENCODER_A 17
#define MOTOR2_ENCODER_B -1

#endif

// MOTOR PINS

#ifdef USE_GENERIC_1_IN_MOTOR_DRIVER
#if NUM_BASE_MOTORS == 4
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
#else
  #define MOTOR1_PWM  1
  #define MOTOR1_IN_A 20
  #define MOTOR1_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_CURRENT A11

  #define MOTOR2_PWM  22
  #define MOTOR2_IN_A 23
  #define MOTOR2_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_CURRENT A10

  #define MOTOR_STR_PWM 4 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR_STR_IN_A 3
  #define MOTOR_STR_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

#endif
  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 


//////////////////////////////////
// Steering related
//
// Steering Architecture:
//   - SteeringUsingLinearActuator object is the top level controller.  Its input is the desired steering angle.
//     - Uses a SteeringAngleToActuatorMapperEbotAckerman object to map the steering angle to the steering actuator position.
//     - Uses a LinearActuator object to control the steering 'rack' position (rack driven by a lead screw)
//        - LinearActuator
//          - Uses a PID loop to control the rack position with the PID output controlling a MotorSpeedController which
//            controls the speed of the motor spinning the rack lead screw.
//              - MotorSpeedController
//                  - Uses a PID loop to control a DC motor using a Motor controller object
//                      - Motor object uses a PWM output to control the motor speed.
//////////////////////////////////

// Steering actuator

#define STR_ACT_RPM_MIN -110                 // Min/max RPM when controlling the motor
#define STR_ACT_RPM_MAX 110 

#define STR_ACT_PID_P   0.03f               // Actuator PID values 
#define STR_ACT_PID_I   0.0005f
#define STR_ACT_PID_D   0.3f

#define STR_ACT_MAX_POS 13760               // Max position of the actuator in encoder units.
#define STR_ACT_POS_THRESH -1

#define STR_LEFT_LIMIT_IN  39               // Active low steering limit sw used when homing the actuator

// Steering motor speed controller

#define STR_SPD_PWM_MIN -1000               // Min max PWM output values when controlling the motor    
#define STR_SPD_PWM_MAX 1000

#define STR_SPD_PID_P   2.0f                // Motor speed controller PID values 
#define STR_SPD_PID_I   1.0f
#define STR_SPD_PID_D   0.0f

#define STR_MOTOR_ENC_TICKS_PER_REV (64*70) // Encoder ticks per one rev of output shaft (64 ticks per motor rev, 70:1 gear ratio)

// Motor power relay control related

// Active high output is connected thru
// emergency stop switch and wireless switch
// to the control input of the relay.
#define MOTOR_RELAY_PWR_OUT 15
// The control input of the relay is monitored
// using this input.
#define MOTOR_RELAY_PWR_IN 16

#define ESTOP_IN           MOTOR_RELAY_PWR_IN  // Active low (emergency stop button pressed or RF switch is off)

#endif

// Original file:
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
#include <stdio.h>
#include "config.h"
#include "motor.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder_none.h"
#include "encoder.h"
#include "kinematics.h"
#include "pid.h"
#include "util.h"
#include "steering.h"

#define SAMPLE_TIME 8 // s
#define ONE_SEC_IN_US 1000000

const int STEERING_FULL_RANGE = 28;

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

// Steering motor
Motor motor_str_controller(PWM_FREQUENCY, PWM_BITS, MOTOR_STR_INV, MOTOR_STR_PWM, MOTOR_STR_IN_A, MOTOR_STR_IN_B, &mstr_dir_status);

const int STR_PWM_MIN = -1000;
const int STR_PWM_MAX = 1000;
PID motor_str_pid(STR_PWM_MIN, STR_PWM_MAX, 40, 10.0, 50.0);

Steering steering(STEER_LEFT_LIMIT_IN, STEERING_FULL_RANGE, 1.5, motor_str_controller, str_motor_enc, str_wheel_enc, motor_str_pid);

Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE);

const int total_motors = 2;
long long int counts_per_rev[total_motors];
Motor *motors[] = {&motor1_controller, &motor2_controller};
EncoderNone *encoders[] = {&motor1_encoder, &motor2_encoder};
String labels[] = {"REAR LEFT - M1: ", "REAR RIGHT - M2: "};

long str_wheel_enc_pos = 0;
long str_motor_enc_pos = 0;

void setup()
{
    Serial.begin(9600);
    while (!Serial)
    {
    }
    Serial.println("Sampling process will spin the motors at its maximum RPM.");
    Serial.println("Please ensure that the robot is ELEVATED and there are NO OBSTRUCTIONS to the wheels.");
    Serial.println("");
    Serial.println("Type 'r' to spin the rear motors one at a time.");
    Serial.println("Type 's' to run steering control test.");
    Serial.println("Type 'e' to run external steering control test.");
    Serial.println("");

    pinMode(MOTOR_RELAY_PWR_OUT, OUTPUT);
    digitalWrite(MOTOR_RELAY_PWR_OUT, HIGH);
    pinMode(MOTOR_RELAY_PWR_IN, INPUT);
    pinMode(STEER_LEFT_LIMIT_IN, INPUT_PULLUP);
}

void loop()
{
    while (Serial.available())
    {
        char character = Serial.read();
        Serial.print(character);
        delay(1);
        if (character == 'r')
        {
            Serial.println("\r\n");
            sampleMotors(1);
        }
        else if (character == 's')
        {
            Serial.println("\r\n");
            testSteering(false);
        }
        else if (character == 'e')
        {
            Serial.println("\r\n");
            testSteering(true);
        }
        else if (character == '\r')
        {
            Serial.println("");
        }
    }
}

void sampleMotors(bool show_summary)
{
    float measured_voltage = constrain(MOTOR_POWER_MEASURED_VOLTAGE, 0, MOTOR_OPERATING_VOLTAGE);
    float scaled_max_rpm = ((measured_voltage / MOTOR_OPERATING_VOLTAGE) * MOTOR_MAX_RPM);
    float total_rev = scaled_max_rpm * (SAMPLE_TIME / 60.0);

    for (int i = 0; i < total_motors; i++)
    {
        encoders[i]->write(0);
    }

    for (int i = 0; i < total_motors; i++)
    {
        Serial.print("SPINNING ");
        Serial.print(labels[i]);

        unsigned long start_time = micros();
        unsigned long last_status = micros();

        encoders[i]->write(0);
        while (true)
        {
            if (micros() - start_time >= SAMPLE_TIME * ONE_SEC_IN_US)
            {
                motors[i]->spin(0);
                Serial.println("");
                break;
            }

            if (micros() - last_status >= ONE_SEC_IN_US)
            {
                last_status = micros();
                Serial.print(".");
            }

            motors[i]->spin(300);

            Serial.print(encoders[0]->read());
            Serial.print(" ");
            Serial.print(encoders[1]->read());
            Serial.println("\r\n");
        }
        Serial.println("Next motor");

        counts_per_rev[i] = encoders[i]->read() / total_rev;
    }
    Serial.println("Finished");
    if (show_summary)
        printSummary();
}

void printSummary()
{
    Serial.println("\r\n================MOTOR ENCODER READINGS================");
    Serial.print(labels[0]);
    Serial.print(encoders[0]->read());
    Serial.print(" ");

    Serial.print(labels[1]);
    Serial.println(encoders[1]->read());

    Serial.println("================COUNTS PER REVOLUTION=================");
    Serial.print(labels[0]);
    Serial.print(counts_per_rev[0]);
    Serial.print(" ");

    Serial.print(labels[1]);
    Serial.println(counts_per_rev[1]);

    Serial.println("====================MAX VELOCITIES====================");
    float max_rpm = kinematics.getMaxRPM();

    Kinematics::velocities max_linear = kinematics.getVelocities(max_rpm, max_rpm, max_rpm, max_rpm);
    Kinematics::velocities max_angular = kinematics.getVelocities(-max_rpm, max_rpm, -max_rpm, max_rpm);

    Serial.print("Linear Velocity: +- ");
    Serial.print(max_linear.linear_x);
    Serial.println(" m/s");

    Serial.print("Angular Velocity: +- ");
    Serial.print(max_angular.angular_z);
    Serial.println(" rad/s");
}

void testSteering(bool external)
{
    if (steering.get_state() == Steering::State::kInit)
    {
        Serial.println("Homing the steering control...");
        steering.home();
        while (steering.get_state() == Steering::State::kHoming)
        {
            steering.update();
        }
    }

    if (steering.get_state() != Steering::State::kWheel)
    {
        Serial.println("Error, expected steering to be homed");
        return;
    }
    Serial.println("Steering homed");

    if (external)
    {
        if (!steering.enable_external_control())
        {
            Serial.println("Error, failed to enable external steering control");
            return;
        }
        Serial.println("Enabled external control.  Press 'l' or 'r' to change the steering.  Press 'e' continue...");

        while (true)
        {
            if (Serial.available())
            {
                char character = Serial.read();
                Serial.print(character);
                delay(1);
                int8_t delta = 0;
                if (character == 'l')
                {
                    delta = -1;
                }
                else if (character == 'r')
                {
                    delta = 1;
                }
                else if (character == 'e')
                {
                    break;
                }
                if (delta != 0)
                {
                    if (!steering.set_position(steering.get_position() + delta))
                    {
                        Serial.println("Error, failed to set steering position");
                    }
                }
            }
            steering.update();
        }
        if (!steering.enable_steering_wheel())
        {
            Serial.println("Error, failed to enable steering wheel control");
            return;
        }
    }

    Serial.println("Steering wheel control mode enabled.  Turn the steering wheel to change steering...");

    while (true)
    {
        steering.update();
    }
}
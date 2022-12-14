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

#define SAMPLE_TIME 8 // s
#define ONE_SEC_IN_US 1000000

int m1_dir_status = 0;
int m2_dir_status = 0;
int mstr_dir_status = 0;

// Steering wheel encoder
Encoder str_wheel_enc(STEERWHL_ENCODER_A, STEERWHL_ENCODER_B, 1);
// Steering motor/shaft encoder
Encoder str_motor_enc(STEERMTR_ENCODER_A, STEERMTR_ENCODER_B, 1);

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
PID motor_str_pid(STR_PWM_MIN, STR_PWM_MAX, 70, 10.0, 50.0);

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
    Serial.println("Type 'l' to find steering limits and center.");
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
            testSteeringMotorControl(true);
        }
        else if (character == 'l')
        {
            Serial.println("\r\n");
            testAutoCenter();
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

void testSteeringMotor()
{
    int pwm = 800;
    unsigned long next_speed_chg = micros();

    while (true)
    {
        if (micros() > next_speed_chg)
        {
            pwm += 40;
            if (pwm > 4000)
            {
                motor_str_controller.spin(0);
                break;
            }
            next_speed_chg = micros() + 5 * ONE_SEC_IN_US;
            motor_str_controller.spin(pwm);
            Serial.print("Speed: ");
            Serial.print(pwm);
            Serial.println("");
        }
    }
}

const int LEFT_STEERING_LIMIT = 14;
const int RIGHT_STEERING_LIMIT = -14;

// Test steering control
void testSteeringMotorControl(bool reset)
{
    if (reset)
    {
        str_wheel_enc.readAndReset();
        str_motor_enc.readAndReset();
    }

    unsigned long next_update = 0;
    unsigned long next_status = 0;

    bool update_ctrl = false;
    bool update_status = false;

    long str_whl_pos = 0;
    long str_shaft_pos = 0;
    unsigned long now = 0;

    int last_diff = 0;

    while (true)
    {
        now = micros();
        update_ctrl = now > next_update;
        update_status = now > next_status;

        if (update_ctrl || update_status)
        {
            str_whl_pos = -1 * str_wheel_enc.read();
            str_whl_pos = 2 * str_whl_pos / 3;

            if (str_whl_pos > LEFT_STEERING_LIMIT)
            {
                str_whl_pos = LEFT_STEERING_LIMIT;
            }
            else if (str_whl_pos < RIGHT_STEERING_LIMIT)
            {
                str_whl_pos = RIGHT_STEERING_LIMIT;
            }

            str_shaft_pos = str_motor_enc.read();
        }

        if (update_ctrl)
        {
            next_update = now + 50 * 1000;

            int new_diff = str_shaft_pos - str_whl_pos;
            if (sgn(last_diff) != sgn(new_diff))
            {
                motor_str_controller.spin(0);
                motor_str_pid.reset();
                next_update = now + 50 * 1000;
            }
            else
            {
                int whl_pos = abs(new_diff) > 1 ? str_whl_pos : str_shaft_pos;
                int new_pwm = motor_str_pid.compute(whl_pos, str_shaft_pos, false);
                motor_str_controller.spin(new_pwm);
                if (new_pwm != 0)
                {
                    Serial.println(new_pwm);
                }
            }
            last_diff = new_diff;
        }

        if (update_status || update_ctrl)
        {
            next_status = now + 500 * 1000;
            Serial.print("      StrWhl: ");
            Serial.print(str_whl_pos);
            Serial.print("      StrShaft: ");
            Serial.println(str_shaft_pos);
        }
    }
}

void testAutoCenter()
{
    str_motor_enc.readAndReset();

    unsigned long next_update = 0;
    unsigned long next_status = 0;
    unsigned long now = 0;

    int next_pos = 0;

    while (true)
    {
        now = micros();
        if (now > next_update)
        {
            next_update = now + 50 * 1000;
            int str_shaft_pos = str_motor_enc.read();

            if (digitalRead(STEER_LEFT_LIMIT_IN) == 0)
            {
                Serial.println("At left limit");
                break;
            }

            int diff = next_pos - str_shaft_pos;

            if (now > next_status)
            {
                next_status = now + 750 * 1000;
                Serial.print("Checking\n\r  Test: ");
                Serial.print(next_pos);
                Serial.print(", actual: ");
                Serial.print(str_shaft_pos);
                Serial.print(", diff: ");
                Serial.println(diff);
            }
            if (diff <= 0)
            {
                next_pos++;
            }
            int pwm = motor_str_pid.compute(next_pos, str_shaft_pos, false);
            motor_str_controller.spin(pwm);
            Serial.print("PWM ");
            Serial.println(pwm);
        }
    }
    str_wheel_enc.readAndReset();
    str_motor_enc.write(LEFT_STEERING_LIMIT);
    next_pos = str_motor_enc.read();

    while (true)
    {
        now = micros();
        if (now > next_update)
        {
            next_update = now + 50 * 1000;
            int str_shaft_pos = str_motor_enc.read();

            if (str_shaft_pos <= 0) {
                break;
            }

            int diff = next_pos - str_shaft_pos;

            if (now > next_status)
            {
                next_status = now + 750 * 1000;
                Serial.print("Checking\n\r  Test: ");
                Serial.print(next_pos);
                Serial.print(", actual: ");
                Serial.print(str_shaft_pos);
                Serial.print(", diff: ");
                Serial.println(diff);
            }
            if (diff >= 0)
            {
                next_pos--;
            }
            int pwm = motor_str_pid.compute(next_pos, str_shaft_pos, false);
            motor_str_controller.spin(pwm);
            Serial.print("PWM ");
            Serial.println(pwm);
        }
    }
    testSteeringMotorControl(false);
}

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
//#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder_none.h"
#include "encoder.h"
#include "kinematics.h"
#include "pid.h"
#include "util.h"
#include "steering.h"
#include "accel_pedal.h"
#include "INA226.h"
#include "hc_sr04.h"

#undef PANNING_DIST_SENSOR

#if defined(PANNING_DIST_SENSOR)
#include "serial_bus_servo.h"
#endif


#define SAMPLE_TIME 8 // s
#define ONE_SEC_IN_US 1000000
#define ONE_SEC_IN_MS 1000

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

AccelPedal accel_pedal(ACCEL_SW_IN, 0, ACCEL_LEVEL_IN, MIN_ACCEL_IN, MAX_ACCEL_IN, ONE_SEC_IN_MS/20, 0.5);

INA226 pwr_mon_ctrl_bat;
INA226 pwr_mon_drive_bat;

HCSR04 dist_sensor0(0, HCSR04_TRIG1_OUT, HCSR04_ECHO1_IN, 3);
HCSR04 dist_sensor1(1, HCSR04_TRIG2_OUT, HCSR04_ECHO2_IN, 3);
HCSR04 *dist_sensors[] = {&dist_sensor0, &dist_sensor1};
const int NUM_DIST_SENSORS = sizeof(dist_sensors)/sizeof(HCSR04*);

#if defined(PANNING_DIST_SENSOR)
SerialServo servo0(Serial8);
#endif

Kinematics kinematics(
    Kinematics::ACKERMANN,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    FR_WHEELS_DISTANCE,
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
    Serial.println("");
    Serial.println("'r'    spin the rear motors one at a time.");
    Serial.println("'s'    run steering control test.");
    Serial.println("'e'    run external steering control test.");
    Serial.println("'a'    run accelerator pedal test.");
    Serial.println("'p'    run distance sensor test.");
    Serial.println("enter  print input status.");
    Serial.println("");

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
        else if (character == 'a')
        {
            Serial.println("\r\n");
            testAccelerator();
        }
        else if (character == 'p')
        {
            Serial.println("\r\n");
            testDistanceSensor();
        }
        else if (character == '\r')
        {
            // Read various inputs
            Serial.print("Estop: ");
            Serial.print(digitalRead(ESTOP_IN));
            Serial.print(", Ign sw: ");
            Serial.print(digitalRead(IGN_SW_IN));
            Serial.print(", Acc (sw, level): ");
            Serial.print(digitalRead(ACCEL_SW_IN));
            Serial.print(", ");
            Serial.print(analogRead(ACCEL_LEVEL_IN));
            Serial.print(", Fwrd/rev sw: ");
            Serial.print(digitalRead(FORW_REV_SW_IN));
            Serial.print(", DriveBat (V,I): ");
            Serial.print(pwr_mon_drive_bat.readBusVoltage());
            Serial.print(", ");
            Serial.print(pwr_mon_drive_bat.readShuntCurrent());
            Serial.print(", CtrlBat (V:I): ");
            Serial.print(pwr_mon_ctrl_bat.readBusVoltage());
            Serial.print(", ");
            Serial.print(pwr_mon_ctrl_bat.readShuntCurrent());
            Serial.print(", steering enc: ");
            Serial.print(str_motor_enc.read());
            Serial.print(", steering limit: ");
            Serial.print(digitalRead(STEER_RIGHT_LIMIT_IN));
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
            steering.update(true);
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
                int16_t delta = 0;
                if (character == 'l')
                {
                    delta = -4;
                }
                else if (character == 'r')
                {
                    delta = 4;
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
            steering.update(true);
        }
        if (!steering.enable_steering_wheel())
        {
            Serial.println("Error, failed to enable steering wheel control");
            return;
        }
    }

    Serial.println("Steering wheel control mode enabled.  Turn the steering wheel to change steering...");

    unsigned long last_status = micros();
    while (true)
    {
        steering.update(true);
        if (micros() - last_status >= ONE_SEC_IN_US) {
            last_status = micros();
            float pos = steering.get_position();
            Serial.println(pos);
        }
    }
}

bool directionChange(float cur_rpm, float req_rpm)
{
    return abs(cur_rpm) > 0.1 && sgn(cur_rpm) != sgn(req_rpm);
}

void testAccelerator()
{
    unsigned long last_update = 0;

    while (true) {
        accel_pedal.update();

        if ((long int)(millis() - last_update) >= 100)
        {
            float level = accel_pedal.get_level();

            int8_t dir = digitalRead(FORW_REV_SW_IN);
            if (!dir) {
                level *= -1;
            }
            float req_rpm = level * MOTOR_MAX_RPM;

            float current_rpm1 = motor1_encoder.getRPM();
            float current_rpm2 = motor2_encoder.getRPM();

            // Don't drive motor in the opposite direction until it stops
            if (directionChange(current_rpm1, req_rpm)) {
                req_rpm = 0.0;
            }
            motor1_controller.spin(motor1_pid.compute(req_rpm, current_rpm1));
            motor2_controller.spin(motor2_pid.compute(req_rpm, current_rpm2));        

            last_update = millis();

            Serial.print("Dir sw: ");
            Serial.print(dir);
            Serial.print(", accel level: ");
            Serial.print(level);
            Serial.print(", req_rpm: ");
            Serial.println(req_rpm);
        }
    }
}

#if defined(PANNING_DIST_SENSOR)
const int scan_pos_start = 100;
const int scan_pos_end = 500;
const int scan_pos_step = 100;
const int servo_id = 1;

void testDistanceSensor()
{
    servo0.move(servo_id, scan_pos_start, 3000);
    delay(500);

    int16_t position = scan_pos_start;
    int dir = -1;

    while (true) {
        if (dist_sensor0.start() != HCSR04::State::kRanging) {
            Serial.println("Failed to start dist ranging");
        }
        Serial.println("Started dist ranging...");

        bool wait = true;
        while (wait) {
            delay(20);
            float dist;
            auto state = dist_sensor0.get_distance_m(dist);
            switch (state) {
                case HCSR04::State::kRanging:
                    Serial.println("Ranging...");
                    break;
                case HCSR04::State::kEdgeHi:
                    Serial.println("Hi...");
                    break;
                case HCSR04::State::kError:
                    Serial.println("Error...");
                    return;
                case HCSR04::State::kTimeout:
                    Serial.println("Timeout...");
                    wait = false;
                    break;
                case HCSR04::State::kFinished:
                    Serial.print("Range: ");
                    Serial.println(dist);
                    wait = false;
                    break;
                default:
                    break;
            }
        }
        delay(500);

        if (position >= scan_pos_end ||
            position <= scan_pos_start) {
            dir *= -1;
        }
        position += scan_pos_step*dir;
        Serial.println(position);
        servo0.move(servo_id, position, 500);
        delay(1000);
    }
}
#endif

void testDistanceSensor()
{
    while (true) {
        for (int i = 0; i < NUM_DIST_SENSORS; i++) {
            HCSR04 *sensor = dist_sensors[i];
        
            if (sensor->start() != HCSR04::State::kRanging) {
                Serial.println("Failed to start dist ranging");
            }

            Serial.print("Started dist ranging, sensor: ");
            Serial.println(i);

            bool wait = true;
            while (wait) {
                delay(20);
                float dist;
                auto state = sensor->get_distance_m(dist);
                switch (state) {
                    case HCSR04::State::kRanging:
                        Serial.println("Ranging...");
                        break;
                    case HCSR04::State::kEdgeHi:
                        Serial.println("Hi...");
                        break;
                    case HCSR04::State::kError:
                        Serial.println("Error...");
                        return;
                    case HCSR04::State::kTimeout:
                        Serial.println("Timeout...");
                        wait = false;
                        break;
                    case HCSR04::State::kFinished:
                        Serial.print("Range: ");
                        Serial.println(dist);
                        wait = false;
                        break;
                    default:
                        break;
                }
            }
            delay(500);
        }
    }
}
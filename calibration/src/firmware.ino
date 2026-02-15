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
#include <float.h>
#include <cmath>

#include "config.h"
#include "motor.h"
#define ENCODER_USE_INTERRUPTS
#include "encoder_single_phase.h"
#include "encoder.h"
#include "kinematics.h"
#include "motor_speed_controller.h"
#include "linear_actuator.h"
#include "steering_using_linear_actuator.h"
#include "steering_angle_to_actuator_mapper_ebot_ackerman.h"
#include "HMC5883L.h"
#include "ADXL345.h"
#include "hc_sr04.h"
#include "serial_bus_servo.h"

#define SAMPLE_TIME     10 //s
#define ONE_SEC_IN_US   1000000
#define ONE_SEC_IN_MS   1000

//////////////////////////////////
// Wheel related
//////////////////////////////////

// Motors

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

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
MotorSpeedController motor1_speed_controller(motor1_controller, motor1_encoder, motor1_pid);

PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
MotorSpeedController motor2_speed_controller(motor2_controller, motor2_encoder, motor2_pid);

#if NUM_BASE_MOTORS == 4
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
MotorSpeedController motor3_speed_controller(motor3_controller, motor3_encoder, motor3_pid);

PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
MotorSpeedController motor4_speed_controller(motor4_controller, motor4_encoder, motor4_pid);
#endif

//////////////////////////////////
// Steering - see comments in lino_base_config file.
//////////////////////////////////

// Motor
Motor motor_str_controller(PWM_FREQUENCY, PWM_BITS, MOTOR_STR_INV, MOTOR_STR_PWM, MOTOR_STR_IN_A, MOTOR_STR_IN_B, -1);

// Motor/shaft encoder
EncoderQuadrature str_motor_enc(STEERMTR_ENCODER_A, STEERMTR_ENCODER_B, STR_MOTOR_ENC_TICKS_PER_REV, MOTOR_STR_ENCODER_INV);
EncoderNull str_wheel_enc;

// Motor speed controller
PID motor_spd_pid(STR_SPD_PWM_MIN, STR_SPD_PWM_MAX, STR_SPD_PID_P, STR_SPD_PID_I, STR_SPD_PID_D);
MotorSpeedController str_motor_speed_controller(motor_str_controller, str_motor_enc, motor_spd_pid);

// Steering actuator
PID str_act_pid(STR_ACT_RPM_MIN, STR_ACT_RPM_MAX, STR_ACT_PID_P, STR_ACT_PID_I, STR_ACT_PID_D);
LinearActuator steering_actuator(LinearActuator::HomeDetection::kSwitch, STR_LEFT_LIMIT_IN,
                                 str_motor_speed_controller, str_motor_enc, str_act_pid, STR_ACT_HOMING_RPM,
                                  STR_ACT_MAX_POS, STR_ACT_POS_THRESH);

SteeringAngleToActuatorMapperEbotAckerman steering_angle_to_lin_actuator_mapper;

SteeringUsingLinearActuator steering_using_linear_act(steering_actuator, steering_angle_to_lin_actuator_mapper);

Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    FR_WHEELS_DISTANCE,
    LR_WHEELS_DISTANCE);

Motor *motors[] = {&motor1_controller, &motor2_controller
#if NUM_BASE_MOTORS == 4
                    ,
                    &motor3_controller, &motor4_controller
#endif 
                   };

EncoderSinglePhase *encoders[] = {&motor1_encoder, &motor2_encoder
#if NUM_BASE_MOTORS == 4
                            ,
                            &motor3_encoder, &motor4_encoder
#endif
                           };
String labels[4] = {"FRONT LEFT - M1: ", "FRONT RIGHT - M2: "
#if NUM_BASE_MOTORS == 4
                    ,
                    "REAR LEFT - M3: ", "REAR RIGHT - M4: "
#endif                    
                    };
int total_motors = sizeof(motors);
long long int counts_per_rev[sizeof(motors)];

const int DIST_SENSOR_UPDATE_PERIOD_MS = 100;
HCSR04 dist_sensor_front(0, HCSR04_TRIG_FRONT_OUT, HCSR04_ECHO_FRONT_IN, 6);
HCSR04 dist_sensor_back(1, HCSR04_TRIG_BACK_OUT, HCSR04_ECHO_BACK_IN, 6);
HCSR04 *dist_sensors[] = {&dist_sensor_front, &dist_sensor_back};
const int NUM_DIST_SENSORS = sizeof(dist_sensors)/sizeof(HCSR04*);

SerialServo servo_serial_front(Serial8, 1, 240, 1000, true);
SerialServo servo_serial_back(Serial8, 2, 240, 1000, true);

void printHelp()
{
    Serial.println("Sampling process will spin the motors at its maximum RPM.");
    Serial.println("Please ensure that the robot is ELEVATED and there are NO OBSTRUCTIONS to the wheels.");
    Serial.println("");
    Serial.println("'s' spin the motors.");
    Serial.println("'c' spin the motors with motor summary.");
    Serial.println("'m' show heading using magnetometer.");
    Serial.println("'a' test accelerometer.");
    Serial.println("'d' test ultrasonic distance sensor.");
    Serial.println("'v' set serial servo id.");
    Serial.println("'e' set serial servo position.");
    Serial.println("'1' output magnetometer in RAW and UNI format for calibration.");
    Serial.println("'2' perform hard-iron magnetometer calibration.");
    Serial.println("'5' Encoder test.");
    Serial.println("'6' motor speed controller test.");
    Serial.println("'7' steering linear actuator test.");
    Serial.println("'8' steering actuator mapper test.");
    Serial.println("'9' steering controller test (using linear actuator).");
    Serial.println("");
}    

void setup()
{
    Serial.begin(9600);
    while (!Serial)
    {
    }

    printHelp();

    pinMode(MOTOR_RELAY_PWR_OUT, OUTPUT);
    digitalWrite(MOTOR_RELAY_PWR_OUT, HIGH);
    //pinMode(STR_LEFT_LIMIT_IN, INPUT_PULLUP);
}


void sampleMotors(bool show_summary)
{
    if (Kinematics::LINO_BASE == Kinematics::DIFFERENTIAL_DRIVE ||
        Kinematics::LINO_BASE == Kinematics::ACKERMANN)
    {
        total_motors = 2;
    }

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
                Serial.print("M0 Enc cnt: " );
                Serial.print(encoders[0]->read());
                Serial.print(", rpm: " );
                Serial.print(encoders[0]->getRPM());
                Serial.print(", M1 Enc cnt: " );
                Serial.print(encoders[1]->read());
                Serial.print(", rpm: " );
                Serial.print(encoders[1]->getRPM());
    #if NUM_MOTORS == 4
                Serial.print(", M3 Enc cnt: " );
                Serial.print(encoders[2]->read());
                Serial.print(", rpm: " );
                Serial.print(encoders[2]->getRPM());
                Serial.print(", M4 Enc cnt: " );
                Serial.print(encoders[3]->read());
                Serial.print(", rpm: " );
                Serial.print(encoders[3]->getRPM());
    #endif            
                Serial.println("");

            }

            motors[i]->spin(200);

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
    Serial.print(" ");

#if NUM_MOTORS == 4
    Serial.print(labels[2]);
    Serial.print(encoders[2]->read());
    Serial.print(" ");

    Serial.print(labels[3]);
    Serial.println(encoders[3]->read());
    Serial.println("");
#endif
    Serial.println("================COUNTS PER REVOLUTION=================");
    Serial.print(labels[0]);
    Serial.print(counts_per_rev[0]);
    Serial.print(" ");

    Serial.print(labels[1]);
    Serial.println(counts_per_rev[1]);
    Serial.print(" ");
    
#if NUM_MOTORS == 4
    Serial.print(labels[2]);
    Serial.print(counts_per_rev[2]);
    Serial.print(" ");

    Serial.print(labels[3]);
    Serial.println(counts_per_rev[3]);
    Serial.println("");
#endif    

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

void accelerometerTest()
{
    const float G_TO_ACCEL_ = 9.81;
    const double ACCEL_SCALE = 1 / 256.0;

    ADXL345 accel;

    Wire.begin();

    while(!accel.testConnection())
    {
        Serial.println("Accelerometer not detected");
        delay(1000);
    }

    accel.initialize();

    uint8_t format = accel.getDataFormat();
    Serial.print("Data format reg: ");
    Serial.println(format, HEX);

//  accel.setFullResolution(1);
//  accel.setRange(3);
    accel.setAutoSleepEnabled(false);

    format = accel.getDataFormat();
    Serial.print("Data format reg: ");
    Serial.println(format, HEX);

    int16_t x, y, z = 0;
    while(true)
    {
        accel.getAcceleration(&x, &y, &z);

        float ax = x * ACCEL_SCALE * G_TO_ACCEL_;
        float ay = y * ACCEL_SCALE * G_TO_ACCEL_;
        float az = z * ACCEL_SCALE * G_TO_ACCEL_;

        Serial.print("Accel (g): ");
        Serial.print(ax, 6);
        Serial.print(", ");
        Serial.print(ay, 6);
        Serial.print(", ");
        Serial.print(az, 6);
        Serial.print("Raw: ");
        Serial.print(x);
        Serial.print(", ");
        Serial.print(y);
        Serial.print(", ");
        Serial.println(z);
        delay(1000);
    }
}

const int16_t HMC5883L_INVALID_RAW_GAUSS = -4096;
const float HMC5883L_GAIN_1370_SCALE = 0.73;
const float MILLI_GAUSS_PER_TELSA = 10000000.0;
const float MILLI_GAUSS_PER_U_TELSA_X10 = 1.0;

void magnetometerInit(HMC5883L &mag)
{
    Wire.begin();

    while(!mag.testConnection())
    {
        Serial.println("Magnetometer not detected");
        delay(1000);
    }

    mag.initialize();
    mag.setMode(HMC5883L_MODE_CONTINUOUS);
    mag.setDataRate(HMC5883L_RATE_15);
    mag.setGain(HMC5883L_GAIN_1370);
}

bool magnetometerRead(HMC5883L &mag, float (&mag_data)[3])
{
    int16_t x, y, z = 0;
    mag.getHeading(&x, &y, &z);
    if (x != HMC5883L_INVALID_RAW_GAUSS &&
        y != HMC5883L_INVALID_RAW_GAUSS &&
        z != HMC5883L_INVALID_RAW_GAUSS)
    {
        // units: uT x 10
        mag_data[0] = x*HMC5883L_GAIN_1370_SCALE;
        mag_data[1] = y*HMC5883L_GAIN_1370_SCALE;
        mag_data[2] = z*HMC5883L_GAIN_1370_SCALE;
        return true;
    }
    return false;
}

void magnetometerOutputDataForCal()
{
    HMC5883L mag;
    magnetometerInit(mag);

    float mag_data[3];

    while(true)
    {
        if (magnetometerRead(mag, mag_data)) {
            Serial.print("Raw:0,0,0,0,0,0,");
            Serial.print((int)(mag_data[0]));
            Serial.print(",");
            Serial.print((int)(mag_data[1]));
            Serial.print(",");
            Serial.println((int)(mag_data[2]));

            // In uT units x10
            Serial.print("Uni:0,0,0,0,0,0,");
            Serial.print(mag_data[0]/10.0, 6);
            Serial.print(", ");
            Serial.print(mag_data[1]/10.0, 6);
            Serial.print(", ");
            Serial.println(mag_data[2]/10.0, 6);
        }
        delay(10);
    }
}

#undef USE_PREV_OFFSET

void magnetometerHardIronCal()
{
    HMC5883L mag;
    magnetometerInit(mag);

    float mag_data[3];
    float mag_min[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
    float mag_max[3] = {FLT_MIN, FLT_MIN, FLT_MIN};
#if defined(USE_PREV_OFFSET)
    float mag_ofst[3] = {-93.07, -122.27, 44.165};
#else
    float mag_ofst[3] = {0, 0, 0};
#endif

    while(true)
    {
        if (magnetometerRead(mag, mag_data)) {

            for (int i = 0; i < 3; i++) {
                if (mag_data[i] < mag_min[i]) {
                   mag_min[i] = mag_data[i];
                } else if (mag_data[i] > mag_max[i]) {
                    mag_max[i] = mag_data[i];
                }

#if !defined(USE_PREV_OFFSET)
                mag_ofst[i] = (mag_min[i] + mag_max[i])/2;
#endif                
            }

            for (int i = 0; i < 3; i++) {
                Serial.print(mag_data[i], 1);
                Serial.print(", ");
            }

            Serial.print("   min/max: ");
            for (int i = 0; i < 3; i++) {
                Serial.print(mag_min[i], 1);
                Serial.print(", ");
                Serial.print(mag_max[i], 1);
                Serial.print(", ");
            }

            Serial.print("   offset: ");
            for (int i = 0; i < 3; i++) {
                Serial.print(mag_ofst[i], 1);
                Serial.print(", ");
            }

            Serial.print("   mag-offset: ");
            for (int i = 0; i < 3; i++) {
                Serial.print(mag_data[i] - mag_ofst[i], 1);
                Serial.print(", ");
            }

            Serial.print("  heading: ");
            Serial.println(atan2(mag_data[0] - mag_ofst[0], mag_data[1] - mag_ofst[1])*180.0/M_PI);
        }
        delay(10);
    }
}

// This method assumes calibration has been applied by the IMU lib
void magnetometerShowHeading()
{
    HMC5883L mag;
    magnetometerInit(mag);

    float mag_data[3];

    while(true)
    {
        if (magnetometerRead(mag, mag_data)) {

            float heading = atan2(mag_data[0], mag_data[1]);

            Serial.print("Heading (deg): ");
            Serial.print(heading*180.0/M_PI);
            Serial.print(",    Mag(mGs): ");
            Serial.print(mag_data[0], 6);
            Serial.print(", ");
            Serial.print(mag_data[1], 6);
            Serial.print(", ");
            Serial.print(mag_data[2], 6);
            Serial.print(",    Mag(T): ");
            Serial.print(mag_data[0]/MILLI_GAUSS_PER_TELSA, 9);
            Serial.print(", ");
            Serial.print(mag_data[1]/MILLI_GAUSS_PER_TELSA, 9);
            Serial.print(", ");
            Serial.println(mag_data[2]/MILLI_GAUSS_PER_TELSA, 9);
        }
        delay(10);
    }
}

void encoderTest(Motor &motor, EncoderInterface &encoder)
{
    int spd = 0;
    motor.spin(spd);

    unsigned long start_time = micros();

    bool run = true;
    while (run)
    {
        if (Serial.available())
        {
            char c = Serial.read();
            Serial.print(c);
            delay(1);

            switch (c)
            {
                case '+':
                {
                    spd += 20;
                    motor.spin(spd);
                    break;
                }
                case '-':
                {
                    spd -= 20;
                    motor.spin(spd);
                    break;
                }
                case 's':
                {
                    spd = 0;
                    motor.spin(spd);
                    break;
                }
                case 'e':
                {
                    run = false;
                    break;
                }
                default:
                {
                    break;
                }
            }
        }

        auto now = micros();
        if (now - start_time >= 500000)
        {
            start_time = now;
            Serial.print("spd ");
            Serial.print(spd);
            Serial.print("   RPM ");
            Serial.print(encoder.getRPM());
            Serial.print("   Ticks ");
            Serial.print(encoder.read());
            Serial.println("\r\n");
        }
    }
}

void motorSpeedControlTest(MotorSpeedController &controller1, MotorSpeedController &controller2)
{
    const int num_ctrls = 2;

    struct Controllers {
        Controllers(MotorSpeedController &controller):
             controller(controller) {
                controller.set_target_rpm(0);
             }
        int rpm{0};
        MotorSpeedController &controller;
    } controllers[num_ctrls] = {controller1, controller2};

    unsigned long start_time = micros();

    int selected = 0;

    bool run = true;
    while (run)
    {
        Controllers &sel_controller = controllers[selected];
        int new_rpm = sel_controller.rpm;

        PID &sel_pid = sel_controller.controller.get_pid();

        if (Serial.available())
        {
            char c = Serial.read();
            Serial.print(c);
            delay(1);

            switch (c)
            {
                case '0':
                {
                    selected = 0;
                    continue;
                }
                case '1':
                {
                    selected = 1;
                    continue;
                }
                case '+':
                {
                    if (abs(new_rpm) < 10) {
                        new_rpm++;
                    } else {
                        new_rpm += 10;
                    }
                    break;
                }
                case '-':
                {
                    if (abs(new_rpm) <= 10) {
                        new_rpm--;
                    } else {
                        new_rpm -= 10;
                    }
                    break;
                }
                case 'p':
                case 'P':
                {
                    auto kp = sel_pid.get_kp();
                    kp += (c == 'p'? -0.1: 0.1);
                    sel_pid.updateKp(kp);
                    Serial.print("Set Kp ");
                    Serial.println(kp);
                    break;
                }
                case 'i':
                case 'I':
                {
                    auto ki = sel_pid.get_ki();
                    ki += (c == 'i'? -0.1: 0.1);
                    sel_pid.updateKi(ki);
                    Serial.print("Set Ki ");
                    Serial.println(ki);
                    break;
                }
                case 'd':
                case 'D':
                {
                    auto kd = sel_pid.get_kd();
                    kd += (c == 'd'? -0.1: 0.1);
                    sel_pid.updateKd(kd);
                    Serial.print("Set Kd ");
                    Serial.println(kd);
                    break;
                }
                case 's':
                {
                    new_rpm = 0;
                    break;
                }
                case 'e':
                {
                    run = false;
                    break;
                }
                default:
                {
                    break;
                }
            }
        }

        if (new_rpm != sel_controller.rpm) {
            sel_controller.rpm = new_rpm;
            sel_controller.controller.set_target_rpm(new_rpm);
        }

        bool log = false;
        auto now = micros();
        if (now - start_time >= 500000) {
            start_time = now;
            log = true;
            Serial.print("SPD Controller test: ");
        }


        for (int i = 0; i < num_ctrls; i++) {
            controllers[i].controller.update();

            if (log) {
                Serial.print("Ctrl: ");
                Serial.print(i);
                Serial.print("  target RPM: ");
                Serial.print(controllers[i].rpm);
                Serial.print("  actual RPM: ");
                Serial.print(controllers[i].controller.get_current_rpm());
                Serial.print("   |   ");
            }
        }

        if (log) {
            Serial.print("Pid(sel), e: ");
            Serial.print(sel_pid.getError());
            Serial.print(", ei: ");
            Serial.print(sel_pid.getIntegral());
            Serial.print(", ed: ");
            Serial.print(sel_pid.getDerivative());
            Serial.print(", or: ");
            Serial.print(sel_pid.getOutputRaw());
            Serial.print(", oc: ");
            Serial.print(sel_pid.getOutputConstrained());

            Serial.println("");
        }
    }
}

void steeringActuatorTest(LinearActuator &actuator, int max_pos, EncoderInterface &encoder)
{
    const int invalid_pos = -1;
    const float invalid_angle = 100.0f;

    actuator.home();
    
    unsigned long start_time = micros();

    while (true)
    {
        bool control = actuator.get_state() == LinearActuator::State::kControl;

        if (Serial.available())
        {
            char c = Serial.read();
            Serial.print(c);
            delay(1);

            if (control) {
                int new_target_pos = invalid_pos;
                float new_target_angle = invalid_angle;
                switch (c)
                {
                    case '0':
                    {
                        new_target_pos = 0;
                        break;
                    }
                    case '1':
                    {
                        new_target_pos = max_pos/5;
                        break;
                    }
                    case '2':
                    {
                        new_target_pos = max_pos*2/5;
                        break;
                    }
                    case '3':
                    {
                        new_target_pos = max_pos*3/5;
                        break;
                    }
                    case '4':
                    {
                        new_target_pos = max_pos*4/5;
                        break;
                    }
                    case '5':
                    {
                        new_target_pos = max_pos*5/5;
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
                if (new_target_pos != invalid_pos) {
                    Serial.print("STR ACT TEST: New target position: ");
                    Serial.print(new_target_pos);
                    Serial.println("\r\n");
                    actuator.set_target_position(new_target_pos);
                } else if (new_target_angle != invalid_angle) {
                    Serial.print("STR ACT TEST: New target angle: ");
                    Serial.print(new_target_angle);
                    Serial.println("\r\n");
                    actuator.set_target_position(new_target_angle);
                }
            }

            if (c == 'h') {
                actuator.home();
                Serial.println("Re-homing\r\n");
            } else if (c == 'e') {
                actuator.disable();
                Serial.println("Ending test\r\n");
                break;
            }
        }

        actuator.update();

        auto now = micros();
        if (now - start_time >= 500000)
        {
            start_time = now;
            Serial.print("STR ACT TEST: homing: ");
            Serial.print(!control);
            Serial.print(",   Position: ");
            Serial.print(encoder.read());
            Serial.println("\r\n");
        }
    }
}

void steeringActuatorMapperTest()
{
    SteeringAngleToActuatorMapperEbotAckerman::AngleToCalcValues debug;
    for (double angle = -40.0; angle <= 40.0; angle += 0.5) {
        auto asetting = steering_angle_to_lin_actuator_mapper.angle_to_actuator_setting(angle*M_PI/180.0, &debug);
        Serial.print("Input angle: ");
        Serial.print(angle);
        if (angle < 0.0)
        {
            Serial.print(" (turning right) ");
        }
        else
        {
            Serial.print(" (turning left) ");
        }
        Serial.print(", RW Angle: ");
        Serial.print(debug.rw_angle*180.0/M_PI);
        Serial.print(", TR end_pos mm: ");
        Serial.print(debug.tie_rod_act_end_pos_mm);
        Serial.print(", Actuator pos mm: ");
        Serial.print(debug.act_pos_mm);
        Serial.print(", Actuator setting (enc ticks), slow: ");
        Serial.print(asetting);

        auto asetting_fast = steering_angle_to_lin_actuator_mapper.angle_to_actuator_setting_fast(angle*M_PI/180.0);
        Serial.print(", fast: ");
        Serial.print(asetting_fast);
        Serial.print(", Angle from actuator setting: ");
        auto inv_angle = steering_angle_to_lin_actuator_mapper.actuator_setting_to_angle_fast(asetting);
        Serial.print(inv_angle*180.0/M_PI);

        Serial.println("\r\n");
    }
}

void steeringByLinearActuatorTest(SteeringUsingLinearActuator &steering)
{
    const float invalid_angle = 100.0f;

    steering.home();
    
    unsigned long start_time = micros();

    while (true)
    {
        bool control = steering.get_state() == SteeringUsingLinearActuator::State::kControl;

        if (Serial.available())
        {
            char c = Serial.read();
            Serial.print(c);
            delay(1);

            if (control) {
                float new_target_angle = invalid_angle;
                switch (c)
                {
                    case '0':
                    {
                        new_target_angle = -27.0;
                        break;
                    }
                    case '1':
                    {
                        new_target_angle = -10.0;
                        break;
                    }
                    case '2':
                    {
                        new_target_angle = 0.0;
                        break;
                    }
                    case '3':
                    {
                        new_target_angle = 10.0;
                        break;
                    }
                    case '4':
                    {
                        new_target_angle = 20.0;
                        break;
                    }
                    case '5':
                    {
                        new_target_angle = 35.0;
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
                if (new_target_angle != invalid_angle) {
                    Serial.print("STR ACT TEST: New target angle: ");
                    Serial.print(new_target_angle);
                    Serial.println("\r\n");
                    steering.set_angle(new_target_angle*M_PI/180.0);
                }
            }

            if (c == 'h') {
                steering.home();
                Serial.println("Re-homing\r\n");
            } else if (c == 'e') {
                steering.disable();
                Serial.println("Ending test\r\n");
                break;
            }
        }

        steering.update();

        auto now = micros();
        if (now - start_time >= 500000)
        {
            start_time = now;
            Serial.print("STR TEST: homing: ");
            Serial.print(!control);
            Serial.print(",   Position (angle): ");
            Serial.print(steering.get_current_angle()*180.0/M_PI);
            Serial.println("\r\n");
        }
    }
}

void testDistanceSensor()
{
    int sensor_idx = 0;
    bool measuring = false;
    HCSR04 *sensor = nullptr;

    while (true) {
        if (!measuring) {
            sensor = dist_sensors[sensor_idx];
            sensor->start();
            measuring = true;

            Serial.print("Started dist ranging, sensor: ");
            Serial.println(sensor_idx);

        } else {            
            float dist;
            if (sensor->get_distance_m(dist))
            {
                Serial.print("Range: ");
                Serial.println(dist);

            }
            measuring = !sensor->finished();

            if (!measuring) {
                sensor_idx++;
                sensor_idx = sensor_idx % NUM_DIST_SENSORS;

                if (sensor_idx == 0) {
                    delay(1000);
                }
            }
        }
        delay(10);
    }
}

void testSetServoId()
{
    servo_serial_front.set_id(2);
}

void testSetServoAngle(SerialServo &servo_serial)
{
    const int16_t positions[] = {0, 500, 1000};
    int pos_idx = 0;

    const float positions_degrees[] = {0, 60, 120, 180, 240};
    int pos_idx_deg = 0;

    int16_t cur_pos = -1;
    int16_t new_pos = -1;

    while (true)
    {
        while (Serial.available())
        {
            char c = Serial.read();
            switch (c)
            {
                case 'e':
                {
                    return;
                }
                case 'm':
                {
                    if (++pos_idx > sizeof(positions)/sizeof(int16_t) - 1) {
                        pos_idx = 0;
                    }
                    new_pos = positions[pos_idx];
                    break;
                }
                case 'd':
                {
                    if (++pos_idx_deg > sizeof(positions_degrees)/sizeof(float) - 1) {
                        pos_idx_deg = 0;
                    }

                    servo_serial.move(positions_degrees[pos_idx_deg], 200);
                    Serial.print("Set servo position to (degrees) ");
                    Serial.println(positions_degrees[pos_idx_deg]);
                    break;
                }
                case '+':
                {
                    new_pos = cur_pos + 1;
                    break;
                }
                case '-':
                {
                    new_pos = cur_pos - 1;
                    break;
                }
                default:
                    break;
            }

            if (new_pos != cur_pos)
            {
                cur_pos = new_pos;
                servo_serial.move(cur_pos, 200);
                Serial.print("Set servo position to ");
                Serial.println(cur_pos);
            }
        }
    }
}


void loop()
{
    while (Serial.available())
    {
        char c = Serial.read();
        Serial.print(c);
        delay(1);
        Serial.println("\r\n");

        switch(c)
        {
            case 'h':
            default:
            {
                printHelp();
                break;
            }            
            case 's':
            {
                sampleMotors(0);
                break;
            }
            case 'c':
            {
                sampleMotors(1);
                break;
            }
            case 'm':
            {
                magnetometerShowHeading();
                break;
            }
            case 'a':
            {
                accelerometerTest();
                break;
            }
            case 'd':
            {
                testDistanceSensor();
                break;
            }
            case 'v':
            {
                testSetServoId();
                break;
            }
            case 'e':
            {
                testSetServoAngle(servo_serial_back);
                break;
            }
            case '1':
            {
                magnetometerOutputDataForCal();
                break;
            }
            case '2':
            {
                magnetometerHardIronCal();
                break;
            }
            case '5':
            {
                encoderTest(motor_str_controller, str_motor_enc);
                break;
            }
            case '6':
            {
                motorSpeedControlTest(motor1_speed_controller, motor2_speed_controller);
                break;
            }
            case '7':
            {
                steeringActuatorTest(steering_actuator, STR_ACT_MAX_POS, str_motor_enc);
                break;
            }
            case '8':
            {
                steeringActuatorMapperTest();
                break;
            }
            case '9':
            {
                steeringByLinearActuatorTest(steering_using_linear_act);
                break;
            }
            case '\r':
            {
                // Read various inputs
                Serial.print("Steering enc: ");
                Serial.print(str_motor_enc.read());
                break;
            }
        }
    }
}

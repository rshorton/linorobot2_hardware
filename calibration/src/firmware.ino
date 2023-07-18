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
#include "config.h"
#include "motor.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#if defined(NO_ENCODER)
#include "encoder_none.h"
#else
#include "encoder.h"
#endif
#include "kinematics.h"
#include "HMC5883L.h"
#include "ADXL345.h"

#define SAMPLE_TIME 10 //s

int m1_dir_status = 0;
int m2_dir_status = 0;
int m3_dir_status = 0;
int m4_dir_status = 0;

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV, &m1_dir_status);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV, &m2_dir_status);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV, &m3_dir_status);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV, &m4_dir_status);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B, -1, &m1_dir_status);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B, -1, &m2_dir_status);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B, -1, &m3_dir_status);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B, -1, &m4_dir_status);

Kinematics kinematics(
    Kinematics::LINO_BASE, 
    MOTOR_MAX_RPM, 
    MAX_RPM_RATIO, 
    MOTOR_OPERATING_VOLTAGE, 
    MOTOR_POWER_MAX_VOLTAGE, 
    WHEEL_DIAMETER, 
    LR_WHEELS_DISTANCE
);

long long int counts_per_rev[4];
int total_motors = 4;
Motor *motors[4] = {&motor1_controller, &motor2_controller, &motor3_controller, &motor4_controller};
Encoder *encoders[4] = {&motor1_encoder, &motor2_encoder, &motor3_encoder, &motor4_encoder};
String labels[4] = {"FRONT LEFT - M1: ", "FRONT RIGHT - M2: ", "REAR LEFT - M3: ", "REAR RIGHT - M4: "};

void setup()
{
    Serial.begin(9600);
    while (!Serial) {
    }
    Serial.println("Sampling process will spin the motors at its maximum RPM.");
    Serial.println("Please ensure that the robot is ELEVATED and there are NO OBSTRUCTIONS to the wheels.");
    Serial.println("");
    Serial.println("Type 'spin' or 's' and press enter to spin the motors.");
    Serial.println("Type 'sample' or 'c' and press enter to spin the motors with motor summary.");
    Serial.println("Press enter to clear command.");
    Serial.println("");

    pinMode(MOTOR_RELAY_PWR_OUT, OUTPUT);
    digitalWrite(MOTOR_RELAY_PWR_OUT, HIGH);

}

void loop()
{
    static String cmd = "";

    while (Serial.available())
    {
        char character = Serial.read(); 
        cmd.concat(character); 
        Serial.print(character);
        delay(1);
        if(character == '\r' and (cmd.equals("spin\r") || cmd.equals("s\r")))
        {
            cmd = "";
            Serial.println("\r\n");
            sampleMotors(0);
        }
        else if(character == '\r' and (cmd.equals("sample\r") || cmd.equals("c\r")))
        {
            cmd = "";
            Serial.println("\r\n");
            sampleMotors(1);
        }
        else if(character == '\r' and cmd.equals("m\r"))
        {
            cmd = "";
            Serial.println("\r\n");
            magnetometerTest();
        }
        else if(character == '\r' and cmd.equals("a\r"))
        {
            cmd = "";
            Serial.println("\r\n");
            accelerometerTest();
        }
        else if(character == '\r')
        {
            Serial.println("");
            cmd = "";
        }
    }
}

void sampleMotors(bool show_summary)
{
    if(Kinematics::LINO_BASE == Kinematics::DIFFERENTIAL_DRIVE)
    {
        total_motors = 2;
    }

    float measured_voltage = constrain(MOTOR_POWER_MEASURED_VOLTAGE, 0, MOTOR_OPERATING_VOLTAGE);
    float scaled_max_rpm = ((measured_voltage / MOTOR_OPERATING_VOLTAGE) * MOTOR_MAX_RPM);
    float total_rev = scaled_max_rpm * (SAMPLE_TIME / 60.0);


    for(int i=0; i<total_motors; i++)
    {
        encoders[i]->write(0);
    }        

    for(int i=0; i<total_motors; i++)
    {
        Serial.print("SPINNING ");
        Serial.print(labels[i]);

        unsigned long start_time = micros();
        unsigned long last_status = micros();

        encoders[i]->write(0);
        while(true)
        {
            if(micros() - start_time >= SAMPLE_TIME * 1000000)
            {
                motors[i]->spin(0);
                Serial.println("");
                break;
            }

            if(micros() - last_status >= 1000000)
            {
                last_status = micros();
                Serial.print(".");
            }

            motors[i]->spin(200);

            Serial.print(encoders[0]->read());
            Serial.print(" ");
            Serial.print(encoders[1]->read());
            Serial.print(" ");
            Serial.print(encoders[2]->read());
            Serial.print(" ");
            Serial.print(encoders[3]->read());
            Serial.println("\r\n");
        }
        
        counts_per_rev[i] = encoders[i]->read() / total_rev;
    }
    if(show_summary)
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

    Serial.print(labels[2]);
    Serial.print(encoders[2]->read());
    Serial.print(" ");

    Serial.print(labels[3]);
    Serial.println(encoders[3]->read());
    Serial.println("");

    Serial.println("================COUNTS PER REVOLUTION=================");
    Serial.print(labels[0]);
    Serial.print(counts_per_rev[0]);
    Serial.print(" ");

    Serial.print(labels[1]);
    Serial.println(counts_per_rev[1]);
    
    Serial.print(labels[2]);
    Serial.print(counts_per_rev[2]);
    Serial.print(" ");

    Serial.print(labels[3]);
    Serial.println(counts_per_rev[3]);
    Serial.println("");

    Serial.println("====================MAX VELOCITIES====================");
    float max_rpm = kinematics.getMaxRPM();
    
    Kinematics::velocities max_linear = kinematics.getVelocities(max_rpm, max_rpm, max_rpm, max_rpm);
    Kinematics::velocities max_angular = kinematics.getVelocities(-max_rpm, max_rpm,-max_rpm, max_rpm);

    Serial.print("Linear Velocity: +- ");
    Serial.print(max_linear.linear_x);
    Serial.println(" m/s");

    Serial.print("Angular Velocity: +- ");
    Serial.print(max_angular.angular_z);
    Serial.println(" rad/s");
}

const int16_t HMC5883L_INVALID_RAW_GAUSS = -4096;
const float HMC5883L_GAIN_1370_SCALE = 0.73;
const float MILLI_GAUSS_PER_TELSA = 10000000.0;

void magnetometerTest()
{
    HMC5883L mag;

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


    int16_t x, y, z = 0;
    while(true)
    {
        mag.getHeading(&x, &y, &z);
        if (x == HMC5883L_INVALID_RAW_GAUSS ||
            y == HMC5883L_INVALID_RAW_GAUSS ||
            z == HMC5883L_INVALID_RAW_GAUSS)
        {
            Serial.println("Invalid heading");
        }
        else
        {
            float xg = x*HMC5883L_GAIN_1370_SCALE;
            float yg = y*HMC5883L_GAIN_1370_SCALE;
            float zg = z*HMC5883L_GAIN_1370_SCALE;


            float heading = atan2(y, x);
            if (heading < 0)
            {
                heading += 2*M_PI;
            }

            Serial.print("Heading (deg): ");
            Serial.print(heading*180.0/M_PI);
            Serial.print(",    Mag(mGs): ");
            Serial.print(xg, 6);
            Serial.print(", ");
            Serial.print(yg, 6);
            Serial.print(", ");
            Serial.print(zg, 6);
            Serial.print(",    Mag(T): ");
            Serial.print(xg/MILLI_GAUSS_PER_TELSA, 9);
            Serial.print(", ");
            Serial.print(yg/MILLI_GAUSS_PER_TELSA, 9);
            Serial.print(", ");
            Serial.println(zg/MILLI_GAUSS_PER_TELSA, 9);
        }
        delay(1000);
    }
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

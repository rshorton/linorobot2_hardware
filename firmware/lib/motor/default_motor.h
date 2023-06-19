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

// Revised for ElsaBot Jeep by S. Horton.

#ifndef DEFAULT_MOTOR
#define DEFAULT_MOTOR

#include <Arduino.h>
#include <Servo.h> 
#include <assert.h>
#include "RoboClaw.h"

#include "motor_interface.h"

class Generic2: public MotorInterface
{
    private:
        int in_a_pin_;
        int in_b_pin_;
        int pwm_pin_;

    protected:
        void forward(int pwm) override
        {
            digitalWrite(in_a_pin_, HIGH);
            digitalWrite(in_b_pin_, LOW);
            analogWrite(pwm_pin_, abs(pwm));
        }

        void reverse(int pwm) override
        {
            digitalWrite(in_a_pin_, LOW);
            digitalWrite(in_b_pin_, HIGH);
            analogWrite(pwm_pin_, abs(pwm));
        }

    public:
        Generic2(float pwm_frequency, int pwm_bits, bool invert, int pwm_pin, int in_a_pin, int in_b_pin): 
            MotorInterface(invert),
            in_a_pin_(in_a_pin),
            in_b_pin_(in_b_pin),
            pwm_pin_(pwm_pin)
        {
            pinMode(in_a_pin_, OUTPUT);
            pinMode(in_b_pin_, OUTPUT);
            pinMode(pwm_pin_, OUTPUT);

            if(pwm_frequency > 0)
            {
                analogWriteFrequency(pwm_pin_, pwm_frequency);
            }
            analogWriteResolution(pwm_bits);

            //ensure that the motor is in neutral state during bootup
            analogWrite(pwm_pin_, abs(0));
        }

        void brake() override
        {
            analogWrite(pwm_pin_, 0);
        }
};

class Generic1: public MotorInterface
{
    private:
        int in_pin_;
        int pwm_pin_;
        int current_pin_;
        float voltage_ref;
        int last_pwm;
        float current_ave;
        int *dir_status_out_;

    protected:
        void forward(int pwm) override
        {
            digitalWrite(in_pin_, HIGH);
            analogWrite(pwm_pin_, abs(pwm));
            last_pwm = pwm;
            if (dir_status_out_) {
                *dir_status_out_ = invert_? 0: 1;
            }                
        }

        void reverse(int pwm) override
        {
            digitalWrite(in_pin_, LOW);
            analogWrite(pwm_pin_, abs(pwm));
            last_pwm = pwm;
            if (dir_status_out_) {
                *dir_status_out_ = invert_? 1: 0;
            }                
        }

    public:
        Generic1(float pwm_frequency, int pwm_bits, bool invert, int pwm_pin, int in_pin, int unused=-1, int current_pin= -1, int *dir_status_out = NULL): 
            MotorInterface(invert),
            in_pin_(in_pin),
            pwm_pin_(pwm_pin),
            current_pin_(current_pin),
            voltage_ref(2.5),
            last_pwm(0),
            current_ave(0.0),
            dir_status_out_(dir_status_out)
        {
            pinMode(in_pin_, OUTPUT);
            pinMode(pwm_pin_, OUTPUT);

            if(pwm_frequency > 0)
            {
                analogWriteFrequency(pwm_pin_, pwm_frequency);
            }
            analogWriteResolution(pwm_bits);

            //ensure that the motor is in neutral state during bootup
            analogWrite(pwm_pin_, abs(0));
        }

        void brake() override
        {
            analogWrite(pwm_pin_, 0);
        }

        float readCurrent() override
        {
            if (current_pin_ == -1) {
                return 0.0;
            }
            float v_in = (float)analogRead(current_pin_)*3.3/1023.0;
            if (last_pwm == 0) {
                voltage_ref = voltage_ref*0.95 + v_in*0.05;
            }
#if 1   // Current measurement of 18V18 Pololu driver board
            float current = v_in/0.020;
            current_ave = current_ave*0.95 + current*0.05;
#else            
            // ACS712-based current sensor (5A range)
            float current = (voltage_ref - v_in)/0.185;
            current_ave = current_ave*0.95 + current*0.05;
#endif            
            return current_ave;

        }

};

class BTS7960: public MotorInterface
{
    private:
        int in_a_pin_;
        int in_b_pin_;
        int *dir_status_out_;

    protected:
        void forward(int pwm) override
        {
            analogWrite(in_a_pin_, 0);
            analogWrite(in_b_pin_, abs(pwm));
            if (dir_status_out_) {
                *dir_status_out_ = invert_? 0: 1;
            }                
        }

        void reverse(int pwm) override
        {
            analogWrite(in_b_pin_, 0);
            analogWrite(in_a_pin_, abs(pwm));
            if (dir_status_out_) {
                *dir_status_out_ = invert_? 1: 0;
            }                
        }

    public:
        BTS7960(float pwm_frequency, int pwm_bits, bool invert, int unused, int in_a_pin, int in_b_pin, int *dir_status_out = NULL): 
            MotorInterface(invert),
            in_a_pin_(in_a_pin),
            in_b_pin_(in_b_pin),
            dir_status_out_(dir_status_out)
        {
            pinMode(in_a_pin_, OUTPUT);
            pinMode(in_b_pin_, OUTPUT);

            if(pwm_frequency > 0)
            {
                analogWriteFrequency(in_a_pin_, pwm_frequency);
                analogWriteFrequency(in_b_pin_, pwm_frequency);

            }
            analogWriteResolution(pwm_bits);

            //ensure that the motor is in neutral state during bootup
            analogWrite(in_a_pin_, 0);
            analogWrite(in_b_pin_, 0);
        }
    
        BTS7960(float pwm_frequency, int pwm_bits, bool invert, int in_a_pin, int in_b_pin): 
            MotorInterface(invert),
            in_a_pin_(in_a_pin),
            in_b_pin_(in_b_pin)
        {
            pinMode(in_a_pin_, OUTPUT);
            pinMode(in_b_pin_, OUTPUT);

            if(pwm_frequency > 0)
            {
                analogWriteFrequency(in_a_pin_, pwm_frequency);
                analogWriteFrequency(in_b_pin_, pwm_frequency);

            }
            analogWriteResolution(pwm_bits);

            //ensure that the motor is in neutral state during bootup
            analogWrite(in_a_pin_, 0);
            analogWrite(in_b_pin_, 0);
        }

        void brake() override
        {
            analogWrite(in_b_pin_, 0);
            analogWrite(in_a_pin_, 0);            
        }
};

class ESC: public MotorInterface
{
    private:
        Servo motor_;
        int pwm_pin_;

    protected:
        void forward(int pwm) override
        {
            motor_.writeMicroseconds(1500 + pwm);
        }

        void reverse(int pwm) override
        {
            motor_.writeMicroseconds(1500 + pwm);
        }

    public:
        ESC(float pwm_frequency, int pwm_bits, bool invert, int pwm_pin, int unused=-1, int unused2=-1): 
            MotorInterface(invert),
            pwm_pin_(pwm_pin)
        {
            motor_.attach(pwm_pin);
            
            //ensure that the motor is in neutral state during bootup
            motor_.writeMicroseconds(1500);
        }

        void brake() override
        {
            motor_.writeMicroseconds(1500);         
        }
};

class RoboClawController
{
    private:
        const int ROBOCLAW_MAX_LEVEL = 127;

    private:
        uint8_t address_;
        uint16_t baud_rate_;
        HardwareSerial &serial_;
        bool invert_serial_;
        uint32_t timeout_;
        int num_channels_;
        RoboClaw roboclaw_;

    protected:

    public:
        RoboClawController(uint8_t address, uint16_t baud_rate, HardwareSerial &serial, bool invert_serial, uint32_t timeout, int num_channels): 
            address_(address),
            baud_rate_(baud_rate),
            serial_(serial),
            invert_serial_(invert_serial),
            timeout_(timeout),
            num_channels_(num_channels),
            roboclaw_(&serial_, timeout_)
        {
            assert(num_channels_ > 0);
            roboclaw_.begin(baud_rate);

            // Configure serial I/O for inverted levels if needed (such as when using
            // opto-isolators that result in an inversion).  Although Roboclaw::begin calls
            // serial begin, call it again here with the serial options if needed.

            if (invert_serial)
            {
                serial_.begin(baud_rate, SERIAL_8N1_RXINV_TXINV);
            }            
        }

        ~RoboClawController()
        {
            stop_all();
        }

        void stop_all()
        {
            roboclaw_.clear();
            for (int i = 0; i < num_channels_; i++)
            {
                if (i == 0)
                {
                    roboclaw_.ForwardM1(address_, 0);
                }
                else
                {
                    roboclaw_.ForwardM2(address_, 0);
                }
            }
            delay(20);
        }

        bool set_level(int channel, int level)
        {
            roboclaw_.clear();
            if (channel > num_channels_ - 1)
            {
                return false;
            }

            if (level > ROBOCLAW_MAX_LEVEL) {
                level = ROBOCLAW_MAX_LEVEL;
            } else if (level < -ROBOCLAW_MAX_LEVEL) {
                level = -ROBOCLAW_MAX_LEVEL;
            }

            bool result = false;
            if (channel == 0)
            {
                if (level < 0)
                {
                    result = roboclaw_.BackwardM1(address_, abs(level));
                }
                else
                {
                    result = roboclaw_.ForwardM1(address_, abs(level));
                }                    
            }
            else
            {
                if (level < 0)
                {
                    result = roboclaw_.BackwardM2(address_, abs(level));
                }
                else
                {
                    result = roboclaw_.ForwardM2(address_, abs(level));
                }                    
            }
            delay(20);
            return result;
        }
};

class RoboClawMotor: public MotorInterface
{
    private:
        const int INVALID_SETTING = 9999;

    private:
        RoboClawController &controller_;
        int channel_;
        int *dir_status_out_;
        int last_setting_;

    private:

    protected:
        void forward(int level) override
        {
            if (level != last_setting_) {
                last_setting_ = level;
                controller_.set_level(channel_, level);
                if (dir_status_out_)
                {
                    *dir_status_out_ = invert_? 0: 1;
                }                
            }                
        }

        void reverse(int level) override
        {
            if (level != last_setting_) {
                last_setting_ = level;
                controller_.set_level(channel_, level);
                if (dir_status_out_)
                {
                    *dir_status_out_ = invert_? 1: 0;
                }                
            }                
        }

    public:
        RoboClawMotor(RoboClawController &controller, int channel, bool invert, int *dir_status_out = NULL):
            MotorInterface(invert),
            controller_(controller),
            channel_(channel),
            dir_status_out_(dir_status_out),
            last_setting_(INVALID_SETTING)
        {
            brake();
        }

        void brake() override
        {
            controller_.set_level(channel_, 0);
        }
};
#endif

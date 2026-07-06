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

#ifndef DEFAULT_IMU
#define DEFAULT_IMU

//include IMU base interface
#include "imu_interface.h"

//include sensor API headers
#include "I2Cdev.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "HMC5883L.h"
#include "MPU6050.h"
#include "MPU9150.h"
#include "MPU9250.h"

class GY85IMU: public IMUInterface 
{
    private:
        //constants specific to the sensor
        const float accel_scale_ = 1 / 256.0;
        const float gyro_scale_ = 1 / 14.375;

        const int16_t HMC5883L_INVALID_RAW_GAUSS = -4096;
        const float HMC5883L_GAIN_1370_SCALE = 0.73;
        const float MILLI_GAUSS_PER_TELSA = 10000000.0;

        // driver objects to be used
        ADXL345 accelerometer_;
        ITG3200 gyroscope_;
        HMC5883L mag_;

        // returned vector for sensor reading
        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;
        geometry_msgs__msg__Vector3 mag_field_;

    public:
        GY85IMU()
        {
            // accel_cov_ = 0.001; //you can overwrite the convariance values here
            // gyro_cov_ = 0.001; //you can overwrite the convariance values here
        }

        bool startSensor() override
        {
            // here you can override startSensor() function and use the sensor's driver API
            // to initialize and test the sensor's connection during boot time
            Wire.begin();
            bool ret;
            accelerometer_.initialize();
            ret = accelerometer_.testConnection();
            if(!ret)
                return false;

            gyroscope_.initialize();
            ret = gyroscope_.testConnection();
            if(!ret)
                return false;

            mag_.testConnection();
            if(!ret)
                return false;

            mag_.initialize();
            mag_.setMode(HMC5883L_MODE_CONTINUOUS);
            mag_.setDataRate(HMC5883L_RATE_15);
            mag_.setGain(HMC5883L_GAIN_1370);
            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            // Seem to be reading bogus values from the accelerometer.
            // Is it my particular IMU or a bug in the code?
            // Disable its use for now.
#if 0            
            // here you can override readAccelerometer function and use the sensor's driver API
            // to grab the data from accelerometer and return as a Vector3 object
            int16_t ax, ay, az;
            
            accelerometer_.getAcceleration(&ax, &ay, &az);

            accel_.x = ax * (double) accel_scale_ * g_to_accel_;
            accel_.y = ay * (double) accel_scale_ * g_to_accel_;
            accel_.z = az * (double) accel_scale_ * g_to_accel_;
#else
            accel_.x = 0.0;
            accel_.y = 0.0;            
            accel_.z = g_to_accel_;            
#endif            

            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            // here you can override readAccelerometer function and use the sensor's driver API
            // to grab the data from gyroscope and return as a Vector3 object
            int16_t gx, gy, gz;

            gyroscope_.getRotation(&gx, &gy, &gz);

            gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

            return gyro_;
        }

        geometry_msgs__msg__Vector3 readMagnetometer() override
        {
            int16_t x, y, z = 0;
            mag_.getHeading(&x, &y, &z);
            if (x == HMC5883L_INVALID_RAW_GAUSS ||
                y == HMC5883L_INVALID_RAW_GAUSS ||
                z == HMC5883L_INVALID_RAW_GAUSS)
            {
                mag_field_.x = NAN;
                mag_field_.y = NAN;
                mag_field_.z = NAN;
            }
            else
            {          
                // units: uT x 10 (= milligauss)
                float mag_data[3] = {x*HMC5883L_GAIN_1370_SCALE,
                                     y*HMC5883L_GAIN_1370_SCALE,
                                     z*HMC5883L_GAIN_1370_SCALE};

#if 1   // IMU not mounted on robot and away from other objects
                // move
                float hard_iron_cal[3] = {-95.3, -113.2, 30.0};
                float soft_iron_cal[3][3] = {
                    { 1.002, 0.000, 0.002},
                    { 0.000, 0.938, 0.001},
                    {-0.002, 0.001, 1.065}
                };
#endif                
#if 0   // IMU mounted on the robot, MotionCal (with somewhat limited orientations)
                float hard_iron_cal[3] = {-30.2, -3.09, 136.25};
                float soft_iron_cal[3][3] = {
                    { 0.899, 0.022, 0.024},
                    { 0.004, 0.949, 0.043},
                    { 0.024, 0.043, 1.176}
                };
#endif
#if 0   // IMU mounted on the robot, MotionCal (mount midway below display panel)
        // (With power relay ON since this affected the heading by a few degrees.)
                float hard_iron_cal[3] = {-52.6, -114.2, 34.6};
                float soft_iron_cal[3][3] = {
                    { 1.011, 0.002, 0.005},
                    { 0.002, 0.938, 0.009},
                    {-0.005, 0.009, 1.055}
                };
#endif


                float h_cal[3];
                for (int i = 0; i < 3; i++) {
                    h_cal[i] = mag_data[i] - hard_iron_cal[i];
                }

                for (int i = 0; i < 3; i++) {
                    mag_data[i] = (soft_iron_cal[i][0] * h_cal[0]) +
                                  (soft_iron_cal[i][1] * h_cal[1]) +
                                  (soft_iron_cal[i][2] * h_cal[2]);
                }

                mag_field_.x =  mag_data[0]/MILLI_GAUSS_PER_TELSA;
                mag_field_.y =  mag_data[1]/MILLI_GAUSS_PER_TELSA;
                mag_field_.z =  mag_data[2]/MILLI_GAUSS_PER_TELSA;
            }
            return mag_field_;
        }
};


class MPU6050IMU: public IMUInterface 
{
    private:
        const float accel_scale_ = 1 / 16384.0;
        const float gyro_scale_ = 1 / 131.0;

        MPU6050 accelerometer_;
        MPU6050 gyroscope_;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        MPU6050IMU()
        {
        }

        bool startSensor() override
        {
            Wire.begin();
            bool ret;
            accelerometer_.initialize();
            ret = accelerometer_.testConnection();
            if(!ret)
                return false;

            gyroscope_.initialize();
            ret = gyroscope_.testConnection();
            if(!ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            int16_t ax, ay, az;
            
            accelerometer_.getAcceleration(&ax, &ay, &az);

            accel_.x = ax * (double) accel_scale_ * g_to_accel_;
            accel_.y = ay * (double) accel_scale_ * g_to_accel_;
            accel_.z = az * (double) accel_scale_ * g_to_accel_;

            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            int16_t gx, gy, gz;

            gyroscope_.getRotation(&gx, &gy, &gz);

            gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

            return gyro_;
        }
};

class MPU9150IMU: public IMUInterface 
{
    private:
        const float accel_scale_ = 1 / 16384.0;
        const float gyro_scale_ = 1 / 131.0;

        MPU9150 accelerometer_;
        MPU9150 gyroscope_;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        MPU9150IMU()
        {
        }

        bool startSensor() override
        {
            Wire.begin();
            bool ret;
            accelerometer_.initialize();
            ret = accelerometer_.testConnection();
            if(!ret)
                return false;

            gyroscope_.initialize();
            ret = gyroscope_.testConnection();
            if(!ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            int16_t ax, ay, az;
            
            accelerometer_.getAcceleration(&ax, &ay, &az);

            accel_.x = ax * (double) accel_scale_ * g_to_accel_;
            accel_.y = ay * (double) accel_scale_ * g_to_accel_;
            accel_.z = az * (double) accel_scale_ * g_to_accel_;

            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            int16_t gx, gy, gz;

            gyroscope_.getRotation(&gx, &gy, &gz);

            gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

            return gyro_;
        }
};

class MPU9250IMU: public IMUInterface 
{
    private:
        const float accel_scale_ = 1 / 16384.0;
        const float gyro_scale_ = 1 / 131.0;

        MPU9250 accelerometer_;
        MPU9250 gyroscope_;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        MPU9250IMU()
        {
        }

        bool startSensor() override
        {
            Wire.begin();
            bool ret;
            accelerometer_.initialize();
            ret = accelerometer_.testConnection();
            if(!ret)
                return false;

            gyroscope_.initialize();
            ret = gyroscope_.testConnection();
            if(!ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            int16_t ax, ay, az;

            accelerometer_.getAcceleration(&ax, &ay, &az);

            accel_.x = ax * (double) accel_scale_ * g_to_accel_;
            accel_.y = ay * (double) accel_scale_ * g_to_accel_;
            accel_.z = az * (double) accel_scale_ * g_to_accel_;
            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            int16_t gx, gy, gz;

            gyroscope_.getRotation(&gx, &gy, &gz);

            gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

            return gyro_;
        }
};

class FakeIMU: public IMUInterface 
{
    private:
        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        FakeIMU()
        {
        }

        bool startSensor() override
        {
            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            return gyro_;
        }
};

#endif
//ADXL345 https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
//HMC8553L https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
//ITG320 https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf


//MPU9150 https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
//MPU9250 https://www.invensense.com/wp-content/uploads/2015/02/MPU-9150-Datasheet.pdf
//MPU6050 https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf

//http://www.sureshjoshi.com/embedded/invensense-imus-what-to-know/
//https://stackoverflow.com/questions/19161872/meaning-of-lsb-unit-and-unit-lsb
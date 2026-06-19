#include "Arduino.h"
#include <stdlib.h>
#include <stdio.h>
#include <limits>
#include <cstdint>

#include <micro_ros_platformio.h>
#include <rclc/rclc.h>
#ifdef PUB_AS_POINT_CLOUD
#include <sensor_msgs/msg/point_cloud2.h>
#include <sensor_msgs/msg/point_field.h>
#else
#include <sensor_msgs/msg/laser_scan.h>
#endif

#include "ros_vl53l7cx_tof_sensor.h"
#include "logger.h"
#include "time_util.h"

#undef DEBUG_LOG
#define DEBUG_FUNC log_message

#define LIMIT_Z
#undef USE_DIST_TEST_VALUES

namespace {
  const int I2C_FREQ = 400000;
  const int RANGING_FREQ = 5;
  
  const int READ_DURATION_CK_LIMIT = 10;          // The expected read time for the TOF status.  This assumes 400kHz I2C freq and
                                                  // all unneeded VL53L7CX status field disabled via the VL53L7CX_DISABLE_* defines in 
                                                  // 'Adafruit VL53L7CX/src/platform.h'.  Only VL53L7CX_DISABLE_DISTANCE_MM and 
                                                  // VL53L7CX_DISABLE_TARGET_STATUS should be undefined.

  const float FOV_RAD = 60.0f * (M_PI / 180.0f);
  const float ZONE_STEP = FOV_RAD / 8.0f;
  const float START_OFFSET = -0.5f * FOV_RAD + (ZONE_STEP / 2.0f);

  const float SENSOR_FROM_FLOOR_Z = 0.157;
#ifdef LIMIT_Z  
  const float MIN_DIST_FROM_FLOOR = 0.07;         // Filter too-low values locally before publishing.  While this can be done via a nav2 costmap
                                                  // setting, visualization is cleaner if filtered at this level.
#else
  const float MIN_DIST_FROM_FLOOR = -99.9;        // Let the Nav2 costmap handle too-low values
#endif  
  const float MIN_Z_POINT = -(SENSOR_FROM_FLOOR_Z - MIN_DIST_FROM_FLOOR);

  const float INVALID_X_DIST = 10.0;
}

RosVl53l7cxTofSensor::RosVl53l7cxTofSensor(TwoWire* wire, bool rotate_180, const String &frame_name, const String &topic_name):
    wire_(wire),
    rotate_180_(rotate_180),
    frame_name_(frame_name),
    topic_name_base_(topic_name)
{}

bool RosVl53l7cxTofSensor::init(rcl_node_t &node)
{
    Logger::DEBUG_FUNC(Logger::LogLevel::Info, "RosVl53l7cxTofSensor: init");

    if (state_ == State::reset) {
        if (!sensor_inited_) {
            Logger::DEBUG_FUNC(Logger::LogLevel::Info, "RosVl53l7cxTofSensor: init, error call sensor_init first");
            return false;
        }

#ifdef PUB_AS_POINT_CLOUD
        topic_name_point_cloud_ = topic_name_base_ + "_point_cloud";
        rclc_publisher_init_default(
            &publisher_point_cloud_,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2),
            topic_name_point_cloud_.c_str());
        init_point_cloud_msg();
#endif
#ifdef PUB_AS_SCAN
        topic_name_scan_ = topic_name_base_ + "_scan";
        rclc_publisher_init_default(
            &publisher_scan_,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
            topic_name_scan_.c_str());
        init_scan_msg();

        topic_name_scan_clear_ = topic_name_base_ + "_scan_clear";
        rclc_publisher_init_default(
            &publisher_scan_clear_,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
            topic_name_scan_clear_.c_str());
        init_scan_clear_msg();

#endif

        state_ = State::init;
    }
    return true;
}

void RosVl53l7cxTofSensor::destroy(rcl_node_t &node)
{
    if (state_ != State::reset) {
#ifdef PUB_AS_POINT_CLOUD
        rcl_publisher_fini(&publisher_point_cloud_, &node);
#endif
#ifdef PUB_AS_SCAN        
        rcl_publisher_fini(&publisher_scan_, &node);
#endif        
        state_ = State::reset;
    }
}

bool RosVl53l7cxTofSensor::update()
{
    if (state_ == State::reset) {
        return false;
    }

    if (!vl53l7cx_.isDataReady()) {
        return false;
    }

    VL53L7CX_ResultsData results;

    auto read_start_time = millis();

    if (vl53l7cx_.getRangingData(&results)) {
        if (ck_read_time_) {
            ck_read_time_ = false;
            auto read_duration = millis() - read_start_time;
            if (read_duration > READ_DURATION_CK_LIMIT) {
                Logger::log_message(Logger::LogLevel::Warn,
                    "RosVl53l7cxTofSensor::update, the status read duration is longer (%d ms) than expected (%d ms)",
                    read_duration, READ_DURATION_CK_LIMIT);
            } else {
                Logger::log_message(Logger::LogLevel::Info,
                    "RosVl53l7cxTofSensor::update, the status read duration is okay (%d ms) compared to expected (%d ms)",
                    read_duration, READ_DURATION_CK_LIMIT);
            }
        }

#ifdef PUB_AS_POINT_CLOUD
        populate_point_cloud(&results);
        {
            struct timespec time_stamp = TimeUtil::get_time();
            cloud_msg_.header.stamp.sec = time_stamp.tv_sec;
            cloud_msg_.header.stamp.nanosec = time_stamp.tv_nsec;
            rcl_publish(&publisher_point_cloud_, &cloud_msg_, NULL);
        }          
#endif
#ifdef PUB_AS_SCAN
        populate_scan(&results);
        {
            struct timespec time_stamp = TimeUtil::get_time();

            scan_clear_msg_.header.stamp.sec = time_stamp.tv_sec;
            scan_clear_msg_.header.stamp.nanosec = time_stamp.tv_nsec;
            rcl_publish(&publisher_scan_clear_, &scan_clear_msg_, NULL);

            scan_msg_.header.stamp.sec = time_stamp.tv_sec;
            scan_msg_.header.stamp.nanosec = time_stamp.tv_nsec;
            rcl_publish(&publisher_scan_, &scan_msg_, NULL);
        }          
#endif
        return true;
    } else {
        Logger::DEBUG_FUNC(Logger::LogLevel::Error, "RosVl53l7cxTofSensor: failed to read ranging");
    }
    return false;
}

bool RosVl53l7cxTofSensor::sensor_init(uint8_t new_address)
{
    // Initialize I2C
    Logger::log_message_serial(Logger::LogLevel::Info, "RosVl53l7cxTofSensor: starting init");

    if (!vl53l7cx_.begin(VL53L7CX_DEFAULT_ADDRESS, wire_, I2C_FREQ)) {
        if (new_address != address_) {
            if (!vl53l7cx_.begin(new_address, wire_, I2C_FREQ)) {
                Logger::log_message_serial(Logger::LogLevel::Error,
                    "RosVl53l7cxTofSensor: Error, failed init i2c for both current and requested addresses, cur 0x%02x, new: 0x%02x",
                    address_, new_address);
                return false;
            }
            Logger::log_message_serial(Logger::LogLevel::Info,
                "RosVl53l7cxTofSensor: init ok but device was already using the new address: 0x%02x", new_address);
                address_ = new_address;
        } else {
            Logger::log_message_serial(Logger::LogLevel::Error, "RosVl53l7cxTofSensor: Error, failed init i2c, address: 0x%02x", address_);
            return false;
        }
    }

    // If a different address was specified, then change it now.  If more than one vl53l7cx device is
    // in use, then the caller should orchestrate reseting all and then initing each device.
    if (new_address != address_) {
        // Note that the setAddress method was fixed in the rshorton fork of the Adafruit library.  (The original assumed the default I2C
        // interface was used.)
        if (!vl53l7cx_.setAddress(new_address, wire_)) {
            Logger::log_message_serial(Logger::LogLevel::Error, "RosVl53l7cxTofSensor: Error, failed to change i2c address from 0x%02x to 0x%02x",
                                       address_, new_address);
            return false;
        } else {
            Logger::log_message_serial(Logger::LogLevel::Info,
                "RosVl53l7cxTofSensor: Info, changed i2c address from 0x%02x to 0x%02x",
                address_, new_address);
        }
        address_ = new_address;
    }
    
    if (!vl53l7cx_.setResolution(RESOLUTION)) {
        Logger::log_message_serial(Logger::LogLevel::Error, "RosVl53l7cxTofSensor: Error, failed to set resolution.");
        return false;
    }

    if (!vl53l7cx_.setRangingFrequency(RANGING_FREQ)) {
        Logger::log_message_serial(Logger::LogLevel::Error, "RosVl53l7cxTofSensor: Error, failed to set ranging frequency.");
        return false;
    }

    if (!vl53l7cx_.setRangingMode(VL53L7CX_RANGING_MODE_CONTINUOUS)) {
        Logger::log_message_serial(Logger::LogLevel::Error, "RosVl53l7cxTofSensor: Error, failed to set mode.");
        return false;
    }

    uint32_t int_time =  vl53l7cx_.getIntegrationTime();
    uint8_t mode = vl53l7cx_.getRangingMode();
    uint8_t sharpener = vl53l7cx_.getSharpenerPercent();
    Logger::log_message_serial(Logger::LogLevel::Info, "RosVl53l7cxTofSensor: int time: %d, mode: %d, sharpener: %d",
                               int_time, (int)mode, (int)sharpener);

    if (!vl53l7cx_.startRanging()) {
        Logger::log_message_serial(Logger::LogLevel::Error, "RosVl53l7cxTofSensor: Error, failed to start ranging.");
        return false;
    }
    Logger::log_message_serial(Logger::LogLevel::Info, "RosVl53l7cxTofSensor: Initialized.");

    sensor_inited_ = true;

    return true;
}

#ifdef PUB_AS_POINT_CLOUD
void RosVl53l7cxTofSensor::init_point_cloud_msg()
{
    // 1. Set basic header metadata
    cloud_msg_.header.frame_id.data = (char *)frame_name_.c_str();
    cloud_msg_.header.frame_id.size = frame_name_.length() + 1;;
    cloud_msg_.header.frame_id.capacity = cloud_msg_.header.frame_id.size + 1;
    
    // 2. Configure a 1D unstructured array of RESOLUTION points (default 8x8 grid)
    cloud_msg_.height = 1;
    cloud_msg_.width = RESOLUTION;
    cloud_msg_.is_bigendian = false;
    cloud_msg_.point_step = 12; // 3 fields * 4 bytes (float32)
    cloud_msg_.row_step = cloud_msg_.point_step * cloud_msg_.width;
    cloud_msg_.is_dense = true;

    // 3. Define the 3 coordinate fields (X, Y, Z)
    cloud_msg_.fields.size = 3;
    cloud_msg_.fields.capacity = 3;
    cloud_msg_.fields.data = (sensor_msgs__msg__PointField*) malloc(3 * sizeof(sensor_msgs__msg__PointField));

    // X Field Configuration
    cloud_msg_.fields.data[0].name.data = (char*)"x";
    cloud_msg_.fields.data[0].name.size = 1;
    cloud_msg_.fields.data[0].offset = 0;
    cloud_msg_.fields.data[0].datatype = sensor_msgs__msg__PointField__FLOAT32;
    cloud_msg_.fields.data[0].count = 1;

    // Y Field Configuration
    cloud_msg_.fields.data[1].name.data = (char*)"y";
    cloud_msg_.fields.data[1].name.size = 1;
    cloud_msg_.fields.data[1].offset = 4;
    cloud_msg_.fields.data[1].datatype = sensor_msgs__msg__PointField__FLOAT32;
    cloud_msg_.fields.data[1].count = 1;

    // Z Field Configuration
    cloud_msg_.fields.data[2].name.data = (char*)"z";
    cloud_msg_.fields.data[2].name.size = 1;
    cloud_msg_.fields.data[2].offset = 8;
    cloud_msg_.fields.data[2].datatype = sensor_msgs__msg__PointField__FLOAT32;
    cloud_msg_.fields.data[2].count = 1;

    // 4. Allocate dynamic memory for the binary point payload
    cloud_msg_.data.capacity = cloud_msg_.row_step * cloud_msg_.height;
    cloud_msg_.data.size = cloud_msg_.data.capacity;
    cloud_msg_.data.data = (uint8_t*) malloc(cloud_msg_.data.capacity);
}

void RosVl53l7cxTofSensor::populate_point_cloud(VL53L7CX_ResultsData *results)
{
    uint8_t* byte_ptr = cloud_msg_.data.data;

    for (int r = 0; r < RES_W; r++) {
        // Calculate vertical angle offset relative to optical center
        float angle_z = (START_OFFSET + (r * ZONE_STEP))*-1.0f;
        int sensor_row = rotate_180_? RES_W - r - 1: r;

        
        for (int c = 0; c < RES_H; c++) {
            // Calculate horizontal angle offset relative to optical center
            float angle_y = (START_OFFSET + (c * ZONE_STEP))*-1.0f;
            
            // Map 2D matrix indices to the driver's continuous 1D block
            int sensor_col = rotate_180_? RES_H - c - 1: c;
            int zone_idx = (sensor_row * RES_W) + sensor_col; 


            // Convert to meters
            // x,y,z using ROS coord convention (x forward, y left, z up)
#if defined(USE_DIST_TEST_VALUES)            
            float x_m = get_dist_test_value(r, c)/1000.0f;
#else
            float x_m = (float)results->distance_mm[zone_idx] / 1000.0f;
#endif            
            float z_m = x_m * tanf(angle_z);
            float y_m = x_m * tanf(angle_y);

            bool valid = results->target_status[zone_idx] == 5 ||
                         results->target_status[zone_idx] == 6 ||
                         results->target_status[zone_idx] == 9;
            // Report invalid points at distance farther than allowed
            // for the nav2 costmap.                         
            if (!valid || z_m < MIN_Z_POINT) {
                x_m = INVALID_X_DIST;
                z_m = x_m * tanf(angle_z);
                y_m = x_m * tanf(angle_y);
            }

            // Directly copy float data into the binary byte packet stream
            memcpy(byte_ptr, &x_m, 4);      // Offsets 0-3
            memcpy(byte_ptr + 4, &y_m, 4);  // Offsets 4-7
            memcpy(byte_ptr + 8, &z_m, 4);  // Offsets 8-11
            
            // Step forward by 12 bytes to point to the next structural point index
            byte_ptr += cloud_msg_.point_step;
        }
    }
}
#endif
#ifdef PUB_AS_SCAN
void RosVl53l7cxTofSensor::init_scan_msg()
{
    scan_msg_.header.frame_id.data = (char *)frame_name_.c_str();
    scan_msg_.header.frame_id.size = frame_name_.length() + 1;;
    scan_msg_.header.frame_id.capacity = scan_msg_.header.frame_id.size + 1;

    scan_msg_.angle_min = -(FOV_RAD / 2.0);
    scan_msg_.angle_max = (FOV_RAD / 2.0);
    scan_msg_.angle_increment = FOV_RAD / (RES_W - 1);
    scan_msg_.time_increment = 0.0;
    scan_msg_.scan_time = 1.0 / 15.0;
    scan_msg_.range_min = 0.02; // 2 cm
    scan_msg_.range_max = 3.5;  // 3.5 meters

    // Link static ranges array pointer directly to micro-ROS memory frame
    scan_msg_.ranges.data = scan_ranges_;
    scan_msg_.ranges.size = RES_W;
    scan_msg_.ranges.capacity = RES_W;    
}

void RosVl53l7cxTofSensor::init_scan_clear_msg()
{
    scan_clear_msg_.header.frame_id.data = (char *)frame_name_.c_str();
    scan_clear_msg_.header.frame_id.size = frame_name_.length() + 1;;
    scan_clear_msg_.header.frame_id.capacity = scan_clear_msg_.header.frame_id.size + 1;

    scan_clear_msg_.angle_min = -(FOV_RAD / 2.0);
    scan_clear_msg_.angle_max = (FOV_RAD / 2.0);
    scan_clear_msg_.angle_increment = FOV_RAD / (RES_CLEAR - 1);
    scan_clear_msg_.time_increment = 0.0;
    scan_clear_msg_.scan_time = 1.0 / 15.0;
    scan_clear_msg_.range_min = 0.02; // 2 cm
    scan_clear_msg_.range_max = 3.5;  // 3.5 meters

    // Link static ranges array pointer directly to micro-ROS memory frame
    scan_clear_msg_.ranges.data = scan_clear_ranges_;
    scan_clear_msg_.ranges.size = RES_CLEAR;
    scan_clear_msg_.ranges.capacity = RES_CLEAR;

  for (int col = 0; col < RES_CLEAR; col++) {
      scan_clear_msg_.ranges.data[col] = scan_clear_msg_.range_max;    
  }
}

void RosVl53l7cxTofSensor::populate_scan(VL53L7CX_ResultsData *results)
{
    // Initialize temporary parsing arrays with maximum possible values
    float horizontal_slice[RES_W];
    for (int i = 0; i < RES_W; i++) {
      horizontal_slice[i] = 10.0;
    }

    // Step 1: Collapse the 8x8 matrix down into vertical row maximum boundaries
    for (int row = 0; row < RES_H; row++) {
        // Calculate vertical angle offset relative to optical center
        float angle_z = (START_OFFSET + (row * ZONE_STEP))*-1.0f;

        for (int col = 0; col < RES_W; col++) {
        
            // VL53L7CX natively stores arrays internally row-by-row
            int index = (row * RES_H) + col; 
        
            // Get status and convert millimeters data into meters
            uint8_t status = results->target_status[index];
            float distance_m = (float)results->distance_mm[index] / 1000.0;

            float z_m = distance_m * tanf(angle_z);

            bool valid = status == 5 ||
                         status == 6 ||
                         status == 9;

            // Drop if below min height
            if (valid && z_m < -(0.165 - 0.05)) {
                valid = false;
            }

            if (valid && distance_m < horizontal_slice[col]) {
                horizontal_slice[col] = distance_m;
            }
        }
    }

    // Step 2: Reverse index arrays to comply with counter-clockwise ROS systems (Right-to-Left)
    for (int col = 0; col < RES_H; col++) {
        int reversed_col = (RES_H - 1) - col;
      
        // If a column failed to find any valid target, set to maximum sensor range limit
        if (horizontal_slice[reversed_col] > 4.0) {
            scan_msg_.ranges.data[col] = scan_msg_.range_max;
        } else {
            scan_msg_.ranges.data[col] = horizontal_slice[reversed_col];
        }
    }  
}
#endif

float RosVl53l7cxTofSensor::get_dist_test_value(int r, int c) const
{
  return ((r*8)+c)*500.0f;
}

#ifndef ROS_vl53l7cx_TOF_SENSOR_H
#define ROS_vl53l7cx_TOF_SENSOR_H

#include "Arduino.h"
#include <Wire.h>
#include <rcl/rcl.h>

#include <Adafruit_VL53L7CX.h>

#define PUB_AS_POINT_CLOUD
#undef PUB_AS_SCAN

#ifdef PUB_AS_POINT_CLOUD
#include <sensor_msgs/msg/point_cloud2.h>
#endif
#ifdef PUB_AS_SCAN
#include <sensor_msgs/msg/laser_scan.h>
#endif

// This class uses the Adafruit_VL53L7CX library to initialize and read ranging data
// from the VL53L7CX device.  It publishes the results as a point cloud.  (The class
// includes support for alternatively publishing as a scan message, but is disabled.)

class RosVl53l7cxTofSensor
{
public:
    static const uint8_t DEFAULT_ADDRESS = VL53L7CX_DEFAULT_ADDRESS;

private:
    enum class State { reset, init };
    static const int RES_W = 8;
    static const int RES_H = 8;
    static const int RESOLUTION = RES_W*RES_H;
    static const int RES_CLEAR = (RES_W - 1)*4 + 1;

public:
    RosVl53l7cxTofSensor(TwoWire* wire, bool rotate_180, const String &frame_name, const String &topic_name);

    bool init(rcl_node_t &node);
    void destroy(rcl_node_t &node);

    bool update();

    bool sensor_init(uint8_t new_address = RosVl53l7cxTofSensor::DEFAULT_ADDRESS);

private:
#ifdef PUB_AS_POINT_CLOUD
    void init_point_cloud_msg();
    void populate_point_cloud(VL53L7CX_ResultsData *results);
#endif
#ifdef PUB_AS_SCAN
    void init_scan_msg();
    void init_scan_clear_msg();
    void populate_scan(VL53L7CX_ResultsData *results);
#endif    

    float get_dist_test_value(int r, int c) const;
private:
    TwoWire* wire_;
    bool rotate_180_;                       // True if sensor is rotated 180 degrees.  While a rotation could be handled via the
                                            // URDF, handling it here allowsreadings that are too low relative to the floor 
                                            // to be filtered by this class if desired.
    Adafruit_VL53L7CX vl53l7cx_;
    const String frame_name_;
    const String topic_name_base_;

    uint8_t address_{DEFAULT_ADDRESS};

    bool sensor_inited_{false};
    State state_{State::reset};
    bool ck_read_time_{true};

#ifdef PUB_AS_POINT_CLOUD
    String topic_name_point_cloud_;
    rcl_publisher_t publisher_point_cloud_;
    sensor_msgs__msg__PointCloud2 cloud_msg_;
#endif
#ifdef PUB_AS_SCAN
    String topic_name_scan_;
    String topic_name_scan_clear_;
    rcl_publisher_t publisher_scan_;
    rcl_publisher_t publisher_scan_clear_;
    sensor_msgs__msg__LaserScan scan_msg_;
    sensor_msgs__msg__LaserScan scan_clear_msg_;
    float scan_ranges_[RES_W];    
    float scan_clear_ranges_[RES_CLEAR];
#endif    
};

#endif // ROS_vl53l7cx_TOF_SENSOR_H

#include "Arduino.h"

#include "ros_vl53l7cx_tof_sensor.h"
#include "logger.h"
#include "vl53l7cx_address_assigner.h"

Vl53l7cxAddressAssigner::Vl53l7cxAddressAssigner(PCF8575 &tof_reset_i2c_gpio):
    tof_reset_i2c_gpio_(tof_reset_i2c_gpio)
{}

bool Vl53l7cxAddressAssigner::add_device(RosVl53l7cxTofSensor *vl53l7cx, int reset_line, uint8_t new_address)
{
    if (device_cnt_ > MAX_DEVICES) {
        return false;
    }
    devices_[device_cnt_].sensor = vl53l7cx;
    devices_[device_cnt_].reset_line = reset_line;
    devices_[device_cnt_].new_address = new_address;
    ++device_cnt_;
    return true;
}

inline uint16_t Vl53l7cxAddressAssigner::clear_bit(uint16_t var, u_int bit_num)
{
    return var & ~((uint16_t)1 << bit_num );
}

inline uint16_t Vl53l7cxAddressAssigner::set_bit(uint16_t var, uint16_t bit_num)
{
    return var | ((uint16_t)1 << bit_num);
}

bool Vl53l7cxAddressAssigner::assign()
{
    Logger::log_message_serial(Logger::LogLevel::Info, "Vl53l7cxAddressAssigner: assign, %d devices", device_cnt_);

    // Reset all devices
    uint16_t gpio_bits = tof_reset_i2c_gpio_.read16();

    for (int i = 0; i < device_cnt_; ++i) {
        gpio_bits = clear_bit(gpio_bits, devices_[i].reset_line);
    }
    tof_reset_i2c_gpio_.write16(gpio_bits);
    Logger::log_message_serial(Logger::LogLevel::Debug, "Vl53l7cxAddressAssigner: Info, gpio bits for reset all: 0x%02x", gpio_bits);
    delay(100);

    // For each device, un-reset it, and then init it.  The device
    // implementation deals with changing the address.
    for (int i = 0; i < device_cnt_; ++i) {
        gpio_bits = set_bit(gpio_bits, devices_[i].reset_line);
        tof_reset_i2c_gpio_.write16(gpio_bits);        
        Logger::log_message_serial(Logger::LogLevel::Debug, "Vl53l7cxAddressAssigner: Info, unreset dev idx: %d, reset line: %d, gpio bits 0x%02x",
            i, devices_[i].reset_line, gpio_bits);
        delay(100);

        auto addr = devices_[i].new_address;
        if (!devices_[i].sensor->sensor_init(addr)) {
            Logger::log_message_serial(Logger::LogLevel::Error, "Vl53l7cxAddressAssigner: Error, failed to init device at address 0x%02x", addr);
            return false;
        }

    }
    return true;
}

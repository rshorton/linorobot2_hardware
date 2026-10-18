#ifndef vl53l7cx_ADDRESS_ASSIGNER_H
#define vl53l7cx_ADDRESS_ASSIGNER_H

#include "Arduino.h"

#if defined(TOF_SUPPORT_I2C_GPIO)
#include "PCF8575.h"
#endif

class RosVl53l7cxTofSensor;

// This class is used to initialize and assign the I2C address of each VL53l7cx
// device in use.  The VL5317cx does not support jumper/hard wired address assignment.
// As a result, the address must be changed at runtime by holding all devices in reset and then
// sequentially un-resetting and assigning to a unique address.

class Vl53l7cxAddressAssigner
{
protected:
    struct Device {
        RosVl53l7cxTofSensor* sensor;
        int reset_line;
        uint8_t new_address;
    };

    static constexpr int MAX_DEVICES = 8;

public:
    Vl53l7cxAddressAssigner()
    {};

    virtual bool add_device(RosVl53l7cxTofSensor* vl53l7cx, int reset_line, uint8_t new_address);
    virtual bool assign() = 0;

protected:
    Device devices_[MAX_DEVICES];
    int device_cnt_{0};
};


#if defined(TOF_SUPPORT_I2C_GPIO)
// For Vl53l7cxAddressAssigner_I2c, a PCF8575 (I2C gpio expander) is used
// to control the reset lines of the VL53l7cx while assigning addresses.

class Vl53l7cxAddressAssigner_I2c: public Vl53l7cxAddressAssigner
{
public:
    Vl53l7cxAddressAssigner_I2c(PCF8575 &tof_reset_i2c_gpio);

    bool assign() override;

private:
    inline uint16_t clear_bit(uint16_t var, u_int bit_num);
    inline uint16_t set_bit(uint16_t var, uint16_t bit_num);

    PCF8575 &tof_reset_i2c_gpio_;
};
#endif

// For Vl53l7cxAddressAssigner_gpio, local GPIO pins are used to control the reset lines of the VL53l7cx
// while assigning addresses.

class Vl53l7cxAddressAssigner_gpio: public Vl53l7cxAddressAssigner
{
public:
    Vl53l7cxAddressAssigner_gpio();

    bool assign() override;
};

#endif // vl53l7cx_ADDRESS_ASSIGNER_H

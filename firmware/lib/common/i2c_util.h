#ifndef I2C_UTIL_H
#define I2C_UTIL_H

#include <Arduino.h>
#include <Wire.h>

namespace I2CUtil {
void scan_i2c_bus(TwoWire &wire);
}

#endif // I2C_UTIL_H
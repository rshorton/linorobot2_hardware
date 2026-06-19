// This code is based on https://learn.adafruit.com/scanning-i2c-addresses/arduino
#include <Arduino.h>
#include "i2c_util.h"

namespace I2CUtil {

void scan_i2c_bus(TwoWire &wire)
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    wire.beginTransmission(address);
    error = wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }        
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;

    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }        
      Serial.println(address,HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }    
}

} // namespace I2CUtil
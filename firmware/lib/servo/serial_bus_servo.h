#ifndef HIW_SERIAL_BUS_SERVO_H
#define HIW_SERIAL_BUS_SERVO_H

#include "Arduino.h"

class SerialServo
{
public:
    SerialServo(HardwareSerial &serial);

    void move(uint8_t id, int16_t position, uint16_t time);

private:
    uint8_t calc_ck_sum(uint8_t buf[]);

private:
    Stream &serial_;
};

#endif // HIW_SERIAL_BUS_SERVO_H

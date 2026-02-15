#ifndef HIW_SERIAL_BUS_SERVO_H
#define HIW_SERIAL_BUS_SERVO_H

#include "Arduino.h"

class SerialServo
{
public:
    SerialServo(HardwareSerial &serial, int16_t id, int16_t range_degrees, int16_t range_steps, bool invert);

    void move(int16_t position, uint16_t time);
    float move(float angle, uint16_t time);

    void set_id(uint8_t newID);

private:
    uint8_t calc_ck_sum(uint8_t buf[]);
    int angle_to_steps(float angle);
    float steps_to_angle(int16_t steps);

private:
    Stream &serial_;
    int16_t id_;
    int16_t range_degrees_;
    int16_t range_steps_;
    bool invert_;
};

#endif // HIW_SERIAL_BUS_SERVO_H

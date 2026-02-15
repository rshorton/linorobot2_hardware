#include "Arduino.h"
#include "utility/direct_pin_read.h"

#include "serial_bus_servo.h"

#define GET_LOW_BYTE(A) (uint8_t)((A))
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)

namespace
{
    const uint8_t MSG_FRAME_HEADER = 0x55;
    const uint8_t MSG_CMD_MOVE_TIME_WRITE = 1;
    const uint8_t MSG_CMD_ID_WRITE = 13;
}

SerialServo::SerialServo(HardwareSerial &serial, int16_t id, int16_t range_degrees, int16_t range_steps, bool invert) :
    serial_(serial),
    id_(id),
    range_degrees_(range_degrees),
    range_steps_(range_steps),
    invert_(invert)
{
    serial.begin(115200);
}

uint8_t SerialServo::calc_ck_sum(uint8_t buf[])
{
    uint8_t i;
    uint16_t temp = 0;
    for (i = 2; i < buf[3] + 2; i++)
    {
      temp += buf[i];
    }
  
    temp = ~temp;
    i = (uint8_t)temp;
    return i;
}

void SerialServo::move(int16_t position, uint16_t time)
{
    const int msgLen = 10;
    uint8_t buf[msgLen];
    if (position < 0)
    {
        position = 0;
    }

    if (position > range_steps_)
    {
        position = range_steps_;
    }

    if (invert_)
    {
      position = range_steps_ - position;
    }

    buf[0] = buf[1] = MSG_FRAME_HEADER;
    buf[2] = id_;
    buf[3] = msgLen - 3;
    buf[4] = MSG_CMD_MOVE_TIME_WRITE;
    buf[5] = GET_LOW_BYTE(position);
    buf[6] = GET_HIGH_BYTE(position);
    buf[7] = GET_LOW_BYTE(time);
    buf[8] = GET_HIGH_BYTE(time);
    buf[9] = calc_ck_sum(buf);
    serial_.write(buf, msgLen);
}

float SerialServo::move(float angle, uint16_t time)
{
    int16_t position = angle_to_steps(angle);
    move(position, time);
    return steps_to_angle(position);
}

void SerialServo::set_id(uint8_t new_id)
{
    const int msgLen = 7;
    uint8_t buf[msgLen];
    buf[0] = buf[1] = MSG_FRAME_HEADER;
    buf[2] = id_;
    buf[3] = msgLen - 3;
    buf[4] = MSG_CMD_ID_WRITE;
    buf[5] = new_id;
    buf[6] = calc_ck_sum(buf);
    serial_.write(buf, msgLen);

    id_ = new_id;
}

int SerialServo::angle_to_steps(float angle)
{
    int16_t steps = (float)range_steps_*angle/(float)range_degrees_;
    if (steps < 0)
    {
        steps = 0;
    }
    else if (steps > range_steps_)
    {
        steps = range_steps_;
    }
    return steps;
}

float SerialServo::steps_to_angle(int16_t steps)
{
    return (float)range_degrees_*(float)steps/(float)range_steps_;
}
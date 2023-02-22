#include "Arduino.h"
#include "utility/direct_pin_read.h"

#include "serial_bus_servo.h"

#define GET_LOW_BYTE(A) (uint8_t)((A))
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)

namespace
{
  const uint8_t MSG_FRAME_HEADER = 0x55;
  const uint8_t MSG_CMD_MOVE_TIME_WRITE = 1;
}

SerialServo::SerialServo(HardwareSerial &serial) : serial_(serial)
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

void SerialServo::move(uint8_t id, int16_t position, uint16_t time)
{
  const int msgLen = 10;
  uint8_t buf[msgLen];
  if (position < 0)
  {
    position = 0;
  }

  if (position > 1000)
  {
    position = 1000;
  }

  buf[0] = buf[1] = MSG_FRAME_HEADER;
  buf[2] = id;
  buf[3] = msgLen - 3;
  buf[4] = MSG_CMD_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = calc_ck_sum(buf);
  serial_.write(buf, 10);
}
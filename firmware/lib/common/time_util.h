#ifndef TIME_UTIL_H
#define TIME_UTIL_H

#include <Arduino.h>

namespace TimeUtil {

void sync_time();
struct timespec get_time();

}

#endif // TIME_UTIL_H
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "time_util.h"
#include "logger.h"

namespace TimeUtil {

void sync_time()
{
    // get the current time from the agent
    if (rmw_uros_sync_session(10) != RMW_RET_OK) {
        Logger::log_message(Logger::LogLevel::Error, "Failed to sync time");
        return;
    }
}

struct timespec get_time()
{
    struct timespec tp = {0};
    unsigned long long now = rmw_uros_epoch_millis();
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

} // namespace TimeUtil

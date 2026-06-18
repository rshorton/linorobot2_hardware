#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl_interfaces/msg/log.h>

class Logger
{
public:
    enum class LogLevel { Disabled, Error, Warn, Info, Debug };

    class TimeProvider
    {
    public:
        virtual struct timespec get_time() = 0;
    };

public:
    static bool create_logger(rcl_node_t &node, TimeProvider &time_provider);
    static bool destroy_logger(rcl_node_t &node);
    static void log_message(Logger::LogLevel level, const char * fmt, ...);

    static void log_message_serial(LogLevel level, const char * fmt, ...);
    static void set_local_log_level(Logger::LogLevel level);

protected:
    Logger(TimeProvider &time_provider);
    bool create(rcl_node_t &node);
    void destroy(rcl_node_t &node);

private:
    void log(Logger::LogLevel level, const char* msg);

private:
    TimeProvider &time_provider_;
    bool inited_{false};
    rcl_publisher_t publisher_log_;

    rcl_interfaces__msg__Log log_msg;
};

#endif // LOGGER_H
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
    enum class LogLevel { Error, Warn, Info, Debug };
    
public:
    static bool create_logger(rcl_node_t &node);
    static bool destroy_logger(rcl_node_t &node);
    static void log_message(Logger::LogLevel level, const char * fmt, ...);

protected:
    Logger();
    bool create(rcl_node_t &node);
    void destroy(rcl_node_t &node);

private:
    void log(Logger::LogLevel level, const char* msg);

private:
    rcl_publisher_t publisher_log_;
    bool inited_;

    rcl_interfaces__msg__Log log_msg;
};

#endif // LOGGER_H
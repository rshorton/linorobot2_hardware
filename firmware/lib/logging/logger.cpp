#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/float32.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "logger.h"

namespace {
const int MAX_MSG_LEN = 1000;    
Logger *logger = nullptr;
}

bool Logger::create_logger(rcl_node_t &node)
{
    if (logger) {
        return false;
    }

    logger = new Logger();
    if (logger->create(node)) {
        return true;
    }
    delete logger;
    logger = nullptr;
}

bool Logger::destroy_logger(rcl_node_t &node)
{
    if (!logger) {
        return false;
    }
    logger->destroy(node);
    delete logger;
    logger = nullptr;
}

void Logger::log_message(LogLevel level, const char * fmt, ...)
{
    if (!logger) {
        return;
    }
	char buf[MAX_MSG_LEN];
    va_list args;
    va_start(args, fmt);

    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    logger->log(level, buf);
}

Logger::Logger():
    inited_(false)
{
}

bool Logger::create(rcl_node_t &node)
{
    if (inited_) {
        return false;
    }

    if (rclc_publisher_init_default(
	  	&publisher_log_,
	  	&node,
	  	ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
	  	"rosout") != RCL_RET_OK) {

        return false;
    }

    inited_ = true;
    return true;
}

void Logger::destroy(rcl_node_t &node)
{
    if (!inited_) {
        return;
    }
    inited_ = false;
    rcl_publisher_fini(&publisher_log_, &node);
}

void Logger::log(Logger::LogLevel level, const char *msg)
{
    if (!inited_) {
        return;
    }

    switch(level) {
        case LogLevel::Error:
            log_msg.level = rcl_interfaces__msg__Log__ERROR;
            break;
        case LogLevel::Warn:
            log_msg.level = rcl_interfaces__msg__Log__WARN;
            break;
        default:    
            log_msg.level = rcl_interfaces__msg__Log__INFO;
            break;
    }

    log_msg.name.data = (char*)"linorobot_jeep";
    log_msg.name.size = strlen(log_msg.name.data) + 1;
    log_msg.msg.data = (char*)msg;
    log_msg.msg.size = strlen(msg) + 1;
    log_msg.file.data = (char*)"";
    log_msg.file.size = strlen(log_msg.file.data) + 1;
    log_msg.function.data = (char*)"";
    log_msg.function.size = strlen(log_msg.function.data) + 1;
    log_msg.line = NULL;
    rcl_publish(&publisher_log_, &log_msg, NULL);
}

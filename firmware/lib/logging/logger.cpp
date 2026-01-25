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

// '/rosout' topic publishers need to use Durability = TRANSIENT_LOCAL to view messages in rqt app.
const rmw_qos_profile_t rmw_qos_profile = {RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10, 
    RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, RMW_QOS_DEADLINE_BEST_AVAILABLE, 
    RMW_QOS_LIFESPAN_DEFAULT, RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE, RMW_QOS_LIVELINESS_LEASE_DURATION_BEST_AVAILABLE, false};
}

bool Logger::create_logger(rcl_node_t &node, TimeProvider &time_provider)
{
    if (logger) {
        return false;
    }

    logger = new Logger(time_provider);
    if (logger->create(node)) {
        return true;
    }
    delete logger;
    logger = nullptr;
    return false;
}

bool Logger::destroy_logger(rcl_node_t &node)
{
    if (!logger) {
        return false;
    }
    logger->destroy(node);
    delete logger;
    logger = nullptr;
    return true;
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

Logger::Logger(TimeProvider &time_provider):
    time_provider_(time_provider)
{
}

bool Logger::create(rcl_node_t &node)
{
    if (inited_) {
        return false;
    }

    if (rclc_publisher_init(
	  	&publisher_log_,
	  	&node,
	  	ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
	  	"rosout", &rmw_qos_profile) != RCL_RET_OK) {

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
        case LogLevel::Debug:
            log_msg.level = rcl_interfaces__msg__Log__DEBUG;
            break;
        default:    
            log_msg.level = rcl_interfaces__msg__Log__INFO;
            break;
    }

    struct timespec time_stamp = time_provider_.get_time();
    log_msg.stamp.sec = time_stamp.tv_sec;
    log_msg.stamp.nanosec = time_stamp.tv_nsec;

    log_msg.name.data = (char*)"linorobot_elsabot";
    log_msg.name.size = strlen(log_msg.name.data) + 1;
    log_msg.msg.data = (char*)msg;
    log_msg.msg.size = strlen(msg) + 1;
    log_msg.file.data = (char*)"";
    log_msg.file.size = strlen(log_msg.file.data) + 1;
    log_msg.function.data = (char*)"";
    log_msg.function.size = strlen(log_msg.function.data) + 1;
    log_msg.line = 0;
    rcl_publish(&publisher_log_, &log_msg, NULL);
}

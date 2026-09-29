#include "Arduino.h"
#include "logger.h"
#include "power_control.h"

namespace {
const long DEF_UPDATE_PERIOD_MS = 1000;
}

PowerControl::PowerControl(int control_pin, bool control_pin_active_hi, int status_pin, bool status_pin_active_hi) :
    control_pin_(control_pin),
    control_pin_active_hi_(control_pin_active_hi),
    status_pin_(status_pin),
    status_pin_active_hi_(status_pin_active_hi)
{
}

PowerControl::~PowerControl()
{
    noInterrupts();
    update_timer_.end();
    power_control_ = nullptr;
    interrupts();
}

// Enable control of the power output pin via a periodic timer
bool PowerControl::configure(uint32_t wd_timeout_ms)
{
    pinMode(control_pin_, OUTPUT);
    pinMode(status_pin_, INPUT);

    enable_power(false);

    wd_timeout_ms_ = wd_timeout_ms;
    auto update_timer_period_ms = wd_timeout_ms > 0? wd_timeout_ms/4: DEF_UPDATE_PERIOD_MS;

    noInterrupts();
    power_control_ = this;
    interrupts();

    handshake();

    // Use a timer to update the power control output to handle case where code gets stuck
    update_timer_.end();
    update_timer_.begin(timer_handler, update_timer_period_ms*1000);

    return true;
}

// Called to indicate normal functionality which allows power to be enabled
void PowerControl::handshake()
{
    dbg_hs_cnt_++;

    noInterrupts();
    last_hs_time_ = millis();
    interrupts();
}

// Set whether power should be enabled
void PowerControl::enable(bool enable)
{
    enable_ = enable;
}

// Read the input that connects to the input of the power relay.  This
// indicates whether the relay should be on/off considering all means of
// power control (e-stop, wireless switch, uC)
bool PowerControl::is_enabled() const
{
    auto in_pin = digitalRead(status_pin_);
    return enable_ && (in_pin && status_pin_active_hi_ || !in_pin && !status_pin_active_hi_);
}

// Called by timer interrupt to update the state of the power out pin
void PowerControl::update_state()
{
    power_control_->dbg_timer_cnt_++;

    auto now = millis();
  
    // Enable power if Enabled and (watchdog hasn't expired or isn't enabled)
    enable_power(enable_ && ((wd_timeout_ms_ == 0) || (now - last_hs_time_ < wd_timeout_ms_)));
}

void PowerControl::enable_power(bool enable)
{
    if (enable) {
        digitalWriteFast(control_pin_, control_pin_active_hi_? HIGH: LOW);
    } else {
        digitalWriteFast(control_pin_, control_pin_active_hi_? LOW: HIGH);
    }        
}

// static
void PowerControl::timer_handler()
{
    if (!power_control_) {
        return;
    }
    power_control_->update_state();
}

void PowerControl::log_debug_info()
{
    Logger::log_message(Logger::LogLevel::Info, "PowerControl, dbg_timer_cnt: %u, is_enabled: %d, hs_cnt: %d",
                        dbg_timer_cnt_, is_enabled(), dbg_hs_cnt_);
}
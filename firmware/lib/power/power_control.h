#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include "Arduino.h"

// Note! - only one instance is currently supported

class PowerControl
{
public:
    PowerControl(int control_pin, bool active_hi, int status_pin, bool status_pin_active_hi);
    ~PowerControl();

    static void timer_handler();

    bool configure(uint32_t wd_timeout_ms);
    void handshake();

    void enable(bool enable);
    bool is_enabled() const;

    void log_debug_info();

private:
    void update_state();
    void enable_power(bool enable);

    static inline PowerControl *power_control_{nullptr};

    int control_pin_{-1};
    bool control_pin_active_hi_{false};

    int status_pin_{-1};
    bool status_pin_active_hi_{false};

    volatile uint32_t wd_timeout_ms_{0};
    volatile bool enable_{false};           // True if power should be enabled
    volatile bool enabled_{false};          // True if power currently enabled (may be false if external safety switches de-asserted)

    IntervalTimer update_timer_;
    uint32_t update_timer_period_us_{0};

    volatile uint32_t last_hs_time_{0};

    volatile uint32_t dbg_timer_cnt_{0};
    uint32_t dbg_hs_cnt_{0};
};

#endif // POWER_CONTROL_H

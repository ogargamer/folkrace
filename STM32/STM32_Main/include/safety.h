#ifndef SAFETY_H
#define SAFETY_H

#include <Arduino.h>

class SafetyManager {
public:
    SafetyManager();

    void init();
    
    // Updates watchdog state, decelerates targets if watchdog is triggered,
    // and flashes the Status LED based on system state.
    void update(uint32_t last_packet_time, float &m1_target, float &m2_target,
                float &m3_target, float &m4_target, bool has_sensor_fault);

    bool isWatchdogActive() const { return _watchdog_active; }

private:
    bool _watchdog_active;
    uint32_t _last_led_toggle;
    bool _led_state;
    
    uint32_t _last_decel_update;

    void update_led(bool has_fault);
    void decelerate_targets(float &m1, float &m2, float &m3, float &m4);
};

#endif // SAFETY_H

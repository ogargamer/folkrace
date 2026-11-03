#!/usr/bin/env python3
"""
fan_control.py
Hardware PWM Fan Controller for Raspberry Pi 4 (BCM2711 PWM0 on GPIO 12 / Pin 32).
Designed for 4-wire PWM coolers (e.g. Raspberry Pi 5 Active Cooler / THRML 30-AC).

Features:
- Exact 25 kHz hardware PWM (40,000 ns period via BCM2711 PWM0).
- Silent base speed: fan always spins at silent minimum (30% PWM) when cool (<= 45°C).
- Temperature filtering & deadband: filters sensor noise, prevents RPM hunting.
- Asymmetric slew-rate limiting:
    * Rapid ramp-up (up to +8%/sec normally, instant if >= 78°C) to prevent overheating.
    * Ultra-gentle ramp-down (max -1.5%/sec) to respect heatsink thermal inertia and
      eliminate annoying acoustic rev-up / rev-down cycles.
- Thermal throttle protection: reaches 100% full speed at >= 80°C.
"""

import os
import signal
import sys
import time
from pathlib import Path

# Hardware PWM Configuration (GPIO 12, PWM0)
PWM_CHIP_DIR = Path("/sys/class/pwm/pwmchip0")
PWM0_DIR = PWM_CHIP_DIR / "pwm0"
PERIOD_NS = 40000      # 40,000 ns = 25 kHz (standard 4-wire fan frequency)

# Temperature Range (°C)
TEMP_MIN = 45.0        # Temp below which fan stays at MIN_SPEED
TEMP_THROTTLE = 81.0   # Thermal throttle threshold where fan reaches 100%
TEMP_EMERGENCY = 81.0  # At or above 81°C, switch to emergency ramp-up rate

# Speed Limits (Duty Fraction 0.0 - 1.0)
MIN_SPEED = 0.40       # Minimum duty cycle (40% baseline airflow)
MAX_SPEED = 1.00       # Maximum duty cycle (100% cooling)

# Hysteresis & Slew Rate Controls
POLL_INTERVAL = 1.0                # Seconds between control loop iterations
TEMP_FILTER_ALPHA = 0.10           # Heavy low-pass filter weight (10s time constant)
TEMP_DEADBAND = 1.0                # 1.0°C deadband: ignore tiny temp oscillations

RAMP_UP_RATE_PER_SEC = 0.04        # 4% duty cycle per second (regular ramp-up)
RAMP_UP_EMERGENCY_RATE = 0.08      # 8% duty cycle per second (emergency ramp-up at >= 81°C)
RAMP_DOWN_RATE_PER_SEC = 0.010     # 1% duty cycle per second (ramp-down)
DUTY_STEP_RESOLUTION = 0.005       # 0.5% duty cycle step quantization

THERMAL_ZONE = "/sys/class/thermal/thermal_zone0/temp"

def init_hardware_pwm() -> bool:
    # Wait for PWM chip directory to exist (up to 15s during early boot)
    for _ in range(30):
        if (PWM_CHIP_DIR / "export").exists():
            break
        time.sleep(0.5)

    if not (PWM_CHIP_DIR / "export").exists():
        print(f"[fan_control] Error: {PWM_CHIP_DIR}/export not found!")
        return False

    # Export pwm0 if not yet exported
    if not PWM0_DIR.exists():
        try:
            with open(PWM_CHIP_DIR / "export", "w") as f:
                f.write("0\n")
        except Exception:
            pass

    # Wait for sysfs files to appear after export
    for _ in range(20):
        if (PWM0_DIR / "period").exists() and (PWM0_DIR / "duty_cycle").exists():
            break
        time.sleep(0.2)

    # Configure period and enable with retries
    for attempt in range(5):
        try:
            # Disable first so period changes cleanly
            if (PWM0_DIR / "enable").exists():
                with open(PWM0_DIR / "enable", "w") as f:
                    f.write("0\n")
            # Duty cycle must be <= period, reset to 0 first
            with open(PWM0_DIR / "duty_cycle", "w") as f:
                f.write("0\n")
            with open(PWM0_DIR / "period", "w") as f:
                f.write(f"{PERIOD_NS}\n")
            with open(PWM0_DIR / "enable", "w") as f:
                f.write("1\n")

            # Verify period was set correctly
            read_period = int((PWM0_DIR / "period").read_text().strip())
            if read_period == PERIOD_NS:
                print(f"[fan_control] PWM0 initialized successfully ({PERIOD_NS} ns period)")
                return True
        except Exception as e:
            print(f"[fan_control] Retrying PWM0 setup (attempt {attempt+1}/5): {e}")
            time.sleep(0.5)

    print("[fan_control] Failed to configure PWM0 period after 5 attempts")
    return False

def set_fan_duty(duty_fraction: float):
    duty_fraction = max(MIN_SPEED, min(MAX_SPEED, duty_fraction))
    duty_ns = int(round(duty_fraction * PERIOD_NS))
    try:
        with open(PWM0_DIR / "duty_cycle", "w") as f:
            f.write(f"{duty_ns}\n")
    except Exception as e:
        print(f"[fan_control] Error updating duty_cycle: {e}, reinitializing PWM...")
        if init_hardware_pwm():
            try:
                with open(PWM0_DIR / "duty_cycle", "w") as f:
                    f.write(f"{duty_ns}\n")
            except Exception:
                pass

def get_raw_cpu_temp() -> float:
    try:
        with open(THERMAL_ZONE, "r") as f:
            return float(f.read().strip()) / 1000.0
    except Exception:
        return 80.0  # Fallback to high temp for safety if read fails

def calculate_target_duty(temp: float) -> float:
    if temp <= TEMP_MIN:
        return MIN_SPEED
    if temp >= TEMP_THROTTLE:
        return MAX_SPEED

    # Progressive curve (power 1.35 keeps fan extra quiet in 45-65°C range)
    fraction = (temp - TEMP_MIN) / (TEMP_THROTTLE - TEMP_MIN)
    speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * (fraction ** 1.35)
    return max(MIN_SPEED, min(MAX_SPEED, speed))

def main():
    print(f"[fan_control] Starting 25 kHz Hardware PWM0 Controller on GPIO 12 (Pin 32)")
    print(f"[fan_control] Baseline: {int(MIN_SPEED*100)}% (<= {TEMP_MIN}°C) -> 100% (>= {TEMP_THROTTLE}°C)")
    print(f"[fan_control] Hysteresis: deadband {TEMP_DEADBAND}°C (LPF alpha: {TEMP_FILTER_ALPHA}) | ramp-up +{int(RAMP_UP_RATE_PER_SEC*100)}%/s (emergency: +{int(RAMP_UP_EMERGENCY_RATE*100)}%/s @ >={TEMP_EMERGENCY}°C) | ramp-down -{RAMP_DOWN_RATE_PER_SEC*100:.1f}%/s")

    init_hardware_pwm()

    def handle_exit(signum, frame):
        print("[fan_control] Shutting down, leaving fan at safe 50%...")
        set_fan_duty(0.50)
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)

    # Initial start spin-up kick (0.5s at 100% to overcome static friction)
    set_fan_duty(1.0)
    time.sleep(0.5)

    raw_temp = get_raw_cpu_temp()
    filtered_temp = raw_temp
    last_eval_temp = filtered_temp
    current_duty = calculate_target_duty(filtered_temp)
    set_fan_duty(current_duty)

    last_loop_time = time.time()
    last_logged_time = 0.0
    last_logged_duty = current_duty

    print(f"[fan_control] Running. Initial CPU temp: {filtered_temp:.1f}°C, Duty: {int(current_duty*100)}%")

    while True:
        time.sleep(POLL_INTERVAL)
        now = time.time()
        dt = max(0.1, min(2.0, now - last_loop_time))
        last_loop_time = now

        raw_temp = get_raw_cpu_temp()
        filtered_temp = (TEMP_FILTER_ALPHA * raw_temp) + ((1.0 - TEMP_FILTER_ALPHA) * filtered_temp)

        # Apply temperature deadband: only update target temperature if change exceeds deadband
        if abs(filtered_temp - last_eval_temp) >= TEMP_DEADBAND:
            last_eval_temp = filtered_temp

        target_duty = calculate_target_duty(last_eval_temp)

        # Asymmetric Slew-Rate Limiting (Ramp Up vs Ramp Down)
        if target_duty > current_duty:
            if filtered_temp >= TEMP_EMERGENCY:
                # Emergency ramp-up at >= 80°C (10%/s)
                max_rise = RAMP_UP_EMERGENCY_RATE * dt
            else:
                # Regular ramp-up (5%/s)
                max_rise = RAMP_UP_RATE_PER_SEC * dt
            current_duty = min(target_duty, current_duty + max_rise)
        elif target_duty < current_duty:
            # Gentle, silent ramp-down respecting heatsink thermal storage
            max_fall = RAMP_DOWN_RATE_PER_SEC * dt
            current_duty = max(target_duty, current_duty - max_fall)

        # Quantize step to prevent micro-fluctuations in sound pitch
        quantized_duty = round(current_duty / DUTY_STEP_RESOLUTION) * DUTY_STEP_RESOLUTION
        quantized_duty = max(MIN_SPEED, min(MAX_SPEED, quantized_duty))
        set_fan_duty(quantized_duty)

        # Periodic or event-driven logging
        if abs(quantized_duty - last_logged_duty) >= 0.04 or (now - last_logged_time) >= 60.0:
            print(f"[fan_control] CPU: {raw_temp:.1f}°C (filt: {filtered_temp:.1f}°C) | Fan: {int(quantized_duty*100)}% ({int(quantized_duty * PERIOD_NS)} ns)")
            last_logged_duty = quantized_duty
            last_logged_time = now

if __name__ == "__main__":
    main()

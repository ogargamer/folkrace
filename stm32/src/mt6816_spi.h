#ifndef MT6816_SPI_H
#define MT6816_SPI_H

#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include "config_pins.h"

// ============================================================================
// MT6816 HIGH-PERFORMANCE 14-BIT MAGNETIC ANGLE SENSOR DRIVER
// ============================================================================
// Single-Read Hardware SPI Caching for 20.0 kHz Real-Time SimpleFOC Commutation.
// - Timed single-read per tick: loopFOC() touches ZERO SPI bus cycles.
// - Direct Port A/B BSRR atomic register toggling for CS assertion (< 6 ns).
// - 128-point piecewise linear interpolation LUT for sub-0.05 deg linearity.
// ============================================================================

class MT6816Sensor : public Sensor {
public:
    int cs_pin;
    GPIO_TypeDef *cs_port;
    uint32_t cs_set_mask;
    uint32_t cs_reset_mask;

    // Diagnostics & Tracking
    volatile uint32_t total_reads;
    volatile uint32_t parity_errors;
    volatile uint32_t parity_retries;
    volatile uint32_t mag_faults;
    volatile uint32_t consecutive_parity_errors;
    volatile bool is_connected;
    volatile uint16_t slow_poll_counter;
    volatile bool last_no_mag;
    volatile bool last_parity_ok;
    volatile uint8_t last_raw_h;
    volatile uint8_t last_raw_l;
    volatile uint16_t last_raw_14bit;
    float zero_offset_deg;

    // 128-Point Stepped Piecewise Linear Interpolation LUT
    float lut_errors[128];
    bool is_lut_active;

    SPISettings spi_settings;

    MT6816Sensor(int pin = PIN_M1_CS)
        : cs_pin(pin), cs_port(GPIOB), cs_set_mask(1 << 2), cs_reset_mask((1 << 2) << 16),
          total_reads(0), parity_errors(0), parity_retries(0), mag_faults(0),
          consecutive_parity_errors(0), is_connected(true), slow_poll_counter(0),
          last_no_mag(false), last_parity_ok(true),
          last_raw_h(0), last_raw_l(0), last_raw_14bit(0),
          zero_offset_deg(0.0f), is_lut_active(false),
          spi_settings(SPI_SPEED_HZ, MSBFIRST, SPI_MODE3) {
        for (int i = 0; i < 128; i++) {
            lut_errors[i] = 0.0f;
        }
    }

    void setupCSPin(int pin) {
        cs_pin = pin;
        pinMode(cs_pin, OUTPUT);
        digitalWrite(cs_pin, HIGH);

        if (pin == PB2) {
            cs_port = GPIOB;
            cs_set_mask = (1 << 2);
        } else if (pin == PB3) {
            cs_port = GPIOB;
            cs_set_mask = (1 << 3);
        } else if (pin == PA2) {
            cs_port = GPIOA;
            cs_set_mask = (1 << 2);
        } else if (pin == PA3) {
            cs_port = GPIOA;
            cs_set_mask = (1 << 3);
        } else {
            cs_port = GPIOB;
            cs_set_mask = (1 << 3);
        }
        cs_reset_mask = cs_set_mask << 16;
    }

    inline void csLow() __attribute__((always_inline)) {
        cs_port->BSRR = cs_reset_mask;
    }

    inline void csHigh() __attribute__((always_inline)) {
        cs_port->BSRR = cs_set_mask;
    }

    static inline void delayNanos(uint32_t ns) __attribute__((always_inline)) {
        // At 170 MHz, 1 cycle = 5.88 ns -> (ns * 17) / 100 cycles
        uint32_t cycles = (ns * 17) / 100;
        uint32_t start = DWT->CYCCNT;
        while ((DWT->CYCCNT - start) < cycles) {
            __NOP();
        }
    }

    static inline bool parityCheck(uint16_t data) {
        // Even parity over 16 bits (count % 2 == 0)
        data ^= data >> 8;
        data ^= data >> 4;
        data ^= data >> 2;
        data ^= data >> 1;
        return (data & 1) == 0;
    }

    void init() override {
        setupCSPin(cs_pin);

        // Configure shared SPI1 peripheral (matching ESP32 implementation)
        SPI.setSCLK(PIN_SPI1_SCK);
        SPI.setMISO(PIN_SPI1_MISO);
        SPI.setMOSI(PIN_SPI1_MOSI);
        SPI.begin();

        // Enable pull-up directly on PB4 (MISO) without destroying AF5 configuration
        GPIOB->PUPDR = (GPIOB->PUPDR & ~(3 << (4 * 2))) | (1 << (4 * 2));

        // MT6816 initial read
        readHardwareSPI();

        // Initialize SimpleFOC Sensor base class
        this->Sensor::init();
    }

    // ------------------------------------------------------------------------
    // Hardware SPI Single-Read Engine (Executed strictly ONCE per 50 us tick)
    // ------------------------------------------------------------------------
    inline void readHardwareSPI() __attribute__((always_inline)) {
        // If sensor is disconnected, throttle to 10 Hz (every 2,000 ticks) so it doesn't pollute shared SPI bus
        if (!is_connected) {
            if (++slow_poll_counter < 2000) {
                return;
            }
            slow_poll_counter = 0;
        }

        SPI.beginTransaction(spi_settings);

        for (int retry = 0; retry < 2; retry++) {
            total_reads++;

            // 1. Read High Byte (Register 0x03)
            csLow();
            delayNanos(250); // t_CSS setup time (>= 150 ns)
            uint16_t rx_h = SPI.transfer16(0x8300);
            delayNanos(250); // t_CSH hold time (>= 150 ns)
            csHigh();

            delayNanos(250); // t_CS_HIGH deassertion hold time (>= 150 ns)

            // 2. Read Low Byte (Register 0x04)
            csLow();
            delayNanos(250); // t_CSS setup time (>= 150 ns)
            uint16_t rx_l = SPI.transfer16(0x8400);
            delayNanos(250); // t_CSH hold time (>= 150 ns) - ensures bit 0 (parity) is captured before CS rises!
            csHigh();

            uint8_t h = rx_h & 0xFF;
            uint8_t l = rx_l & 0xFF;

            uint16_t angle_data = ((uint16_t)h << 8) | l;
            uint16_t raw14 = ((uint16_t)h << 6) | ((l & 0xFC) >> 2);
            bool no_mag = (l & 0x02) != 0;
            bool parity_ok = parityCheck(angle_data);

            last_raw_h = h;
            last_raw_l = l;
            last_no_mag = no_mag;

            if (no_mag) {
                mag_faults++;
            }

            if (parity_ok) {
                last_raw_14bit = raw14;
                consecutive_parity_errors = 0;
                last_parity_ok = true;
                is_connected = true;
                if (retry > 0) {
                    parity_retries++;
                }
                break;
            } else {
                if (retry == 0) {
                    delayNanos(250);
                } else {
                    parity_errors++;
                    consecutive_parity_errors++;
                    if (consecutive_parity_errors >= 4) {
                        last_parity_ok = false;
                        if (consecutive_parity_errors >= 200 && no_mag) {
                            is_connected = false; // Mark disconnected to avoid floating line noise
                        }
                    }
                }
            }
        }

        SPI.endTransaction();
    }

    // ------------------------------------------------------------------------
    // Calibrated Angle Calculation (Sub-microsecond, Zero SPI reads)
    // ------------------------------------------------------------------------
    inline float getRawMechanicalAngle() const {
        return ((float)last_raw_14bit / 16384.0f) * _2PI;
    }

    inline float getCalibratedAngleRad() const {
        float raw_rad = getRawMechanicalAngle();

        if (!is_lut_active) {
            return raw_rad;
        }

        // 128-point Piecewise Linear Interpolation
        int seg = (last_raw_14bit >> 7) & 0x7F;       // 0 .. 127
        float frac = (float)(last_raw_14bit & 0x7F) / 128.0f;
        int next_seg = (seg + 1) & 0x7F;

        float err = lut_errors[seg] * (1.0f - frac) + lut_errors[next_seg] * frac;
        float angle = raw_rad - err;

        if (angle < 0.0f) angle += _2PI;
        else if (angle >= _2PI) angle -= _2PI;

        return angle;
    }

    // Standard SimpleFOC Sensor contract:
    // Called by Sensor::update() during motor.loopFOC(). Uses CACHED reading!
    float getSensorAngle() override {
        return getCalibratedAngleRad();
    }

    float getSingleTurnDeg() const {
        float deg = ((float)last_raw_14bit / 16384.0f) * 360.0f - zero_offset_deg;
        while (deg < 0.0f) deg += 360.0f;
        while (deg >= 360.0f) deg -= 360.0f;
        return deg;
    }

    void zeroCurrentPosition() {
        zero_offset_deg = ((float)last_raw_14bit / 16384.0f) * 360.0f;
        this->full_rotations = 0;
    }

    void setLUTLine(uint8_t line_idx, const float* values, uint8_t count) {
        if (line_idx < 8) {
            int start_idx = line_idx * 16;
            for (uint8_t i = 0; i < count && (start_idx + i) < 128; i++) {
                lut_errors[start_idx + i] = values[i];
            }
            if (line_idx == 7) {
                is_lut_active = true;
            }
        }
    }

    void clearLUT() {
        for (int i = 0; i < 128; i++) {
            lut_errors[i] = 0.0f;
        }
        is_lut_active = false;
    }
};

#endif // MT6816_SPI_H

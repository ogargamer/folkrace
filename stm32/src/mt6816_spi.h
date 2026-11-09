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
    uint32_t last_update_tick;
    float zero_offset_deg;
    volatile float cached_calibrated_angle;

    // 128-Point Stepped Piecewise Linear Interpolation LUT
    float lut_errors[128];
    bool is_lut_active;

    SPISettings spi_settings;
    SPIClass *spi_bus;
    int sck_pin;
    int miso_pin;
    int mosi_pin;

    MT6816Sensor(int pin = PIN_M1_CS, SPIClass *bus = &SPI, int sck = PIN_SPI1_SCK, int miso = PIN_SPI1_MISO, int mosi = PIN_SPI1_MOSI)
        : cs_pin(pin), spi_bus(bus), sck_pin(sck), miso_pin(miso), mosi_pin(mosi),
          cs_port(GPIOB), cs_set_mask(1 << 2), cs_reset_mask((1 << 2) << 16),
          total_reads(0), parity_errors(0), parity_retries(0), mag_faults(0),
          consecutive_parity_errors(0), is_connected(true), slow_poll_counter(0),
          last_no_mag(false), last_parity_ok(true),
          last_raw_h(0), last_raw_l(0), last_raw_14bit(0), last_update_tick(0xFFFFFFFF),
          zero_offset_deg(0.0f), cached_calibrated_angle(0.0f), is_lut_active(false),
          spi_settings(SPI_SPEED_HZ, MSBFIRST, SPI_MODE3) {
        setupCSPin(pin);
        for (int i = 0; i < 128; i++) {
            lut_errors[i] = 0.0f;
        }
    }

    void setSPIBus(SPIClass *bus, int sck, int miso, int mosi) {
        spi_bus = bus;
        sck_pin = sck;
        miso_pin = miso;
        mosi_pin = mosi;
    }

    void setupCSPin(int pin) {
        cs_pin = pin;
        pinMode(cs_pin, OUTPUT);
        digitalWrite(cs_pin, HIGH);

        if (pin == PA15) {
            cs_port = GPIOA;
            cs_set_mask = (1 << 15);
        } else if (pin == PB2) {
            cs_port = GPIOB;
            cs_set_mask = (1 << 2);
        } else if (pin == PB3) {
            cs_port = GPIOB;
            cs_set_mask = (1 << 3);
        } else if (pin == PB11) {
            cs_port = GPIOB;
            cs_set_mask = (1 << 11);
        } else if (pin == PA2) {
            cs_port = GPIOA;
            cs_set_mask = (1 << 2);
        } else if (pin == PA3) {
            cs_port = GPIOA;
            cs_set_mask = (1 << 3);
        } else {
            cs_port = digitalPinToPort(pin);
            cs_set_mask = digitalPinToBitMask(pin);
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

    void resetErrors() {
        parity_errors = 0;
        parity_retries = 0;
        mag_faults = 0;
        consecutive_parity_errors = 0;
        last_parity_ok = true;
        is_connected = true;
        slow_poll_counter = 0;
    }

    void init() override {
        setupCSPin(cs_pin);
        resetErrors();

        // Configure hardware SPI peripheral
        if (spi_bus != nullptr) {
            spi_bus->setSCLK(sck_pin);
            spi_bus->setMISO(miso_pin);
            spi_bus->setMOSI(mosi_pin);
            spi_bus->begin();

            // Enable pull-up directly on MISO without destroying AF5 configuration
            if (miso_pin == PB4) {
                GPIOB->PUPDR = (GPIOB->PUPDR & ~(3UL << (4 * 2))) | (1UL << (4 * 2));
            } else if (miso_pin == PB14) {
                GPIOB->PUPDR = (GPIOB->PUPDR & ~(3UL << (14 * 2))) | (1UL << (14 * 2));
            }
        }

        // MT6816 initial read
        readHardwareSPI();

        // Clear any glitch/noise gathered during power-on initialization
        resetErrors();

        // Initialize SimpleFOC Sensor base class
        this->Sensor::init();
        clearVelocityBuffer();
    }

    // ------------------------------------------------------------------------
    // Hardware SPI Single-Read Engine (Executed strictly ONCE per 50 us tick)
    // ------------------------------------------------------------------------
    inline void readHardwareSPI() __attribute__((always_inline)) {
        if (!spi_bus) return;

        // If sensor is disconnected, throttle to 10 Hz (every 2,000 ticks) so it doesn't poll constantly
        if (!is_connected) {
            if (++slow_poll_counter < 2000) {
                return;
            }
            slow_poll_counter = 0;
        }

        spi_bus->beginTransaction(spi_settings);

        total_reads++;

        // 1. Read High Byte (Register 0x03)
        csLow();
        delayNanos(180); // t_CSS setup time (>= 150 ns)
        uint16_t rx_h = spi_bus->transfer16(0x8300);
        delayNanos(180); // t_CSH hold time (>= 150 ns)
        csHigh();

        delayNanos(180); // t_CS_HIGH deassertion hold time (>= 150 ns)

        // 2. Read Low Byte (Register 0x04)
        csLow();
        delayNanos(180); // t_CSS setup time (>= 150 ns)
        uint16_t rx_l = spi_bus->transfer16(0x8400);
        delayNanos(180); // t_CSH hold time (>= 150 ns)
        csHigh();

        parseRawData(rx_h & 0xFF, rx_l & 0xFF);

        spi_bus->endTransaction();
    }

    inline void parseRawData(uint8_t h, uint8_t l) {
        last_raw_h = h;
        last_raw_l = l;

        uint16_t angle_data = ((uint16_t)h << 8) | l;
        uint16_t raw14 = ((uint16_t)h << 6) | ((l & 0xFC) >> 2);
        bool no_mag = (l & 0x02) != 0;
        bool parity_ok = parityCheck(angle_data);

        last_no_mag = no_mag;

        if (no_mag) {
            mag_faults++;
        }

        if (parity_ok) {
#if defined(INVERT_ENCODER_DIRECTION) && (INVERT_ENCODER_DIRECTION == 1)
            raw14 = (16384 - raw14) & 0x3FFF;
#endif
            last_raw_14bit = raw14;
            consecutive_parity_errors = 0;
            last_parity_ok = true;
            is_connected = true;
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
    // Called by Sensor::update() and motor.loopFOC(). Returns CACHED reading within the same tick!
    float getSensorAngle() override {
        extern volatile uint32_t foc_tick_count;
        if (last_update_tick == foc_tick_count) {
            return cached_calibrated_angle;
        }
        cached_calibrated_angle = getCalibratedAngleRad();
        return cached_calibrated_angle;
    }

    // ------------------------------------------------------------------------
    // DWT Velocity Window Tracker (Tunable 0.2 .. 6.0 ms)
    // ------------------------------------------------------------------------
    static constexpr uint8_t VEL_HIST_LEN = 128; // Power-of-2 ring buffer (up to 6.4 ms @ 20 kHz)
    float hist_angle[VEL_HIST_LEN] = {0};
    uint32_t hist_dwt[VEL_HIST_LEN] = {0};
    uint8_t hist_head = 0;
    uint8_t hist_count = 0;
    uint8_t window_samples = 40; // 40 samples = 2.0 ms default @ 20 kHz
    float window_ms = 2.0f;

    void setDWTWindowMs(float ms) {
        if (ms < 0.2f) ms = 0.2f;
        if (ms > 6.0f) ms = 6.0f;
        window_ms = ms;
        uint8_t samples = (uint8_t)(ms * 20.0f + 0.5f);
        if (samples < 2) samples = 2;
        if (samples > (VEL_HIST_LEN - 2)) samples = VEL_HIST_LEN - 2;
        window_samples = samples;
    }

    void clearVelocityBuffer() {
        hist_head = 0;
        hist_count = 0;
        this->velocity = 0.0f;
    }

    void update() override {
        // Prevent duplicate updates within the same 50 us FOC tick (e.g. called in focTimerISR then in loopFOC)
        extern volatile uint32_t foc_tick_count;
        if (last_update_tick == foc_tick_count) {
            return;
        }
        last_update_tick = foc_tick_count;

        // Calculate and cache calibrated angle ONCE per tick (eliminates duplicate 128-pt LUT evaluations)
        cached_calibrated_angle = getCalibratedAngleRad();
        float val = cached_calibrated_angle;
        float d_ang = val - angle_prev;
        if (fabsf(d_ang) > (0.8f * _2PI)) {
            full_rotations += (d_ang > 0.0f) ? -1 : 1;
        }
        angle_prev = val;

        // Push current RAW mechanical angle and DWT timestamp into ring buffer.
        // Using raw 14-bit mechanical angle eliminates 128-point piecewise-linear derivative discontinuities
        // (eliminating audible 2.56 kHz whine) while Sensor::update() retains calibrated LUT angle for FOC commutation!
        uint32_t now = DWT->CYCCNT;
        hist_angle[hist_head] = getRawMechanicalAngle();
        hist_dwt[hist_head] = now;
        hist_head = (hist_head + 1) & (VEL_HIST_LEN - 1);
        if (hist_count < VEL_HIST_LEN) {
            hist_count++;
        }
    }

    float getVelocity() override {
        if (hist_count < 2) {
            return 0.0f;
        }

        uint8_t target_span = window_samples;
        uint8_t span = (hist_count >= target_span) ? target_span : hist_count;
        uint8_t oldest = (hist_head - span) & (VEL_HIST_LEN - 1);
        uint8_t newest = (hist_head - 1) & (VEL_HIST_LEN - 1);

        uint32_t dt_cycles = hist_dwt[newest] - hist_dwt[oldest];
        if (dt_cycles == 0) {
            return this->velocity;
        }

        // Exact cycle-level duration in seconds (170 MHz = 170,000,000 cycles/sec)
        float dt = (float)dt_cycles * (1.0f / 170000000.0f);
        float d_angle = hist_angle[newest] - hist_angle[oldest];

        // Unwrap shortest angular path across [-PI, +PI]
        while (d_angle > (float)_PI) d_angle -= (float)_2PI;
        while (d_angle < -(float)_PI) d_angle += (float)_2PI;

        this->velocity = d_angle / dt;
        return this->velocity;
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
        clearVelocityBuffer();
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

// ----------------------------------------------------------------------------
// Hard Real-Time Parallel Dual-SPI Encoder Read (SPI1 & SPI2 Concurrent Burst)
// ----------------------------------------------------------------------------
// Simultaneously clocks out 16-bit register reads on SPI1 (Enc1) and SPI2 (Enc2).
// Direct STM32G4 hardware FIFO register access (< 3.6 us total for BOTH encoders).
// ----------------------------------------------------------------------------
static inline void readDualEncodersParallel(MT6816Sensor &enc1, MT6816Sensor &enc2) __attribute__((always_inline));

static inline void readDualEncodersParallel(MT6816Sensor &enc1, MT6816Sensor &enc2) {
    // 1. Drain any residual bytes in RX FIFOs
    while (SPI1->SR & SPI_SR_RXNE) { (void)*(__IO uint8_t *)&SPI1->DR; }
    while (SPI2->SR & SPI_SR_RXNE) { (void)*(__IO uint8_t *)&SPI2->DR; }

    // 2. Assert both Chip Selects
    enc1.csLow();
    enc2.csLow();
    MT6816Sensor::delayNanos(120); // t_CSS setup time (>= 100 ns)

    // 3. Simultaneously queue 0x8300 on SPI1 and SPI2 TX FIFOs
    while (!(SPI1->SR & SPI_SR_TXE));
    *(__IO uint8_t *)&SPI1->DR = 0x83;
    *(__IO uint8_t *)&SPI1->DR = 0x00;

    while (!(SPI2->SR & SPI_SR_TXE));
    *(__IO uint8_t *)&SPI2->DR = 0x83;
    *(__IO uint8_t *)&SPI2->DR = 0x00;

    // Both SPI1 and SPI2 hardware engines are now shifting 16 bits in parallel!

    // 4. Read Register 0x03 from SPI1
    uint32_t to = 0;
    while (!(SPI1->SR & SPI_SR_RXNE) && ++to < 1000);
    uint8_t d1 = *(__IO uint8_t *)&SPI1->DR;
    to = 0;
    while (!(SPI1->SR & SPI_SR_RXNE) && ++to < 1000);
    uint8_t h1 = *(__IO uint8_t *)&SPI1->DR;
    to = 0;
    while ((SPI1->SR & SPI_SR_BSY) && ++to < 1000);

    // 5. Read Register 0x03 from SPI2 (already finished in parallel)
    to = 0;
    while (!(SPI2->SR & SPI_SR_RXNE) && ++to < 1000);
    uint8_t d2 = *(__IO uint8_t *)&SPI2->DR;
    to = 0;
    while (!(SPI2->SR & SPI_SR_RXNE) && ++to < 1000);
    uint8_t h2 = *(__IO uint8_t *)&SPI2->DR;
    to = 0;
    while ((SPI2->SR & SPI_SR_BSY) && ++to < 1000);

    // 6. Deassert both CS lines
    MT6816Sensor::delayNanos(60);
    enc1.csHigh();
    enc2.csHigh();

    MT6816Sensor::delayNanos(150); // t_CS_HIGH hold time (>= 150 ns)

    // --- Register 0x04 (Low Byte + Parity + Status) ---

    // 7. Assert both Chip Selects
    enc1.csLow();
    enc2.csLow();
    MT6816Sensor::delayNanos(120);

    // 8. Simultaneously queue 0x8400 on SPI1 and SPI2 TX FIFOs
    while (!(SPI1->SR & SPI_SR_TXE));
    *(__IO uint8_t *)&SPI1->DR = 0x84;
    *(__IO uint8_t *)&SPI1->DR = 0x00;

    while (!(SPI2->SR & SPI_SR_TXE));
    *(__IO uint8_t *)&SPI2->DR = 0x84;
    *(__IO uint8_t *)&SPI2->DR = 0x00;

    // 9. Read Register 0x04 from SPI1
    to = 0;
    while (!(SPI1->SR & SPI_SR_RXNE) && ++to < 1000);
    uint8_t d3 = *(__IO uint8_t *)&SPI1->DR;
    to = 0;
    while (!(SPI1->SR & SPI_SR_RXNE) && ++to < 1000);
    uint8_t l1 = *(__IO uint8_t *)&SPI1->DR;
    to = 0;
    while ((SPI1->SR & SPI_SR_BSY) && ++to < 1000);

    // 10. Read Register 0x04 from SPI2
    to = 0;
    while (!(SPI2->SR & SPI_SR_RXNE) && ++to < 1000);
    uint8_t d4 = *(__IO uint8_t *)&SPI2->DR;
    to = 0;
    while (!(SPI2->SR & SPI_SR_RXNE) && ++to < 1000);
    uint8_t l2 = *(__IO uint8_t *)&SPI2->DR;
    to = 0;
    while ((SPI2->SR & SPI_SR_BSY) && ++to < 1000);

    // 11. Deassert both CS lines
    MT6816Sensor::delayNanos(60);
    enc1.csHigh();
    enc2.csHigh();

    // 12. Parse angles and parity
    enc1.total_reads++;
    enc2.total_reads++;
    enc1.parseRawData(h1, l1);
    enc2.parseRawData(h2, l2);
}

#endif // MT6816_SPI_H

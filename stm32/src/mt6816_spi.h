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
// - Supports dedicated or shared SPI bus instances (SPI1, SPI2, etc.).
// - Fast atomic Port BSRR pin register toggling for CS assertion (< 6 ns).
// - 128-point piecewise linear interpolation LUT for sub-0.05 deg linearity.
// ============================================================================

class MT6816Sensor : public Sensor {
public:
    int cs_pin;
    SPIClass *spi_bus;
    int sck_pin;
    int miso_pin;
    int mosi_pin;

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

    MT6816Sensor(int pin = PIN_M1_CS, SPIClass *bus = &SPI, int sck = PIN_SPI1_SCK, int miso = PIN_SPI1_MISO, int mosi = PIN_SPI1_MOSI)
        : cs_pin(pin), spi_bus(bus), sck_pin(sck), miso_pin(miso), mosi_pin(mosi),
          cs_port(GPIOB), cs_set_mask(1 << 2), cs_reset_mask((1 << 2) << 16),
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

        cs_port = digitalPinToPort(pin);
        cs_set_mask = digitalPinToBitMask(pin);
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

        if (spi_bus != nullptr) {
            spi_bus->setSCLK(sck_pin);
            spi_bus->setMISO(miso_pin);
            spi_bus->setMOSI(mosi_pin);
            spi_bus->begin();
        }

        // Enable pull-up directly on MISO pin to prevent floating line when CS is high
        GPIO_TypeDef *miso_port = digitalPinToPort(miso_pin);
        PinName pn = digitalPinToPinName(miso_pin);
        uint32_t pin_num = STM_PIN(pn);
        miso_port->PUPDR = (miso_port->PUPDR & ~(3 << (pin_num * 2))) | (1 << (pin_num * 2));

        // MT6816 initial read
        readHardwareSPI();

        // Initialize SimpleFOC Sensor base class
        this->Sensor::init();
    }

    // ------------------------------------------------------------------------
    // Hardware SPI Burst-Read Engine (Executed strictly ONCE per 50 us tick)
    // ------------------------------------------------------------------------
    // One CS assertion per sample: 0x83,0x00,0x84,0x00 clocked back-to-back so
    // the angle high/low bytes come from the same MT6816 sample (no tearing).
    // The Arduino core configures the SPI peripheral for 8-bit frames, so the
    // burst is performed with byte-wide DR accesses.
    SPI_TypeDef *spi_reg = nullptr;

    static inline void spiFlush(SPI_TypeDef *s) __attribute__((always_inline)) {
        while (s->SR & SPI_SR_RXNE) { (void)*(volatile uint8_t *)&s->DR; }
    }
    static inline void spiPut(SPI_TypeDef *s, uint8_t b) __attribute__((always_inline)) {
        *(volatile uint8_t *)&s->DR = b;
    }
    static inline uint8_t spiGet(SPI_TypeDef *s) __attribute__((always_inline)) {
        while (!(s->SR & SPI_SR_RXNE)) {}
        return *(volatile uint8_t *)&s->DR;
    }

    bool shouldPoll() {
        // Disconnected sensors are throttled to 10 Hz (every 2,000 ticks)
        if (!is_connected) {
            if (++slow_poll_counter < 2000) return false;
            slow_poll_counter = 0;
        }
        return true;
    }

    // Validates one sample. Returns true when the sample is accepted (or final failure).
    inline bool processSample(uint8_t h, uint8_t l, int retry) __attribute__((always_inline)) {
        uint16_t angle_data = ((uint16_t)h << 8) | l;
        uint16_t raw14 = ((uint16_t)h << 6) | ((l & 0xFC) >> 2);
        bool no_mag = (l & 0x02) != 0;
        // Reject floating/shorted MISO (0xFFFF / 0x0000 both pass even parity)
        bool parity_ok = (angle_data != 0xFFFF && angle_data != 0x0000) && parityCheck(angle_data);

        total_reads++;
        last_raw_h = h;
        last_raw_l = l;
        last_no_mag = no_mag;
        if (no_mag) mag_faults++;

        if (parity_ok) {
            last_raw_14bit = raw14;
            consecutive_parity_errors = 0;
            last_parity_ok = true;
            is_connected = true;
            if (retry > 0) parity_retries++;
            return true;
        }
        if (retry == 0) return false;

        parity_errors++;
        consecutive_parity_errors++;
        if (consecutive_parity_errors >= 4) {
            last_parity_ok = false;
            if (consecutive_parity_errors >= 200 && no_mag) {
                is_connected = false; // Mark disconnected to avoid floating line noise
            }
        }
        return true;
    }

    // Single-sensor 32-bit burst (CS held low for the whole frame, BSY-safe release)
    inline void burstRead(uint8_t &h, uint8_t &l) __attribute__((always_inline)) {
        SPI_TypeDef *s = spi_reg;
        csLow();
        delayNanos(100); // t_CSS setup time
        spiFlush(s);
        spiPut(s, 0x83); spiGet(s);
        spiPut(s, 0x00); h = spiGet(s);
        spiPut(s, 0x84); spiGet(s);
        spiPut(s, 0x00); l = spiGet(s);
        while (s->SR & SPI_SR_BSY) {}  // last clock edge fully done before CS rises
        delayNanos(100); // t_CSH hold time
        csHigh();
    }

    inline void readHardwareSPI() __attribute__((always_inline)) {
        if (shouldPoll()) readPolled();
    }

    // Interleaved read of two sensors on independent SPI peripherals (e.g. SPI1 + SPI2).
    // Both shift registers clock simultaneously, halving SPI dead time in the ISR.
    static inline void readDualParallel(MT6816Sensor &a, MT6816Sensor &b) {
        bool pa = a.shouldPoll();
        bool pb = b.shouldPoll();
        if (!(pa && pb) || a.spi_reg == b.spi_reg) {
            // Fall back to independent reads (reset throttle counters already consumed above)
            if (pa) a.readPolled();
            if (pb) b.readPolled();
            return;
        }

        a.spi_bus->beginTransaction(a.spi_settings);
        b.spi_bus->beginTransaction(b.spi_settings);
        if (a.spi_reg == nullptr) a.spi_reg = a.spi_bus->getHandle()->Instance;
        if (b.spi_reg == nullptr) b.spi_reg = b.spi_bus->getHandle()->Instance;
        SPI_TypeDef *sa = a.spi_reg;
        SPI_TypeDef *sb = b.spi_reg;

        a.csLow();
        b.csLow();
        delayNanos(100); // t_CSS setup time
        spiFlush(sa);
        spiFlush(sb);

        static const uint8_t tx[4] = {0x83, 0x00, 0x84, 0x00};
        uint8_t ra[4], rb[4];
        for (int i = 0; i < 4; i++) {
            spiPut(sa, tx[i]);
            spiPut(sb, tx[i]);
            ra[i] = spiGet(sa);
            rb[i] = spiGet(sb);
        }
        while ((sa->SR & SPI_SR_BSY) || (sb->SR & SPI_SR_BSY)) {}
        delayNanos(100); // t_CSH hold time
        a.csHigh();
        b.csHigh();

        a.spi_bus->endTransaction();
        b.spi_bus->endTransaction();

        // Failed samples get one sequential retry (rare)
        if (!a.processSample(ra[1], ra[3], 0)) a.retryOnce();
        if (!b.processSample(rb[1], rb[3], 0)) b.retryOnce();
    }

    // Full single-sensor read without throttle check
    void readPolled() {
        spi_bus->beginTransaction(spi_settings);
        if (spi_reg == nullptr) spi_reg = spi_bus->getHandle()->Instance;
        for (int retry = 0; retry < 2; retry++) {
            uint8_t h, l;
            burstRead(h, l);
            if (processSample(h, l, retry)) break;
            delayNanos(150);
        }
        spi_bus->endTransaction();
    }

    void retryOnce() {
        delayNanos(150);
        spi_bus->beginTransaction(spi_settings);
        uint8_t h, l;
        burstRead(h, l);
        processSample(h, l, 1);
        spi_bus->endTransaction();
    }
    // ------------------------------------------------------------------------
    // Calibrated Angle Calculation (Sub-microsecond, Zero SPI reads)
    // ------------------------------------------------------------------------
    inline float getRawMechanicalAngle() const {
        return ((float)last_raw_14bit / 16384.0f) * _2PI;
    }

    inline float getCalibratedAngleRad() const {
        return calibratedAngleFromRaw14(last_raw_14bit);
    }

    inline float calibratedAngleFromRaw14(uint16_t raw14) const {
        float raw_rad = ((float)raw14 / 16384.0f) * _2PI;

        if (!is_lut_active) {
            return raw_rad;
        }

        // 128-point Piecewise Linear Interpolation
        int seg = (raw14 >> 7) & 0x7F;       // 0 .. 127
        float frac = (float)(raw14 & 0x7F) / 128.0f;
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
        float deg = getCalibratedAngleRad() * (360.0f / _2PI) - zero_offset_deg;
        while (deg < 0.0f) deg += 360.0f;
        while (deg >= 360.0f) deg -= 360.0f;
        return deg;
    }

    void zeroCurrentPosition() {
        zero_offset_deg = getCalibratedAngleRad() * (360.0f / _2PI);
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

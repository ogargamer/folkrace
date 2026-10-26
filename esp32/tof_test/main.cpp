#include <Arduino.h>
#include <SPI.h>
#include "esp_rom_gpio.h"
#include "driver/gpio.h"
#include "vl53l8cx_api.h"
#include "vl53l8cx_buffers.h"
#include "vl53l8cx_plugin_detection_thresholds.h"
#include "vl53l8cx_plugin_motion_indicator.h"
#include "vl53l8cx_plugin_xtalk.h"

// =========================================================================
// HARDWARE PIN DEFINITIONS (QUAD TOF ON SHARED HSPI BUS)
// =========================================================================

#define NUM_TOFS             4

// --- Shared SPI Bus Lines ---
#define TOF_SHARED_SCK_PIN   39
#define TOF_SHARED_MOSI_PIN  40

// --- ToF 1 ---
#define TOF1_CS_PIN          10
#define TOF1_MISO_PIN        41
#define TOF1_INT_PIN         5
#define TOF1_LPN_PIN         6

// --- ToF 2 ---
#define TOF2_CS_PIN          9
#define TOF2_MISO_PIN        13
#define TOF2_INT_PIN         4
#define TOF2_LPN_PIN         8

// --- ToF 3 ---
#define TOF3_CS_PIN          7
#define TOF3_MISO_PIN        16
#define TOF3_INT_PIN         14
#define TOF3_LPN_PIN         15

// --- ToF 4 ---
#define TOF4_CS_PIN          17
#define TOF4_MISO_PIN        38
#define TOF4_INT_PIN         18
#define TOF4_LPN_PIN         21

// ST-certified SPI clock frequency (400 kHz for breadboard jumper cables, 2 MHz for ranging)
#define SPI_INIT_SPEED_HZ    400000
#define SPI_RANGING_SPEED_HZ 2000000
#define CHUNK_SIZE           1024

// =========================================================================
// HARDWARE DRIVER STRUCTURES & CONTEXT
// =========================================================================

struct ToF_Device_Context {
    SPIClass *spi;
    int cs_pin;
    int lpn_pin;
    int int_pin;
    int sck_pin;
    int miso_pin;
    int mosi_pin;
    uint32_t speed_hz;
    const char *name;
    uint8_t id;
};

SPIClass HardwareSPI(HSPI);

ToF_Device_Context ctx_tofs[NUM_TOFS] = {
    { &HardwareSPI, TOF1_CS_PIN, TOF1_LPN_PIN, TOF1_INT_PIN, TOF_SHARED_SCK_PIN, TOF1_MISO_PIN, TOF_SHARED_MOSI_PIN, SPI_INIT_SPEED_HZ, "ToF 1 (CS:10, MISO:41)", 1 },
    { &HardwareSPI, TOF2_CS_PIN, TOF2_LPN_PIN, TOF2_INT_PIN, TOF_SHARED_SCK_PIN, TOF2_MISO_PIN, TOF_SHARED_MOSI_PIN, SPI_INIT_SPEED_HZ, "ToF 2 (CS:9, MISO:13)",  2 },
    { &HardwareSPI, TOF3_CS_PIN, TOF3_LPN_PIN, TOF3_INT_PIN, TOF_SHARED_SCK_PIN, TOF3_MISO_PIN, TOF_SHARED_MOSI_PIN, SPI_INIT_SPEED_HZ, "ToF 3 (CS:7, MISO:16)",  3 },
    { &HardwareSPI, TOF4_CS_PIN, TOF4_LPN_PIN, TOF4_INT_PIN, TOF_SHARED_SCK_PIN, TOF4_MISO_PIN, TOF_SHARED_MOSI_PIN, SPI_INIT_SPEED_HZ, "ToF 4 (CS:17, MISO:38)", 4 }
};

VL53L8CX_Configuration Dev[NUM_TOFS];
VL53L8CX_ResultsData Results[NUM_TOFS];
bool tof_active[NUM_TOFS] = {false, false, false, false};
volatile bool int_triggered[NUM_TOFS] = {false, false, false, false};

void IRAM_ATTR isr_tof1() { int_triggered[0] = true; }
void IRAM_ATTR isr_tof2() { int_triggered[1] = true; }
void IRAM_ATTR isr_tof3() { int_triggered[2] = true; }
void IRAM_ATTR isr_tof4() { int_triggered[3] = true; }

typedef void (*isr_t)();
const isr_t isrs[NUM_TOFS] = { isr_tof1, isr_tof2, isr_tof3, isr_tof4 };

unsigned long frame_count[NUM_TOFS] = {0, 0, 0, 0};
unsigned long last_frame_time_ms[NUM_TOFS] = {0, 0, 0, 0};
unsigned long last_poll_ms[NUM_TOFS] = {0, 0, 0, 0};

// =========================================================================
// RUNTIME SENSOR CONFIGURATION STATE (Default: 4x4 @ 60 Hz)
// =========================================================================

uint8_t current_resolution = VL53L8CX_RESOLUTION_4X4;
uint8_t current_frequency_hz = 60;
uint32_t current_integration_ms = 10;
uint8_t current_target_order = VL53L8CX_TARGET_ORDER_CLOSEST;
uint8_t current_sharpener = 1;
uint8_t current_nb_targets = 2;

// =========================================================================
// ON-CHIP 8-ZONE 90° RADAR FUSION ENGINE (BACKWARD COMPATIBILITY)
// =========================================================================

struct FusedRadarZone {
    uint16_t distance_mm;
    uint8_t  status;
    uint16_t spad_count;
    uint8_t  target_num;
    uint8_t  fallback_type;
    uint16_t t2_dist;
    uint16_t t3_dist;
};

struct FusedRadarState {
    FusedRadarZone zones[8];
    uint16_t min_distance_mm;
    uint32_t last_update_ms;
    unsigned long seq;
};

FusedRadarState fused_radar = {};

static uint8_t error_latch_count[8] = {0};
static uint16_t last_valid_dist[8] = {3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000};

// Telemetry stream configuration
bool stream_diag_matrix = true;   // Full 16-zone raw matrix telemetry for all 4 ToFs (DEFAULT)
bool stream_arc_mode = false;     // Fused 8-zone radar telemetry (only ToF 1 & 2)
bool print_ascii_enabled = false; // ASCII table monitor output (disabled during telemetry streaming)

static uint8_t tx_chunk_buffer[CHUNK_SIZE + 4];

// =========================================================================
// ST ULD HARDWARE PLATFORM ABSTRACTION LAYER
// =========================================================================

uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAddress, uint8_t value) {
    ToF_Device_Context *ctx = (ToF_Device_Context *)p_platform->handle;
    uint8_t buf[3];
    buf[0] = (uint8_t)(((RegisterAddress | 0x8000) >> 8) & 0xFF);
    buf[1] = (uint8_t)(RegisterAddress & 0xFF);
    buf[2] = value;

    ctx->spi->beginTransaction(SPISettings(ctx->speed_hz, MSBFIRST, SPI_MODE3));
    digitalWrite(ctx->cs_pin, LOW);
    delayMicroseconds(5);
    ctx->spi->transferBytes(buf, NULL, 3);
    delayMicroseconds(5);
    digitalWrite(ctx->cs_pin, HIGH);
    delayMicroseconds(5);
    ctx->spi->endTransaction();
    delayMicroseconds(10);
    return 0;
}

uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size) {
    ToF_Device_Context *ctx = (ToF_Device_Context *)p_platform->handle;
    uint32_t pos = 0;
    while (pos < size) {
        uint32_t chunk = (size - pos > CHUNK_SIZE) ? CHUNK_SIZE : (size - pos);
        uint16_t r = RegisterAddress + pos;

        tx_chunk_buffer[0] = (uint8_t)(((r | 0x8000) >> 8) & 0xFF);
        tx_chunk_buffer[1] = (uint8_t)(r & 0xFF);
        memcpy(&tx_chunk_buffer[2], &p_values[pos], chunk);

        ctx->spi->beginTransaction(SPISettings(ctx->speed_hz, MSBFIRST, SPI_MODE3));
        digitalWrite(ctx->cs_pin, LOW);
        delayMicroseconds(5);
        ctx->spi->transferBytes(tx_chunk_buffer, NULL, chunk + 2);
        delayMicroseconds(5);
        digitalWrite(ctx->cs_pin, HIGH);
        delayMicroseconds(5);
        ctx->spi->endTransaction();
        delayMicroseconds(10);

        pos += chunk;
    }
    return 0;
}

#include "soc/gpio_sig_map.h"

extern "C" {
    void spiAttachMISO(spi_t *spi, int8_t miso);
}

static inline void attach_miso_matrix(ToF_Device_Context *ctx) {
    int miso_pin = ctx->miso_pin;
    pinMode(miso_pin, INPUT_PULLUP);
    esp_rom_gpio_pad_select_gpio(miso_pin);

    spiAttachMISO(ctx->spi->bus(), miso_pin);

    #ifdef FSPIQ_IN_IDX
    esp_rom_gpio_connect_in_signal(miso_pin, FSPIQ_IN_IDX, false);
    #endif
    #ifdef FSPID_IN_IDX
    esp_rom_gpio_connect_in_signal(miso_pin, FSPID_IN_IDX, false);
    #endif
    #ifdef SPI3_Q_IN_IDX
    esp_rom_gpio_connect_in_signal(miso_pin, SPI3_Q_IN_IDX, false);
    #endif
    #ifdef SPI3_D_IN_IDX
    esp_rom_gpio_connect_in_signal(miso_pin, SPI3_D_IN_IDX, false);
    #endif
}

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAddress, uint8_t *p_value) {
    ToF_Device_Context *ctx = (ToF_Device_Context *)p_platform->handle;
    attach_miso_matrix(ctx);

    uint8_t tx[3];
    uint8_t rx[3] = {0};
    tx[0] = (uint8_t)(((RegisterAddress & ~0x8000) >> 8) & 0xFF);
    tx[1] = (uint8_t)(RegisterAddress & 0xFF);
    tx[2] = 0x00;

    ctx->spi->beginTransaction(SPISettings(ctx->speed_hz, MSBFIRST, SPI_MODE3));
    digitalWrite(ctx->cs_pin, LOW);
    delayMicroseconds(5);
    ctx->spi->transferBytes(tx, rx, 3);
    delayMicroseconds(5);
    digitalWrite(ctx->cs_pin, HIGH);
    delayMicroseconds(5);
    ctx->spi->endTransaction();

    *p_value = rx[2];
    return 0;
}

static uint8_t rx_chunk_buffer[CHUNK_SIZE + 4];

uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size) {
    ToF_Device_Context *ctx = (ToF_Device_Context *)p_platform->handle;
    attach_miso_matrix(ctx);

    uint32_t pos = 0;
    while (pos < size) {
        uint32_t chunk = (size - pos > CHUNK_SIZE) ? CHUNK_SIZE : (size - pos);
        uint16_t r = RegisterAddress + pos;

        tx_chunk_buffer[0] = (uint8_t)(((r & ~0x8000) >> 8) & 0xFF);
        tx_chunk_buffer[1] = (uint8_t)(r & 0xFF);
        memset(&tx_chunk_buffer[2], 0, chunk);

        ctx->spi->beginTransaction(SPISettings(ctx->speed_hz, MSBFIRST, SPI_MODE3));
        digitalWrite(ctx->cs_pin, LOW);
        delayMicroseconds(5);
        ctx->spi->transferBytes(tx_chunk_buffer, rx_chunk_buffer, chunk + 2);
        delayMicroseconds(5);
        digitalWrite(ctx->cs_pin, HIGH);
        delayMicroseconds(5);
        ctx->spi->endTransaction();

        memcpy(&p_values[pos], &rx_chunk_buffer[2], chunk);
        pos += chunk;
    }
    return 0;
}

void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size) {
    for (uint16_t i = 0; i < size; i += 4) {
        uint8_t tmp = buffer[i];
        buffer[i] = buffer[i + 3];
        buffer[i + 3] = tmp;
        tmp = buffer[i + 1];
        buffer[i + 1] = buffer[i + 2];
        buffer[i + 2] = tmp;
    }
}

uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t TimeMs) {
    delay(TimeMs);
    return 0;
}

// =========================================================================
// SENSOR INITIALIZATION HELPER
// =========================================================================

void run_deep_sensor_diagnostics(ToF_Device_Context *ctx) {
    Serial.printf("\n--- [DEEP HARDWARE DIAGNOSTIC] for %s ---\n", ctx->name);

    // 1. Check MISO with CS=HIGH vs CS=LOW
    pinMode(ctx->cs_pin, OUTPUT);
    digitalWrite(ctx->cs_pin, HIGH);
    pinMode(ctx->miso_pin, INPUT_PULLUP);
    int miso_cs_high = digitalRead(ctx->miso_pin);

    digitalWrite(ctx->cs_pin, LOW);
    delayMicroseconds(10);
    int miso_cs_low = digitalRead(ctx->miso_pin);
    digitalWrite(ctx->cs_pin, HIGH);

    Serial.printf("  1. CS Toggling: CS=HIGH -> MISO=%d | CS=LOW -> MISO=%d\n", miso_cs_high, miso_cs_low);

    // 2. Bit-Bang SPI Read of Device ID (Bank 0, Reg 0x0000)
    pinMode(ctx->sck_pin, OUTPUT);
    digitalWrite(ctx->sck_pin, HIGH);
    pinMode(ctx->mosi_pin, OUTPUT);
    digitalWrite(ctx->mosi_pin, HIGH);
    pinMode(ctx->lpn_pin, OUTPUT);
    digitalWrite(ctx->lpn_pin, HIGH);
    delay(20);

    auto bb_send_byte = [&](uint8_t b) {
        for (int i = 7; i >= 0; i--) {
            digitalWrite(ctx->sck_pin, LOW);
            digitalWrite(ctx->mosi_pin, (b >> i) & 1);
            delayMicroseconds(10);
            digitalWrite(ctx->sck_pin, HIGH);
            delayMicroseconds(10);
        }
    };

    auto bb_recv_byte = [&](char *bits_str) -> uint8_t {
        uint8_t res = 0;
        for (int i = 7; i >= 0; i--) {
            digitalWrite(ctx->sck_pin, LOW);
            delayMicroseconds(10);
            digitalWrite(ctx->sck_pin, HIGH);
            delayMicroseconds(4);
            int bit = digitalRead(ctx->miso_pin);
            bits_str[7 - i] = bit ? '1' : '0';
            res = (res << 1) | bit;
            delayMicroseconds(6);
        }
        bits_str[8] = '\0';
        return res;
    };

    // Bitbang Bank 0 select
    digitalWrite(ctx->cs_pin, LOW);
    delayMicroseconds(10);
    bb_send_byte(0xFF);
    bb_send_byte(0xFF);
    bb_send_byte(0x00);
    delayMicroseconds(10);
    digitalWrite(ctx->cs_pin, HIGH);
    delayMicroseconds(20);

    // Bitbang Read 0x0000
    char bits_dev[9] = {0};
    char bits_rev[9] = {0};
    digitalWrite(ctx->cs_pin, LOW);
    delayMicroseconds(10);
    bb_send_byte(0x00);
    bb_send_byte(0x00);
    uint8_t bb_dev = bb_recv_byte(bits_dev);
    uint8_t bb_rev = bb_recv_byte(bits_rev);
    delayMicroseconds(10);
    digitalWrite(ctx->cs_pin, HIGH);

    Serial.printf("  2. Bit-Bang SPI Mode 3 (50kHz): dev_id=0x%02X [bits: %s], rev_id=0x%02X [bits: %s]\n",
                  bb_dev, bits_dev, bb_rev, bits_rev);

    // 3. I2C ACK Detection Probe on (SCK, MOSI) and (SCK, MISO)
    auto i2c_ping = [&](int scl, int sda) -> bool {
        pinMode(scl, OUTPUT);
        pinMode(sda, OUTPUT);
        digitalWrite(scl, HIGH);
        digitalWrite(sda, HIGH);
        delayMicroseconds(10);
        digitalWrite(sda, LOW);
        delayMicroseconds(10);
        digitalWrite(scl, LOW);
        delayMicroseconds(10);

        uint8_t addr = 0x52; // 0x29 << 1
        for (int i = 7; i >= 0; i--) {
            digitalWrite(sda, (addr >> i) & 1);
            delayMicroseconds(10);
            digitalWrite(scl, HIGH);
            delayMicroseconds(10);
            digitalWrite(scl, LOW);
            delayMicroseconds(10);
        }
        pinMode(sda, INPUT_PULLUP);
        digitalWrite(scl, HIGH);
        delayMicroseconds(10);
        int ack = (digitalRead(sda) == LOW);
        digitalWrite(scl, LOW);
        delayMicroseconds(10);
        // Stop
        pinMode(sda, OUTPUT);
        digitalWrite(sda, LOW);
        delayMicroseconds(10);
        digitalWrite(scl, HIGH);
        delayMicroseconds(10);
        digitalWrite(sda, HIGH);
        return ack;
    };

    bool i2c_mosi = i2c_ping(ctx->sck_pin, ctx->mosi_pin);
    bool i2c_miso = i2c_ping(ctx->sck_pin, ctx->miso_pin);
    Serial.printf("  3. I2C Ping (0x29): on SCL/MOSI = %s | on SCL/MISO = %s\n",
                  i2c_mosi ? "ACK RECEIVED (Sensor in I2C Mode!)" : "No ACK",
                  i2c_miso ? "ACK RECEIVED (Sensor in I2C Mode!)" : "No ACK");

    // Restore hardware SPI pins
    HardwareSPI.begin(TOF_SHARED_SCK_PIN, ctx_tofs[0].miso_pin, TOF_SHARED_MOSI_PIN, -1);
    Serial.println(F("--------------------------------------------------\n"));
}

bool initialize_tof_sensor(VL53L8CX_Configuration *p_dev, ToF_Device_Context *ctx) {
    Serial.printf("\n>>> [STARTING] %s Init Sequence <<<\n", ctx->name);

    ctx->speed_hz = SPI_INIT_SPEED_HZ;

    // Explicitly configure HardwareSPI with this sensor's MISO pin
    HardwareSPI.begin(TOF_SHARED_SCK_PIN, ctx->miso_pin, TOF_SHARED_MOSI_PIN, -1);
    delay(10);

    memset(p_dev, 0, sizeof(VL53L8CX_Configuration));
    p_dev->platform.handle = (void *)ctx;
    p_dev->platform.address = 0x29;

    // 1. Physical pin state probe
    pinMode(ctx->miso_pin, INPUT_PULLUP);
    int miso_pu = digitalRead(ctx->miso_pin);
    pinMode(ctx->miso_pin, INPUT_PULLDOWN);
    int miso_pd = digitalRead(ctx->miso_pin);
    pinMode(ctx->miso_pin, INPUT_PULLUP);

    pinMode(ctx->int_pin, INPUT_PULLUP);
    int int_pu = digitalRead(ctx->int_pin);

    Serial.printf("[%s] Pin Probe: MISO %d [PU=%d, PD=%d] | INT %d [PU=%d] | CS=%d | LPN=%d\n",
                  ctx->name, ctx->miso_pin, miso_pu, miso_pd, ctx->int_pin, int_pu, ctx->cs_pin, ctx->lpn_pin);

    // 1. Hardware Reset via LPn with physical continuity test
    uint8_t dev_id = 0, rev_id = 0;
    uint8_t isAlive = 0;
    uint8_t status = 0;
    bool found_alive = false;

    // Diagnostic: Test LPn pin physical response
    digitalWrite(ctx->lpn_pin, LOW);
    delay(50);
    uint8_t r06_low = 0, dev_id_low = 0;
    VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);
    VL53L8CX_RdByte(&(p_dev->platform), 0x0000, &dev_id_low);
    VL53L8CX_RdByte(&(p_dev->platform), 0x0006, &r06_low);

    digitalWrite(ctx->lpn_pin, HIGH);
    delay(100);
    uint8_t r06_high = 0, dev_id_high = 0;
    VL53L8CX_RdByte(&(p_dev->platform), 0x0000, &dev_id_high);
    VL53L8CX_RdByte(&(p_dev->platform), 0x0006, &r06_high);

    Serial.printf("[%s] LPn Pin %d Test: [LPn=LOW -> dev_id=0x%02X, reg06=0x%02X] | [LPn=HIGH -> dev_id=0x%02X, reg06=0x%02X]\n",
                  ctx->name, ctx->lpn_pin, dev_id_low, r06_low, dev_id_high, r06_high);

    for (int attempt = 1; attempt <= 3; attempt++) {
        digitalWrite(ctx->lpn_pin, LOW);
        delay(100);
        digitalWrite(ctx->lpn_pin, HIGH);
        delay(100);

        VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);
        VL53L8CX_RdByte(&(p_dev->platform), 0, &dev_id);
        VL53L8CX_RdByte(&(p_dev->platform), 1, &rev_id);

        status = vl53l8cx_is_alive(p_dev, &isAlive);

        if ((isAlive && dev_id == 0xF0) || dev_id == 0xF0 || dev_id_high == 0xF0) {
            found_alive = true;
            Serial.printf("[%s] Sensor responded with dev_id=0xF0! Proceeding with init...\n", ctx->name);
            break;
        }
        delay(30);
    }

    if (!found_alive) {
        Serial.printf("[ERROR] %s did not report alive! (dev_id=0x%02X, rev_id=0x%02X, isAlive=%u)\n", 
                      ctx->name, dev_id, rev_id, isAlive);
        run_deep_sensor_diagnostics(ctx);
        return false;
    }

    // 2. True MOSI Bank-Switch Verification & Pre-Init MCU Status
    Serial.printf("[%s] Verifying MOSI via Memory Bank Switching...\n", ctx->name);
    uint8_t b0_id = 0, b0_rev = 0, b2_id = 0, b2_rev = 0, b_read = 0;

    VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);
    VL53L8CX_RdByte(&(p_dev->platform), 0x0000, &b0_id);
    VL53L8CX_RdByte(&(p_dev->platform), 0x0001, &b0_rev);

    VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x02);
    VL53L8CX_RdByte(&(p_dev->platform), 0x7fff, &b_read);
    VL53L8CX_RdByte(&(p_dev->platform), 0x0000, &b2_id);
    VL53L8CX_RdByte(&(p_dev->platform), 0x0001, &b2_rev);

    // Switch back to Bank 0 to read GO2 MCU Status (reg 0x0006 lives in Bank 0!)
    VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);
    uint8_t r06_pre = 0, r07_pre = 0;
    VL53L8CX_RdByte(&(p_dev->platform), 0x0006, &r06_pre);
    VL53L8CX_RdByte(&(p_dev->platform), 0x0007, &r07_pre);
    Serial.printf("[%s] Bank 0: [0x%02X 0x%02X] | Wr 0x7fff=0x02 -> Rd 0x7fff=0x%02X | Bank 2: [0x%02X 0x%02X]\n", 
                  ctx->name, b0_id, b0_rev, b_read, b2_id, b2_rev);
    Serial.printf("[%s] Pre-Init Bank 0 Reg 0x06=0x%02X (MCU Boot Flag), Reg 0x07=0x%02X\n",
                  ctx->name, r06_pre, r07_pre);

    // 3. Upload 84KB Firmware Blob
    Serial.printf("[%s] Uploading ST Official 84KB Firmware via Burst SPI @ %lu Hz...\n", ctx->name, ctx->speed_hz);
    unsigned long t0 = millis();
    status = vl53l8cx_init(p_dev);
    Serial.printf("[%s] vl53l8cx_init status = %u (upload took %lu ms)\n", ctx->name, status, millis() - t0);

    if (status != 0) {
        Serial.printf("[WARN] %s Init returned %u, retrying with reset...\n", ctx->name, status);
        digitalWrite(ctx->lpn_pin, LOW);
        delay(25);
        digitalWrite(ctx->lpn_pin, HIGH);
        delay(60);
        status = vl53l8cx_init(p_dev);
        Serial.printf("[%s] Retry vl53l8cx_init status = %u\n", ctx->name, status);
        if (status != 0) {
            Serial.printf("[ERROR] %s Firmware upload failed! status = %u\n", ctx->name, status);
            return false;
        }
    }

    // Switch to Ranging Speed
    ctx->speed_hz = SPI_RANGING_SPEED_HZ;

    // 4. Configure Resolution & Frequency
    Serial.printf("[%s] Configuring resolution (%s) @ %u Hz...\n", 
                  ctx->name, 
                  (current_resolution == VL53L8CX_RESOLUTION_8X8) ? "8x8" : "4x4", 
                  current_frequency_hz);
    vl53l8cx_set_resolution(p_dev, current_resolution);
    vl53l8cx_set_ranging_frequency_hz(p_dev, current_frequency_hz);
    vl53l8cx_set_integration_time_ms(p_dev, current_integration_ms);
    vl53l8cx_set_target_order(p_dev, current_target_order);
    vl53l8cx_set_sharpener_percent(p_dev, current_sharpener);
    vl53l8cx_set_VHV_repeat_count(p_dev, 14400);

    // 5. Start Continuous Ranging
    Serial.printf("[%s] Starting continuous ranging session...\n", ctx->name);
    status = vl53l8cx_start_ranging(p_dev);
    Serial.printf("[%s] vl53l8cx_start_ranging status = %u\n", ctx->name, status);

    if (status != 0) {
        Serial.printf("[ERROR] %s failed to start ranging! status = %u\n", ctx->name, status);
        return false;
    }

    Serial.printf("[SUCCESS] %s is fully operational!\n", ctx->name);
    return true;
}

// =========================================================================
// ON-CHIP 8-ZONE 90° RADAR FUSION & TELEMETRY
// =========================================================================

static inline bool is_valid_tof_target(int16_t d, uint8_t st) {
    return (st == 5 || st == 6 || st == 9 || st == 10) && (d > 0 && d <= 4000);
}

void update_onchip_fused_radar() {
    static const uint8_t zone_sensor[8] = {1, 1, 1, 1, 2, 2, 2, 2};
    static const uint8_t zone_src_idx[8] = {13, 9, 5, 1, 2, 6, 10, 14};

    uint16_t overall_min = 3000;

    for (int z = 0; z < 8; z++) {
        uint8_t sid = zone_sensor[z];
        uint8_t src = zone_src_idx[z];
        bool active = (sid == 1) ? tof_active[0] : tof_active[1];
        VL53L8CX_ResultsData *res = (sid == 1) ? &Results[0] : &Results[1];

        if (!active) {
            fused_radar.zones[z].distance_mm = 3000;
            fused_radar.zones[z].status = 0;
            fused_radar.zones[z].spad_count = 0;
            fused_radar.zones[z].target_num = 0;
            fused_radar.zones[z].fallback_type = 2; // open space
            fused_radar.zones[z].t2_dist = 0;
            fused_radar.zones[z].t3_dist = 0;
            continue;
        }

        uint16_t spad_val = 0;
#ifndef VL53L8CX_DISABLE_NB_SPADS_ENABLED
        spad_val = res->nb_spads_enabled[src];
#endif
        fused_radar.zones[z].spad_count = spad_val;

        int src0 = src * VL53L8CX_NB_TARGET_PER_ZONE;
        int src1 = src * VL53L8CX_NB_TARGET_PER_ZONE + 1;
        int src2 = src * VL53L8CX_NB_TARGET_PER_ZONE + 2;

        int16_t d0 = res->distance_mm[src0];
        uint8_t st0 = res->target_status[src0];

        int16_t d1 = (VL53L8CX_NB_TARGET_PER_ZONE > 1) ? res->distance_mm[src1] : 0;
        uint8_t st1 = (VL53L8CX_NB_TARGET_PER_ZONE > 1) ? res->target_status[src1] : 0;

        int16_t d2 = (VL53L8CX_NB_TARGET_PER_ZONE > 2) ? res->distance_mm[src2] : 0;
        uint8_t st2 = (VL53L8CX_NB_TARGET_PER_ZONE > 2) ? res->target_status[src2] : 0;

        bool v0 = is_valid_tof_target(d0, st0);
        bool v1 = is_valid_tof_target(d1, st1);
        bool v2 = (current_nb_targets >= 3) ? is_valid_tof_target(d2, st2) : false;

        fused_radar.zones[z].t2_dist = v1 ? (uint16_t)d1 : 0;
        fused_radar.zones[z].t3_dist = v2 ? (uint16_t)d2 : 0;

        if (v0 || v1 || v2) {
            uint16_t best_d = 4000;
            uint8_t best_st = 0;
            uint8_t best_t = 1;

            if (v0 && (uint16_t)d0 < best_d) {
                best_d = (uint16_t)d0;
                best_st = st0;
                best_t = 1;
            }
            if (v1 && (uint16_t)d1 < best_d) {
                best_d = (uint16_t)d1;
                best_st = st1;
                best_t = 2;
            }
            if (v2 && (uint16_t)d2 < best_d) {
                best_d = (uint16_t)d2;
                best_st = st2;
                best_t = 3;
            }

            fused_radar.zones[z].distance_mm = best_d;
            fused_radar.zones[z].status = best_st;
            fused_radar.zones[z].target_num = best_t;
            fused_radar.zones[z].fallback_type = 0; // valid hit

            error_latch_count[z] = 0;
            last_valid_dist[z] = best_d;
        } else {
            error_latch_count[z]++;
            if (error_latch_count[z] < 2 && last_valid_dist[z] > 0) {
                fused_radar.zones[z].distance_mm = last_valid_dist[z];
                fused_radar.zones[z].status = st0;
                fused_radar.zones[z].target_num = 0;
                fused_radar.zones[z].fallback_type = 3; // latched frame
            } else {
                fused_radar.zones[z].status = st0;
                fused_radar.zones[z].target_num = 0;
                if (spad_val == 1024) {
                    fused_radar.zones[z].distance_mm = 0;
                    fused_radar.zones[z].fallback_type = 1; // 0mm contact
                } else {
                    fused_radar.zones[z].distance_mm = 3000;
                    fused_radar.zones[z].fallback_type = 2; // 3m open space
                }
            }
        }

        uint16_t zd = fused_radar.zones[z].distance_mm;
        if (zd > 0 && zd < overall_min) {
            overall_min = zd;
        }
    }

    fused_radar.min_distance_mm = overall_min;
    fused_radar.last_update_ms = millis();
    fused_radar.seq++;
}

void send_arc_telemetry(uint8_t sensor_id) {
    static char arc_buf[512];
    int pos = snprintf(arc_buf, sizeof(arc_buf),
        "{\"t\":\"A\",\"id\":%u,\"seq\":%lu,\"ts\":%lu,\"min\":%u,\"d\":[",
        sensor_id, fused_radar.seq, (unsigned long)millis(), fused_radar.min_distance_mm);

    for (int i = 0; i < 8; i++) {
        pos += snprintf(arc_buf + pos, sizeof(arc_buf) - pos, i ? ",%u" : "%u", fused_radar.zones[i].distance_mm);
    }
    pos += snprintf(arc_buf + pos, sizeof(arc_buf) - pos, "],\"st\":[");
    for (int i = 0; i < 8; i++) {
        pos += snprintf(arc_buf + pos, sizeof(arc_buf) - pos, i ? ",%u" : "%u", fused_radar.zones[i].status);
    }
    pos += snprintf(arc_buf + pos, sizeof(arc_buf) - pos, "],\"sp\":[");
    for (int i = 0; i < 8; i++) {
        pos += snprintf(arc_buf + pos, sizeof(arc_buf) - pos, i ? ",%u" : "%u", fused_radar.zones[i].spad_count);
    }
    pos += snprintf(arc_buf + pos, sizeof(arc_buf) - pos, "],\"fb\":[");
    for (int i = 0; i < 8; i++) {
        pos += snprintf(arc_buf + pos, sizeof(arc_buf) - pos, i ? ",%u" : "%u", (unsigned int)fused_radar.zones[i].fallback_type);
    }
    pos += snprintf(arc_buf + pos, sizeof(arc_buf) - pos, "],\"tn\":[");
    for (int i = 0; i < 8; i++) {
        pos += snprintf(arc_buf + pos, sizeof(arc_buf) - pos, i ? ",%u" : "%u", (unsigned int)fused_radar.zones[i].target_num);
    }
    pos += snprintf(arc_buf + pos, sizeof(arc_buf) - pos, "],\"t2\":[");
    for (int i = 0; i < 8; i++) {
        pos += snprintf(arc_buf + pos, sizeof(arc_buf) - pos, i ? ",%u" : "%u", fused_radar.zones[i].t2_dist);
    }
    pos += snprintf(arc_buf + pos, sizeof(arc_buf) - pos, "]}\n");

    Serial.write((const uint8_t *)arc_buf, pos);
}

static char telemetry_buf[8192];

static inline int append_array_u(char *buf, int pos, const char *key, const uint32_t *vals, int count) {
    pos += snprintf(buf + pos, sizeof(telemetry_buf) - pos, ",\"%s\":[", key);
    for (int i = 0; i < count; i++) {
        pos += snprintf(buf + pos, sizeof(telemetry_buf) - pos, i ? ",%lu" : "%lu", (unsigned long)vals[i]);
    }
    buf[pos++] = ']';
    return pos;
}

void send_json_telemetry(uint8_t sensor_id, VL53L8CX_ResultsData *p_res, unsigned long f_count) {
    const int n_zones = (current_resolution == VL53L8CX_RESOLUTION_8X8) ? 64 : 16;
    const int max_hw_targets = (VL53L8CX_NB_TARGET_PER_ZONE < 3) ? VL53L8CX_NB_TARGET_PER_ZONE : 3;
    const int nt_out = (current_nb_targets < 2) ? 2 : ((current_nb_targets > max_hw_targets) ? max_hw_targets : current_nb_targets);
    const int n_vals = n_zones * nt_out;

    uint32_t d[256], st[256], spad[64];
    for (int z = 0; z < n_zones; z++) {
        for (int t = 0; t < nt_out; t++) {
            int src = z * VL53L8CX_NB_TARGET_PER_ZONE + t;
            int dst = z * nt_out + t;
            int16_t raw_d = p_res->distance_mm[src];
            d[dst]  = (raw_d < 0) ? 0 : (uint32_t)raw_d;
            st[dst] = p_res->target_status[src];
        }
#ifndef VL53L8CX_DISABLE_NB_SPADS_ENABLED
        spad[z] = p_res->nb_spads_enabled[z];
#else
        spad[z] = 0;
#endif
    }

    int pos = snprintf(telemetry_buf, sizeof(telemetry_buf),
                       "{\"t\":\"D\",\"id\":%u,\"seq\":%lu,\"ts\":%lu,\"r\":%d,\"nt\":%d,\"temp\":%d",
                       sensor_id, (unsigned long)f_count, (unsigned long)millis(),
                       n_zones, nt_out, (int)p_res->silicon_temp_degc);
    pos = append_array_u(telemetry_buf, pos, "d", d, n_vals);
    pos = append_array_u(telemetry_buf, pos, "st", st, n_vals);
#ifndef VL53L8CX_DISABLE_NB_SPADS_ENABLED
    pos = append_array_u(telemetry_buf, pos, "spad", spad, n_zones);
#endif
    telemetry_buf[pos++] = '}';
    telemetry_buf[pos++] = '\n';
    Serial.write((const uint8_t *)telemetry_buf, pos);
}

// Beautiful human-readable 4x4 ASCII display for Serial Monitor
void print_ascii_matrices() {
    static unsigned long last_print_ms = 0;
    if (millis() - last_print_ms < 500) return; // 2 Hz
    last_print_ms = millis();

    Serial.println(F("\n========================================================================================================================="));
    Serial.printf(" [ToF 1 CS:10] %s (%lu f) | [ToF 2 CS:9] %s (%lu f) | [ToF 3 CS:7] %s (%lu f) | [ToF 4 CS:17] %s (%lu f)\n",
                  tof_active[0] ? "ONLINE" : "OFFLINE", frame_count[0],
                  tof_active[1] ? "ONLINE" : "OFFLINE", frame_count[1],
                  tof_active[2] ? "ONLINE" : "OFFLINE", frame_count[2],
                  tof_active[3] ? "ONLINE" : "OFFLINE", frame_count[3]);
    Serial.println(F("------------------------------+------------------------------+------------------------------+----------------------------"));

    const int n_side = (current_resolution == VL53L8CX_RESOLUTION_8X8) ? 8 : 4;
    for (int r = 0; r < n_side; r++) {
        for (int s = 0; s < NUM_TOFS; s++) {
            if (s > 0) Serial.print(F(" | "));
            for (int c = 0; c < n_side; c++) {
                int z = r * n_side + c;
                int src = z * VL53L8CX_NB_TARGET_PER_ZONE;
                int d = tof_active[s] ? (int)Results[s].distance_mm[src] : -1;
                int st = tof_active[s] ? Results[s].target_status[src] : 0;
                if (tof_active[s] && (st == 5 || st == 6 || st == 9 || st == 10)) {
                    Serial.printf("%5d", d);
                } else {
                    Serial.print(F("  ---"));
                }
            }
        }
        Serial.println();
    }
    Serial.println(F("========================================================================================================================="));
}

// =========================================================================
// SENSOR RECOVERY AND CONTROL
// =========================================================================

bool fast_soft_recover_sensor(VL53L8CX_Configuration *p_dev, ToF_Device_Context *ctx) {
    digitalWrite(ctx->cs_pin, HIGH);
    delay(2);
    vl53l8cx_stop_ranging(p_dev);
    delay(5);
    uint8_t st = vl53l8cx_start_ranging(p_dev);
    return (st == 0);
}

void apply_sensor_configuration(String params) {
    int res = -1, freq = -1, integ = -1, order = -1, sharp = -1, targets = -1;
    
    int start = 0;
    while (start < params.length()) {
        int eq = params.indexOf('=', start);
        if (eq == -1) break;
        int comma = params.indexOf(',', eq);
        if (comma == -1) comma = params.length();
        
        String key = params.substring(start, eq);
        String val = params.substring(eq + 1, comma);
        key.trim(); val.trim();
        
        if (key == "res") res = val.toInt();
        else if (key == "freq") freq = val.toInt();
        else if (key == "integ") integ = val.toInt();
        else if (key == "order") order = val.toInt();
        else if (key == "sharp") sharp = val.toInt();
        else if (key == "targets" || key == "nt") targets = val.toInt();
        
        start = comma + 1;
    }
    
    for (int i = 0; i < NUM_TOFS; i++) {
        if (tof_active[i]) vl53l8cx_stop_ranging(&Dev[i]);
    }
    delay(5);
    
    if (res == 16 || res == 64) {
        current_resolution = (res == 64) ? VL53L8CX_RESOLUTION_8X8 : VL53L8CX_RESOLUTION_4X4;
        for (int i = 0; i < NUM_TOFS; i++) {
            if (tof_active[i]) vl53l8cx_set_resolution(&Dev[i], current_resolution);
        }
    }
    
    if (freq >= 1 && freq <= 60) {
        if (current_resolution == VL53L8CX_RESOLUTION_8X8 && freq > 15) {
            freq = 15;
        }
        current_frequency_hz = freq;
        for (int i = 0; i < NUM_TOFS; i++) {
            if (tof_active[i]) vl53l8cx_set_ranging_frequency_hz(&Dev[i], current_frequency_hz);
        }
    }
    
    if (integ >= 2 && integ <= 1000) {
        current_integration_ms = integ;
    }
    if (current_integration_ms * current_frequency_hz > 950) {
        current_integration_ms = max(2UL, 950UL / current_frequency_hz);
    }
    for (int i = 0; i < NUM_TOFS; i++) {
        if (tof_active[i]) vl53l8cx_set_integration_time_ms(&Dev[i], current_integration_ms);
    }
    
    if (order == 1 || order == 2) {
        current_target_order = order;
        for (int i = 0; i < NUM_TOFS; i++) {
            if (tof_active[i]) vl53l8cx_set_target_order(&Dev[i], current_target_order);
        }
    }
    
    if (sharp >= 0 && sharp <= 99) {
        current_sharpener = sharp;
        for (int i = 0; i < NUM_TOFS; i++) {
            if (tof_active[i]) vl53l8cx_set_sharpener_percent(&Dev[i], current_sharpener);
        }
    }
    
    if (targets == 2 || targets == 3) {
        current_nb_targets = (uint8_t)targets;
    }
    
    for (int i = 0; i < NUM_TOFS; i++) {
        if (tof_active[i]) {
            vl53l8cx_set_VHV_repeat_count(&Dev[i], 14400);
            vl53l8cx_start_ranging(&Dev[i]);
        }
    }
    Serial.println(F("{\"type\":\"ACK\",\"status\":\"OK\",\"cmd\":\"SET_CONFIG\"}"));
}

void dispatch_command(const String &cmd) {
    if (cmd == "GET_CONFIG") {
        Serial.printf("{\"type\":\"CONFIG\",\"res\":%u,\"freq\":%u,\"integ\":%lu,\"order\":%u,\"sharp\":%u,\"targets\":%u,\"mode\":1,\"power\":1,\"margin\":50}\n",
                      (current_resolution == VL53L8CX_RESOLUTION_8X8 ? 64 : 16),
                      current_frequency_hz, current_integration_ms, current_target_order, current_sharpener, current_nb_targets);
    } else if (cmd == "RESET") {
        Serial.println(F("{\"type\":\"ACK\",\"status\":\"OK\",\"cmd\":\"RESET\"}"));
        for (int i = 0; i < NUM_TOFS; i++) {
            tof_active[i] = initialize_tof_sensor(&Dev[i], &ctx_tofs[i]);
        }
    } else if (cmd == "SET_STREAM:arc") {
        stream_arc_mode = true;
        stream_diag_matrix = false;
        print_ascii_enabled = false;
        Serial.println(F("{\"type\":\"ACK\",\"status\":\"OK\",\"stream\":\"arc\"}"));
    } else if (cmd == "SET_STREAM:matrix") {
        stream_arc_mode = false;
        stream_diag_matrix = true;
        Serial.println(F("{\"type\":\"ACK\",\"status\":\"OK\",\"stream\":\"matrix\"}"));
    } else if (cmd == "SET_STREAM:both") {
        stream_arc_mode = true;
        stream_diag_matrix = true;
        Serial.println(F("{\"type\":\"ACK\",\"status\":\"OK\",\"stream\":\"both\"}"));
    } else if (cmd == "ASCII" || cmd == "PRINT_ASCII=1") {
        print_ascii_enabled = true;
        Serial.println(F("{\"type\":\"ACK\",\"status\":\"OK\",\"ascii\":1}"));
    } else if (cmd == "PRINT_ASCII=0") {
        print_ascii_enabled = false;
        Serial.println(F("{\"type\":\"ACK\",\"status\":\"OK\",\"ascii\":0}"));
    } else if (cmd.startsWith("TEST") || cmd == "SCAN") {
        int sid = 0;
        if (cmd.startsWith("TEST ") || cmd.startsWith("TEST:")) {
            sid = cmd.substring(5).toInt();
        }
        if (sid >= 1 && sid <= 4) {
            run_deep_sensor_diagnostics(&ctx_tofs[sid - 1]);
        } else {
            for (int i = 0; i < NUM_TOFS; i++) {
                run_deep_sensor_diagnostics(&ctx_tofs[i]);
            }
        }
    } else if (cmd.startsWith("SET_CONFIG:") || cmd.startsWith("SET_CONFIG ")) {
        int colon = cmd.indexOf(':');
        if (colon == -1) colon = cmd.indexOf(' ');
        apply_sensor_configuration(cmd.substring(colon + 1));
    }
}

void handle_serial_commands() {
    static String rx_buf = "";
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            rx_buf.trim();
            if (rx_buf.length() > 0) {
                dispatch_command(rx_buf);
            }
            rx_buf = "";
        } else if (c != '\r') {
            rx_buf += c;
            if (rx_buf.length() > 256) {
                rx_buf = "";
            }
        }
    }
}

// =========================================================================
// MAIN SETUP
// =========================================================================

void setup() {
    Serial.setTxBufferSize(8192);
    Serial.begin(921600);
    delay(200);

    Serial.println(F("\n================================================================="));
    Serial.println(F("   DA WINNA — QUAD VL53L8CX TOF SENSOR HUB (HSPI BUS)           "));
    Serial.println(F("   Shared SPI: SCK=39, MOSI=40                                   "));
    for (int i = 0; i < NUM_TOFS; i++) {
        Serial.printf("   ToF %d: CS=%d, INT=%d, LPN=%d, MISO=%d\n",
                      i + 1, ctx_tofs[i].cs_pin, ctx_tofs[i].int_pin, ctx_tofs[i].lpn_pin, ctx_tofs[i].miso_pin);
    }
    Serial.println(F("================================================================="));

    // Reset GPIOs and detach any JTAG / Matrix functions
    for (int i = 0; i < NUM_TOFS; i++) {
        gpio_reset_pin((gpio_num_t)ctx_tofs[i].cs_pin);
        gpio_reset_pin((gpio_num_t)ctx_tofs[i].lpn_pin);
        gpio_reset_pin((gpio_num_t)ctx_tofs[i].int_pin);
        gpio_reset_pin((gpio_num_t)ctx_tofs[i].miso_pin);

        // Initial pin state: CS pins HIGH (inactive)
        pinMode(ctx_tofs[i].cs_pin, OUTPUT);
        digitalWrite(ctx_tofs[i].cs_pin, HIGH);

        // Staggered LPn startup: keep ALL in low-power standby initially
        pinMode(ctx_tofs[i].lpn_pin, OUTPUT);
        digitalWrite(ctx_tofs[i].lpn_pin, LOW);

        pinMode(ctx_tofs[i].int_pin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(ctx_tofs[i].int_pin), isrs[i], FALLING);
        pinMode(ctx_tofs[i].miso_pin, INPUT_PULLUP);
    }

    gpio_reset_pin((gpio_num_t)TOF_SHARED_SCK_PIN);
    gpio_reset_pin((gpio_num_t)TOF_SHARED_MOSI_PIN);

    delay(50);

    // Initialize SPI with default pins
    HardwareSPI.begin(TOF_SHARED_SCK_PIN, ctx_tofs[0].miso_pin, TOF_SHARED_MOSI_PIN, -1);
    delay(50);

    // Staggered initialization: bring each sensor out of reset one by one
    for (int i = 0; i < NUM_TOFS; i++) {
        Serial.printf("\n[POWER] Bringing %s out of reset...\n", ctx_tofs[i].name);
        digitalWrite(ctx_tofs[i].lpn_pin, HIGH);
        delay(100);
        tof_active[i] = initialize_tof_sensor(&Dev[i], &ctx_tofs[i]);
        delay(50);
    }

    // Immediate boot retry if any sensor failed initial negotiation
    for (int i = 0; i < NUM_TOFS; i++) {
        if (!tof_active[i]) {
            Serial.printf("\n[BOOT-RETRY] Retrying %s initialization...\n", ctx_tofs[i].name);
            delay(150);
            tof_active[i] = initialize_tof_sensor(&Dev[i], &ctx_tofs[i]);
        }
    }

    Serial.println(F("\n================================================================="));
    Serial.printf(" QUAD TOF INITIALIZATION SUMMARY:\n");
    for (int i = 0; i < NUM_TOFS; i++) {
        Serial.printf("   ToF %d (CS %2d, MISO %2d): %s\n",
                      i + 1, ctx_tofs[i].cs_pin, ctx_tofs[i].miso_pin,
                      tof_active[i] ? "ONLINE" : "FAILED / NOT FOUND");
    }
    Serial.println(F("=================================================================\n"));
}

// =========================================================================
// MAIN LOOP
// =========================================================================

void loop() {
    handle_serial_commands();

    // Service each active ToF
    for (int i = 0; i < NUM_TOFS; i++) {
        if (tof_active[i]) {
            bool ready = int_triggered[i] || (digitalRead(ctx_tofs[i].int_pin) == LOW);
            if (ready || (millis() - last_poll_ms[i] >= 1)) {
                last_poll_ms[i] = millis();
                int_triggered[i] = false;
                uint8_t isReady = 0;
                vl53l8cx_check_data_ready(&Dev[i], &isReady);
                if (isReady) {
                    if (vl53l8cx_get_ranging_data(&Dev[i], &Results[i]) == 0) {
                        frame_count[i]++;
                        last_frame_time_ms[i] = millis();
                        if (stream_diag_matrix) {
                            send_json_telemetry(i + 1, &Results[i], frame_count[i]);
                        }
                        if (stream_arc_mode && (i == 0 || i == 1)) {
                            update_onchip_fused_radar();
                            send_arc_telemetry(i + 1);
                        }
                    }
                }
            }
        }
    }

    // Periodic ASCII table print ONLY if explicitly enabled
    if (print_ascii_enabled) {
        print_ascii_matrices();
    }

    // Background auto-reconnect if any sensor is dead (attempt every 10 seconds)
    static unsigned long last_reconnect_attempt_ms = 0;
    if (millis() - last_reconnect_attempt_ms > 10000) {
        last_reconnect_attempt_ms = millis();
        for (int i = 0; i < NUM_TOFS; i++) {
            if (!tof_active[i]) {
                Serial.printf("\n[AUTO-RETRY] Retrying %s initialization...\n", ctx_tofs[i].name);
                tof_active[i] = initialize_tof_sensor(&Dev[i], &ctx_tofs[i]);
            }
        }
    }

    delayMicroseconds(50);
}

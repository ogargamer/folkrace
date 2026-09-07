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
// HARDWARE PIN DEFINITIONS (VERIFIED WORKING CONFIGURATION)
// =========================================================================

// --- ToF 2 (Line 2 / HSPI Bus) ---
#define TOF2_SCK_PIN    39
#define TOF2_MOSI_PIN   40
#define TOF2_MISO_PIN   41
#define TOF2_CS_PIN     42
#define TOF2_INT_PIN    38
#define TOF2_LPN_PIN    47

// Ultra-stable SPI clock frequencies
#define SPI_INIT_SPEED_HZ   2000000 // 2 MHz for clean boot
#define SPI_RANGING_SPEED_HZ 3000000 // 3 MHz for live stream
#define CHUNK_SIZE          1024

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
};

SPIClass HardwareSPI(HSPI);

ToF_Device_Context ctx_tof2 = { 
    &HardwareSPI, 
    TOF2_CS_PIN, 
    TOF2_LPN_PIN, 
    TOF2_INT_PIN, 
    TOF2_SCK_PIN, 
    TOF2_MISO_PIN, 
    TOF2_MOSI_PIN, 
    SPI_INIT_SPEED_HZ, 
    "ToF 2 (HSPI Line 2)" 
};

VL53L8CX_Configuration Dev2;
VL53L8CX_ResultsData Results2;
bool tof2_active = false;

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
    delayMicroseconds(1);
    ctx->spi->transfer(buf, 3);
    delayMicroseconds(1);
    digitalWrite(ctx->cs_pin, HIGH);
    delayMicroseconds(1);
    ctx->spi->endTransaction();
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
        delayMicroseconds(1);
        ctx->spi->transfer(tx_chunk_buffer, chunk + 2);
        delayMicroseconds(1);
        digitalWrite(ctx->cs_pin, HIGH);
        delayMicroseconds(1);
        ctx->spi->endTransaction();

        pos += chunk;
    }
    return 0;
}

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAddress, uint8_t *p_value) {
    ToF_Device_Context *ctx = (ToF_Device_Context *)p_platform->handle;
    uint8_t addr_buf[2];
    addr_buf[0] = (uint8_t)(((RegisterAddress & ~0x8000) >> 8) & 0xFF);
    addr_buf[1] = (uint8_t)(RegisterAddress & 0xFF);

    ctx->spi->beginTransaction(SPISettings(ctx->speed_hz, MSBFIRST, SPI_MODE3));
    digitalWrite(ctx->cs_pin, LOW);
    delayMicroseconds(1);
    ctx->spi->transfer(addr_buf, 2);
    *p_value = ctx->spi->transfer(0x00);
    delayMicroseconds(1);
    digitalWrite(ctx->cs_pin, HIGH);
    delayMicroseconds(1);
    ctx->spi->endTransaction();
    return 0;
}

uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size) {
    ToF_Device_Context *ctx = (ToF_Device_Context *)p_platform->handle;
    uint32_t pos = 0;
    while (pos < size) {
        uint32_t chunk = (size - pos > CHUNK_SIZE) ? CHUNK_SIZE : (size - pos);
        uint16_t r = RegisterAddress + pos;

        uint8_t addr_buf[2];
        addr_buf[0] = (uint8_t)(((r & ~0x8000) >> 8) & 0xFF);
        addr_buf[1] = (uint8_t)(r & 0xFF);

        ctx->spi->beginTransaction(SPISettings(ctx->speed_hz, MSBFIRST, SPI_MODE3));
        digitalWrite(ctx->cs_pin, LOW);
        delayMicroseconds(1);
        ctx->spi->transfer(addr_buf, 2);
        for (uint32_t i = 0; i < chunk; i++) {
            p_values[pos + i] = ctx->spi->transfer(0x00);
        }
        delayMicroseconds(1);
        digitalWrite(ctx->cs_pin, HIGH);
        delayMicroseconds(1);
        ctx->spi->endTransaction();

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

bool initialize_tof_sensor(VL53L8CX_Configuration *p_dev, ToF_Device_Context *ctx) {
    Serial.printf("\n>>> [STARTING] %s Init Sequence <<<\n", ctx->name);

    ctx->speed_hz = SPI_INIT_SPEED_HZ;

    memset(p_dev, 0, sizeof(VL53L8CX_Configuration));
    p_dev->platform.handle = (void *)ctx;
    p_dev->platform.address = 0x29;

    // 1. Hardware Reset via LPn
    Serial.printf("[%s] Power cycling via LPn (GPIO %d)...\n", ctx->name, ctx->lpn_pin);
    digitalWrite(ctx->lpn_pin, LOW);
    delay(100);
    digitalWrite(ctx->lpn_pin, HIGH);
    delay(200);

    // 2. Check is_alive
    Serial.printf("[%s] Verifying SPI communication & device ID...\n", ctx->name);
    uint8_t isAlive = 0;
    uint8_t status = vl53l8cx_is_alive(p_dev, &isAlive);
    Serial.printf("[%s] vl53l8cx_is_alive: status = %u, isAlive = %u\n", ctx->name, status, isAlive);

    if (!isAlive) {
        Serial.printf("[ERROR] %s did not report alive! (status=%u)\n", ctx->name, status);
        return false;
    }

    // 3. Upload 84KB Firmware Blob
    Serial.printf("[%s] Uploading ST Official 84KB Firmware via Burst SPI @ %lu Hz...\n", ctx->name, ctx->speed_hz);
    unsigned long t0 = millis();
    status = vl53l8cx_init(p_dev);
    Serial.printf("[%s] vl53l8cx_init: status = %u (upload took %lu ms)\n", ctx->name, status, millis() - t0);

    if (status != 0) {
        Serial.printf("[ERROR] %s Firmware upload failed! status = %u\n", ctx->name, status);
        return false;
    }

    // Switch to Ranging Speed (3 MHz)
    ctx->speed_hz = SPI_RANGING_SPEED_HZ;

    // 4. Configure Resolution & Frequency (4x4 matrix @ 15 Hz)
    Serial.printf("[%s] Configuring 4x4 matrix resolution @ 15 Hz...\n", ctx->name);
    vl53l8cx_set_resolution(p_dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_frequency_hz(p_dev, 15);
    vl53l8cx_set_target_order(p_dev, VL53L8CX_TARGET_ORDER_CLOSEST);

    // 5. Start Continuous Ranging
    Serial.printf("[%s] Starting continuous ranging session...\n", ctx->name);
    status = vl53l8cx_start_ranging(p_dev);
    Serial.printf("[%s] vl53l8cx_start_ranging: status = %u\n", ctx->name, status);

    if (status != 0) {
        Serial.printf("[ERROR] %s failed to start ranging! status = %u\n", ctx->name, status);
        return false;
    }

    Serial.printf("[SUCCESS] %s is fully operational and streaming live!\n", ctx->name);
    return true;
}

// =========================================================================
// MAIN SETUP
// =========================================================================

void setup() {
    Serial.begin(921600);
    delay(2000);

    Serial.println(F("\n================================================================="));
    Serial.println(F("       ESP32-S3 VL53L8CX TOF VERIFICATION ENGINE                 "));
    Serial.println(F("  ToF 2: SCK=39, MOSI=40, MISO=41, CS=42, INT=38, LPN=47        "));
    Serial.println(F("================================================================="));

    pinMode(TOF2_CS_PIN, OUTPUT);
    digitalWrite(TOF2_CS_PIN, HIGH);
    pinMode(TOF2_LPN_PIN, OUTPUT);
    digitalWrite(TOF2_LPN_PIN, HIGH);
    pinMode(TOF2_INT_PIN, INPUT_PULLUP);

    // Start HSPI
    HardwareSPI.begin(TOF2_SCK_PIN, TOF2_MISO_PIN, TOF2_MOSI_PIN, -1);

    // Initialize ToF 2
    tof2_active = initialize_tof_sensor(&Dev2, &ctx_tof2);

    Serial.println(F("\n================================================================="));
    Serial.printf(" TOF INITIALIZATION SUMMARY: ToF2=%s\n", tof2_active ? "ONLINE" : "FAILED");
    Serial.println(F("=================================================================\n"));
}

// =========================================================================
// MAIN LOOP: STREAM LIVE 4X4 DISTANCE MATRICES
// =========================================================================

unsigned long frame_count = 0;
unsigned long last_print_time = 0;

void loop() {
    uint8_t isReady = 0;

    if (tof2_active) {
        uint8_t st = vl53l8cx_check_data_ready(&Dev2, &isReady);
        if (st == 0 && isReady) {
            if (vl53l8cx_get_ranging_data(&Dev2, &Results2) == 0) {
                frame_count++;
            }
        }
    }

    if (millis() - last_print_time >= 100) {
        last_print_time = millis();

        if (tof2_active && frame_count > 0) {
            Serial.printf("[ToF 2 - HSPI] Frame #%lu (Stream #%3u):\n", frame_count, Dev2.streamcount);
            for (int r = 0; r < 4; r++) {
                Serial.print("  |");
                for (int c = 0; c < 4; c++) {
                    int idx = r * 4 + c;
                    int16_t dist = Results2.distance_mm[idx];
                    uint8_t st = Results2.target_status[idx];
                    Serial.printf(" %4dmm(s%1u) |", dist, st);
                }
                Serial.println();
            }
            Serial.println("-----------------------------------------------------------------");
        }
    }

    delay(5);
}

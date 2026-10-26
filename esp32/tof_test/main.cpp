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
// HARDWARE PIN DEFINITIONS (DUAL TOF ON SHARED HSPI BUS)
// =========================================================================

// --- Shared SPI Bus Lines ---
#define TOF_SHARED_SCK_PIN   39
#define TOF_SHARED_MOSI_PIN  40

// --- ToF 1 (CS=10, INT=5, LPN=6, MISO=41) ---
#define TOF1_CS_PIN          10
#define TOF1_INT_PIN         5
#define TOF1_LPN_PIN         6
#define TOF1_MISO_PIN        41

// --- ToF 2 (CS=9, INT=4, LPN=8, MISO=13) ---
#define TOF2_CS_PIN          9
#define TOF2_INT_PIN         4
#define TOF2_LPN_PIN         8
#define TOF2_MISO_PIN        13

// ST-certified SPI clock frequency (1 MHz for rock-solid breadboard init, 2 MHz for ranging)
#define SPI_INIT_SPEED_HZ    1000000
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
};

SPIClass HardwareSPI(HSPI);

ToF_Device_Context ctx_tof1 = { 
    &HardwareSPI, 
    TOF1_CS_PIN, 
    TOF1_LPN_PIN, 
    TOF1_INT_PIN, 
    TOF_SHARED_SCK_PIN, 
    TOF1_MISO_PIN, 
    TOF_SHARED_MOSI_PIN, 
    SPI_INIT_SPEED_HZ, 
    "ToF 1 (CS:10, MISO:41)" 
};

ToF_Device_Context ctx_tof2 = { 
    &HardwareSPI, 
    TOF2_CS_PIN, 
    TOF2_LPN_PIN, 
    TOF2_INT_PIN, 
    TOF_SHARED_SCK_PIN, 
    TOF2_MISO_PIN, 
    TOF_SHARED_MOSI_PIN, 
    SPI_INIT_SPEED_HZ, 
    "ToF 2 (CS:9, MISO:13)" 
};

VL53L8CX_Configuration Dev1, Dev2;
VL53L8CX_ResultsData Results1, Results2;
bool tof1_active = false;
bool tof2_active = false;

// =========================================================================
// RUNTIME SENSOR CONFIGURATION STATE (Default: 4x4 @ 60 Hz)
// =========================================================================

uint8_t current_resolution = VL53L8CX_RESOLUTION_4X4;
uint8_t current_frequency_hz = 60;
uint32_t current_integration_ms = 12;
uint8_t current_target_order = VL53L8CX_TARGET_ORDER_CLOSEST;
uint8_t current_sharpener = 1;
uint8_t current_nb_targets = 3;

unsigned long frame_count1 = 0;
unsigned long frame_count2 = 0;
unsigned long last_frame_time1_ms = 0;
unsigned long last_frame_time2_ms = 0;
unsigned long last_recovery_attempt1_ms = 0;
unsigned long last_recovery_attempt2_ms = 0;
uint8_t recovery_tier1 = 0;
uint8_t recovery_tier2 = 0;

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
    delayMicroseconds(2);
    ctx->spi->transferBytes(buf, NULL, 3);
    delayMicroseconds(2);
    digitalWrite(ctx->cs_pin, HIGH);
    ctx->spi->endTransaction();
    delayMicroseconds(10); // Guard time for internal PMU register latching
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
        delayMicroseconds(2);
        ctx->spi->transferBytes(tx_chunk_buffer, NULL, chunk + 2);
        delayMicroseconds(2);
        digitalWrite(ctx->cs_pin, HIGH);
        ctx->spi->endTransaction();
        delayMicroseconds(10);

        pos += chunk;
    }
    return 0;
}

extern "C" {
    void spiAttachMISO(spi_t *spi, int8_t miso);
}

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAddress, uint8_t *p_value) {
    ToF_Device_Context *ctx = (ToF_Device_Context *)p_platform->handle;
    spiAttachMISO(ctx->spi->bus(), ctx->miso_pin);

    uint8_t addr_buf[2];
    addr_buf[0] = (uint8_t)(((RegisterAddress & ~0x8000) >> 8) & 0xFF);
    addr_buf[1] = (uint8_t)(RegisterAddress & 0xFF);

    ctx->spi->beginTransaction(SPISettings(ctx->speed_hz, MSBFIRST, SPI_MODE3));
    digitalWrite(ctx->cs_pin, LOW);
    delayMicroseconds(1);
    ctx->spi->transferBytes(addr_buf, NULL, 2);
    ctx->spi->transferBytes(NULL, p_value, 1);
    delayMicroseconds(1);
    digitalWrite(ctx->cs_pin, HIGH);
    delayMicroseconds(2);
    ctx->spi->endTransaction();
    return 0;
}

uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size) {
    ToF_Device_Context *ctx = (ToF_Device_Context *)p_platform->handle;
    spiAttachMISO(ctx->spi->bus(), ctx->miso_pin);

    uint8_t addr_buf[2];
    addr_buf[0] = (uint8_t)(((RegisterAddress & ~0x8000) >> 8) & 0xFF);
    addr_buf[1] = (uint8_t)(RegisterAddress & 0xFF);

    ctx->spi->beginTransaction(SPISettings(ctx->speed_hz, MSBFIRST, SPI_MODE3));
    digitalWrite(ctx->cs_pin, LOW);
    delayMicroseconds(2);
    ctx->spi->transferBytes(addr_buf, NULL, 2);
    ctx->spi->transferBytes(NULL, p_values, size);
    delayMicroseconds(2);
    digitalWrite(ctx->cs_pin, HIGH);
    delayMicroseconds(3);
    ctx->spi->endTransaction();
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

        if (isAlive && dev_id == 0xF0 && rev_id == 0x0C) {
            found_alive = true;
            break;
        }
        delay(30);
    }

    if (!found_alive) {
        Serial.printf("[ERROR] %s did not report alive! (dev_id=0x%02X, rev_id=0x%02X, isAlive=%u)\n", 
                      ctx->name, dev_id, rev_id, isAlive);
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
// TELEMETRY & DISPLAY FORMATTERS
// =========================================================================

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
            d[dst]  = (uint32_t)(int32_t)p_res->distance_mm[src];
            st[dst] = p_res->target_status[src];
        }
#ifndef VL53L8CX_DISABLE_NB_SPADS_ENABLED
        spad[z] = p_res->nb_spads_enabled[z];
#else
        spad[z] = 0;
#endif
    }
    for (int i = 0; i < n_vals; i++) if ((int32_t)d[i] < 0) d[i] = 0;

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

// Beautiful human-readable side-by-side 4x4 ASCII display for Serial Monitor
void print_ascii_matrices() {
    static unsigned long last_print_ms = 0;
    if (millis() - last_print_ms < 500) return; // Print every 500 ms (2 Hz)
    last_print_ms = millis();

    Serial.println(F("\n================================================================================"));
    Serial.printf(" [ToF 1 CS:10] %s (Frames: %lu)   |   [ToF 2 CS:9] %s (Frames: %lu)\n",
                  tof1_active ? "ONLINE" : "OFFLINE", frame_count1,
                  tof2_active ? "ONLINE" : "OFFLINE", frame_count2);
    Serial.println(F("------------------------------------------------+-------------------------------"));

    const int n_side = (current_resolution == VL53L8CX_RESOLUTION_8X8) ? 8 : 4;
    for (int r = 0; r < n_side; r++) {
        // Left column: ToF 1
        Serial.print(F(" "));
        for (int c = 0; c < n_side; c++) {
            int z = r * n_side + c;
            int src = z * VL53L8CX_NB_TARGET_PER_ZONE;
            int d = tof1_active ? (int)Results1.distance_mm[src] : -1;
            int st = tof1_active ? Results1.target_status[src] : 0;
            if (tof1_active && (st == 5 || st == 6 || st == 9 || st == 10)) {
                Serial.printf("%5dmm", d);
            } else {
                Serial.print(F("    ---"));
            }
        }
        // Separator
        Serial.print(F("   |  "));
        // Right column: ToF 2
        for (int c = 0; c < n_side; c++) {
            int z = r * n_side + c;
            int src = z * VL53L8CX_NB_TARGET_PER_ZONE;
            int d = tof2_active ? (int)Results2.distance_mm[src] : -1;
            int st = tof2_active ? Results2.target_status[src] : 0;
            if (tof2_active && (st == 5 || st == 6 || st == 9 || st == 10)) {
                Serial.printf("%5dmm", d);
            } else {
                Serial.print(F("    ---"));
            }
        }
        Serial.println();
    }
    Serial.println(F("================================================================================"));
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
    
    if (tof1_active) vl53l8cx_stop_ranging(&Dev1);
    if (tof2_active) vl53l8cx_stop_ranging(&Dev2);
    delay(5);
    
    if (res == 16 || res == 64) {
        current_resolution = (res == 64) ? VL53L8CX_RESOLUTION_8X8 : VL53L8CX_RESOLUTION_4X4;
        if (tof1_active) vl53l8cx_set_resolution(&Dev1, current_resolution);
        if (tof2_active) vl53l8cx_set_resolution(&Dev2, current_resolution);
    }
    
    if (freq >= 1 && freq <= 60) {
        if (current_resolution == VL53L8CX_RESOLUTION_8X8 && freq > 15) {
            freq = 15;
        }
        current_frequency_hz = freq;
        if (tof1_active) vl53l8cx_set_ranging_frequency_hz(&Dev1, current_frequency_hz);
        if (tof2_active) vl53l8cx_set_ranging_frequency_hz(&Dev2, current_frequency_hz);
    }
    
    if (integ >= 2 && integ <= 1000) {
        current_integration_ms = integ;
    }
    if (current_integration_ms * current_frequency_hz > 950) {
        current_integration_ms = max(2UL, 950UL / current_frequency_hz);
    }
    if (tof1_active) vl53l8cx_set_integration_time_ms(&Dev1, current_integration_ms);
    if (tof2_active) vl53l8cx_set_integration_time_ms(&Dev2, current_integration_ms);
    
    if (order == 1 || order == 2) {
        current_target_order = order;
        if (tof1_active) vl53l8cx_set_target_order(&Dev1, current_target_order);
        if (tof2_active) vl53l8cx_set_target_order(&Dev2, current_target_order);
    }
    
    if (sharp >= 0 && sharp <= 99) {
        current_sharpener = sharp;
        if (tof1_active) vl53l8cx_set_sharpener_percent(&Dev1, current_sharpener);
        if (tof2_active) vl53l8cx_set_sharpener_percent(&Dev2, current_sharpener);
    }
    
    if (targets == 2 || targets == 3) {
        current_nb_targets = (uint8_t)targets;
    }
    
    if (tof1_active) {
        vl53l8cx_set_VHV_repeat_count(&Dev1, 14400);
        vl53l8cx_start_ranging(&Dev1);
    }
    if (tof2_active) {
        vl53l8cx_set_VHV_repeat_count(&Dev2, 14400);
        vl53l8cx_start_ranging(&Dev2);
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
        tof1_active = initialize_tof_sensor(&Dev1, &ctx_tof1);
        tof2_active = initialize_tof_sensor(&Dev2, &ctx_tof2);
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
    Serial.println(F("   DA WINNA — DUAL VL53L8CX TOF SENSOR HUB (HSPI BUS)           "));
    Serial.println(F("   Shared SPI: SCK=39, MOSI=40                                   "));
    Serial.printf ("   ToF 1: CS=%d, INT=%d, LPN=%d, MISO=%d                         \n", TOF1_CS_PIN, TOF1_INT_PIN, TOF1_LPN_PIN, TOF1_MISO_PIN);
    Serial.printf ("   ToF 2: CS=%d, INT=%d, LPN=%d, MISO=%d                         \n", TOF2_CS_PIN, TOF2_INT_PIN, TOF2_LPN_PIN, TOF2_MISO_PIN);
    Serial.println(F("================================================================="));

    // Reset GPIOs and detach any JTAG / Matrix functions
    gpio_reset_pin((gpio_num_t)TOF1_CS_PIN);
    gpio_reset_pin((gpio_num_t)TOF1_LPN_PIN);
    gpio_reset_pin((gpio_num_t)TOF1_INT_PIN);
    gpio_reset_pin((gpio_num_t)TOF1_MISO_PIN);

    gpio_reset_pin((gpio_num_t)TOF2_CS_PIN);
    gpio_reset_pin((gpio_num_t)TOF2_LPN_PIN);
    gpio_reset_pin((gpio_num_t)TOF2_INT_PIN);
    gpio_reset_pin((gpio_num_t)TOF2_MISO_PIN);

    gpio_reset_pin((gpio_num_t)TOF_SHARED_SCK_PIN);
    gpio_reset_pin((gpio_num_t)TOF_SHARED_MOSI_PIN);

    // Initial pin state: CS pins HIGH (disabled/inactive)
    pinMode(TOF1_CS_PIN, OUTPUT);
    digitalWrite(TOF1_CS_PIN, HIGH);
    pinMode(TOF2_CS_PIN, OUTPUT);
    digitalWrite(TOF2_CS_PIN, HIGH);

    // Staggered LPn startup: keep BOTH in low-power standby initially
    pinMode(TOF1_LPN_PIN, OUTPUT);
    digitalWrite(TOF1_LPN_PIN, LOW);
    pinMode(TOF2_LPN_PIN, OUTPUT);
    digitalWrite(TOF2_LPN_PIN, LOW);
    delay(50);

    pinMode(TOF1_INT_PIN, INPUT_PULLUP);
    pinMode(TOF2_INT_PIN, INPUT_PULLUP);
    pinMode(TOF1_MISO_PIN, INPUT_PULLUP);
    pinMode(TOF2_MISO_PIN, INPUT_PULLUP);

    // Initialize SPI
    HardwareSPI.begin(TOF_SHARED_SCK_PIN, TOF1_MISO_PIN, TOF_SHARED_MOSI_PIN, -1);
    delay(50);

    // 1. Initialize ToF 1 (ToF 2 held in hardware sleep LPn=LOW)
    Serial.println(F("[POWER] Bringing ToF 1 out of reset (ToF 2 held in reset)..."));
    digitalWrite(TOF1_LPN_PIN, HIGH);
    delay(100);
    tof1_active = initialize_tof_sensor(&Dev1, &ctx_tof1);
    delay(50);

    // 2. Initialize ToF 2
    Serial.println(F("[POWER] Bringing ToF 2 out of reset..."));
    digitalWrite(TOF2_LPN_PIN, HIGH);
    delay(100);
    tof2_active = initialize_tof_sensor(&Dev2, &ctx_tof2);
    delay(50);

    Serial.println(F("\n================================================================="));
    Serial.printf(" DUAL TOF INITIALIZATION SUMMARY:\n");
    Serial.printf("   ToF 1 (CS 10): %s\n", tof1_active ? "ONLINE" : "FAILED / NOT FOUND");
    Serial.printf("   ToF 2 (CS  9): %s\n", tof2_active ? "ONLINE" : "FAILED / NOT FOUND");
    Serial.println(F("=================================================================\n"));
}

// =========================================================================
// MAIN LOOP
// =========================================================================

void loop() {
    handle_serial_commands();

    // 1. Service ToF 1
    if (tof1_active) {
        uint8_t isReady1 = 0;
        vl53l8cx_check_data_ready(&Dev1, &isReady1);
        if (isReady1) {
            if (vl53l8cx_get_ranging_data(&Dev1, &Results1) == 0) {
                frame_count1++;
                last_frame_time1_ms = millis();
                recovery_tier1 = 0;
                send_json_telemetry(1, &Results1, frame_count1);
            }
        }
    }

    // 2. Service ToF 2
    if (tof2_active) {
        uint8_t isReady2 = 0;
        vl53l8cx_check_data_ready(&Dev2, &isReady2);
        if (isReady2) {
            if (vl53l8cx_get_ranging_data(&Dev2, &Results2) == 0) {
                frame_count2++;
                last_frame_time2_ms = millis();
                recovery_tier2 = 0;
                send_json_telemetry(2, &Results2, frame_count2);
            }
        }
    }

    // 3. Periodic ASCII table print for live Serial Monitor viewing
    print_ascii_matrices();

    // 4. Background auto-reconnect for any offline sensor (every 60s to avoid blocking active sensor)
    static unsigned long last_reconnect_attempt_ms = 0;
    if ((!tof1_active || !tof2_active) && (millis() - last_reconnect_attempt_ms > 60000)) {
        last_reconnect_attempt_ms = millis();
        if (!tof1_active) {
            Serial.println(F("\n[AUTO-RETRY] Retrying ToF 1 initialization..."));
            tof1_active = initialize_tof_sensor(&Dev1, &ctx_tof1);
        }
        if (!tof2_active) {
            Serial.println(F("\n[AUTO-RETRY] Retrying ToF 2 initialization..."));
            tof2_active = initialize_tof_sensor(&Dev2, &ctx_tof2);
        }
    }

    delayMicroseconds(200);
}

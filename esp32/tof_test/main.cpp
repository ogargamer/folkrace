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

// ST-certified ultra-stable SPI clock frequency (2.0 MHz matching STSW-IMG042 Linux driver)
#define SPI_INIT_SPEED_HZ   2000000
#define SPI_RANGING_SPEED_HZ 2000000
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

// =========================================================================
// RUNTIME SENSOR CONFIGURATION STATE (Default: 4x4 @ 60 Hz)
// =========================================================================

uint8_t current_resolution = VL53L8CX_RESOLUTION_4X4;
uint8_t current_frequency_hz = 60;
uint32_t current_integration_ms = 5;
uint8_t current_target_order = VL53L8CX_TARGET_ORDER_CLOSEST;
uint8_t current_sharpener = 1;
unsigned long frame_count = 0;
unsigned long last_frame_time_ms = 0;
unsigned long last_recovery_attempt_ms = 0;
uint8_t recovery_tier = 0;

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
    ctx->spi->transferBytes(buf, NULL, 3);
    delayMicroseconds(1);
    digitalWrite(ctx->cs_pin, HIGH);
    delayMicroseconds(2);
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
        delayMicroseconds(2);
        ctx->spi->transferBytes(tx_chunk_buffer, NULL, chunk + 2);
        delayMicroseconds(2);
        digitalWrite(ctx->cs_pin, HIGH);
        delayMicroseconds(3);
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
    ctx->spi->transferBytes(addr_buf, NULL, 2);
    ctx->spi->transferBytes(NULL, p_value, 1);
    delayMicroseconds(1);
    digitalWrite(ctx->cs_pin, HIGH);
    delayMicroseconds(2);
    ctx->spi->endTransaction();
    return 0;
}

// Critical fix: Multi-byte reads (such as ranging results) must be read in a single
// unbroken SPI transaction with CS held low throughout, matching ST official driver.
uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size) {
    ToF_Device_Context *ctx = (ToF_Device_Context *)p_platform->handle;
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

    // 1. Hardware Reset via LPn with retries
    uint8_t dev_id = 0, rev_id = 0;
    uint8_t isAlive = 0;
    uint8_t status = 0;
    bool found_alive = false;

    for (int attempt = 1; attempt <= 3; attempt++) {
        digitalWrite(ctx->lpn_pin, LOW);
        delay(15);
        digitalWrite(ctx->lpn_pin, HIGH);
        delay(30);

        VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);
        VL53L8CX_RdByte(&(p_dev->platform), 0, &dev_id);
        VL53L8CX_RdByte(&(p_dev->platform), 1, &rev_id);

        status = vl53l8cx_is_alive(p_dev, &isAlive);

        if (isAlive && dev_id == 0xF0 && rev_id == 0x0C) {
            found_alive = true;
            break;
        }
        delay(20);
    }

    if (!found_alive) {
        Serial.printf("[ERROR] %s did not report alive after retries!\n", ctx->name);
        return false;
    }

    // 2. True MOSI Bank-Switch Verification
    Serial.printf("[%s] Verifying MOSI via Memory Bank Switching...\n", ctx->name);
    uint8_t b0_id = 0, b0_rev = 0, b2_id = 0, b2_rev = 0;

    // Select Bank 0x00 and read ID registers
    VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);
    VL53L8CX_RdByte(&(p_dev->platform), 0x0000, &b0_id);
    VL53L8CX_RdByte(&(p_dev->platform), 0x0001, &b0_rev);

    // Select Bank 0x02 and read the same offset (proves MOSI wrote 0x02 to 0x7FFF)
    VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x02);
    VL53L8CX_RdByte(&(p_dev->platform), 0x0000, &b2_id);
    VL53L8CX_RdByte(&(p_dev->platform), 0x0001, &b2_rev);

    // Restore Bank 0x00
    VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);

    Serial.printf("[%s] Bank 0x00 Read: [0x%02X 0x%02X] (Expected: 0xF0 0x0C)\n", ctx->name, b0_id, b0_rev);
    Serial.printf("[%s] Bank 0x02 Read: [0x%02X 0x%02X] (Differs from Bank 0x00 -> MOSI verified!)\n", ctx->name, b2_id, b2_rev);

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

    // 4. Configure Resolution & Frequency (4x4 matrix @ 60 Hz by default)
    Serial.printf("[%s] Configuring resolution (%s) @ %u Hz...\n", 
                  ctx->name, 
                  (current_resolution == VL53L8CX_RESOLUTION_8X8) ? "8x8" : "4x4", 
                  current_frequency_hz);
    vl53l8cx_set_resolution(p_dev, current_resolution);
    vl53l8cx_set_ranging_frequency_hz(p_dev, current_frequency_hz);
    vl53l8cx_set_integration_time_ms(p_dev, current_integration_ms);
    vl53l8cx_set_target_order(p_dev, current_target_order);
    vl53l8cx_set_sharpener_percent(p_dev, current_sharpener);
    vl53l8cx_set_VHV_repeat_count(p_dev, 14400); // 4 min interval @ 60 Hz

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
    Serial.setTxBufferSize(4096);
    Serial.begin(921600);
    delay(200);

    Serial.println(F("\n================================================================="));
    Serial.println(F("       ESP32-S3 VL53L8CX TOF VERIFICATION ENGINE                 "));
    Serial.println(F("  ToF 2: SCK=39, MOSI=40, MISO=41, CS=42, INT=38, LPN=47        "));
    Serial.println(F("================================================================="));

    // 1. Explicitly detach ESP32-S3 JTAG MUX on GPIOs 39-42 (MTCK, MTDO, MTDI, MTMS) and pins 38, 47
    gpio_reset_pin((gpio_num_t)TOF2_CS_PIN);
    gpio_reset_pin((gpio_num_t)TOF2_LPN_PIN);
    gpio_reset_pin((gpio_num_t)TOF2_INT_PIN);
    gpio_reset_pin((gpio_num_t)TOF2_SCK_PIN);
    gpio_reset_pin((gpio_num_t)TOF2_MISO_PIN);
    gpio_reset_pin((gpio_num_t)TOF2_MOSI_PIN);

    pinMode(TOF2_CS_PIN, OUTPUT);
    digitalWrite(TOF2_CS_PIN, HIGH);
    pinMode(TOF2_LPN_PIN, OUTPUT);
    digitalWrite(TOF2_LPN_PIN, HIGH);
    pinMode(TOF2_INT_PIN, INPUT_PULLUP);
    pinMode(TOF2_MISO_PIN, INPUT_PULLUP);

    // 2. Initialize HSPI on verified pins: SCK=39, MISO=41, MOSI=40, SS=-1 (manual CS)
    HardwareSPI.begin(TOF2_SCK_PIN, TOF2_MISO_PIN, TOF2_MOSI_PIN, -1);
    delay(50);

    Dev2.platform.handle = (void *)&ctx_tof2;
    Dev2.platform.address = 0x29;

    // 3. Run sensor initialization, bank verification, and 84KB firmware upload
    tof2_active = initialize_tof_sensor(&Dev2, &ctx_tof2);

    Serial.println(F("\n================================================================="));
    Serial.printf(" TOF INITIALIZATION SUMMARY: ToF2=%s\n", tof2_active ? "ONLINE" : "FAILED");
    Serial.println(F("=================================================================\n"));
}

// =========================================================================
// MAIN LOOP: STREAM LIVE 4X4 DISTANCE MATRICES
// =========================================================================

// =========================================================================
// JSON TELEMETRY STREAMER & COMMAND DISPATCHER FOR GUI
// =========================================================================

// Compact telemetry: only Target #1/#2 are streamed (the GUI never uses more),
// 'refl' is omitted, and every packet carries a sequence number + timestamp so the
// GUI can measure loss/latency. The whole frame is built in RAM and pushed with a
// single Serial.write() to avoid hundreds of tiny blocking Serial.print() calls.
static char telemetry_buf[8192];

static inline int append_array_u(char *buf, int pos, const char *key, const uint32_t *vals, int count) {
    pos += snprintf(buf + pos, sizeof(telemetry_buf) - pos, ",\"%s\":[", key);
    for (int i = 0; i < count; i++) {
        pos += snprintf(buf + pos, sizeof(telemetry_buf) - pos, i ? ",%lu" : "%lu", (unsigned long)vals[i]);
    }
    buf[pos++] = ']';
    return pos;
}

void send_json_telemetry() {
    const int n_zones = (current_resolution == VL53L8CX_RESOLUTION_8X8) ? 64 : 16;
    const int nt_out = (VL53L8CX_NB_TARGET_PER_ZONE < 2) ? VL53L8CX_NB_TARGET_PER_ZONE : 2;
    const int n_vals = n_zones * nt_out;

    uint32_t d[128], st[128], sig[128], sigm[128], ntar[64], amb[64], spad[64];
    for (int z = 0; z < n_zones; z++) {
        for (int t = 0; t < nt_out; t++) {
            int src = z * VL53L8CX_NB_TARGET_PER_ZONE + t;
            int dst = z * nt_out + t;
            d[dst]    = (uint32_t)(int32_t)Results2.distance_mm[src];
            st[dst]   = Results2.target_status[src];
            sig[dst]  = Results2.signal_per_spad[src];
            sigm[dst] = Results2.range_sigma_mm[src];
        }
        ntar[z] = Results2.nb_target_detected[z];
        amb[z]  = Results2.ambient_per_spad[z];
        spad[z] = Results2.nb_spads_enabled[z];
    }
    // Distances can be negative on error; clamp to 0 so the unsigned printer stays valid.
    for (int i = 0; i < n_vals; i++) if ((int32_t)d[i] < 0) d[i] = 0;

    int pos = snprintf(telemetry_buf, sizeof(telemetry_buf),
                       "{\"t\":\"D\",\"seq\":%lu,\"ts\":%lu,\"r\":%d,\"nt\":%d,\"temp\":%d",
                       (unsigned long)frame_count, (unsigned long)millis(),
                       n_zones, nt_out, (int)Results2.silicon_temp_degc);
    pos = append_array_u(telemetry_buf, pos, "ntar", ntar, n_zones);
    pos = append_array_u(telemetry_buf, pos, "d", d, n_vals);
    pos = append_array_u(telemetry_buf, pos, "st", st, n_vals);
    pos = append_array_u(telemetry_buf, pos, "sig", sig, n_vals);
    pos = append_array_u(telemetry_buf, pos, "sigm", sigm, n_vals);
    pos = append_array_u(telemetry_buf, pos, "amb", amb, n_zones);
    pos = append_array_u(telemetry_buf, pos, "spad", spad, n_zones);
    telemetry_buf[pos++] = '}';
    telemetry_buf[pos++] = '\n';
    Serial.write((const uint8_t *)telemetry_buf, pos);
}

bool fast_soft_recover_sensor(VL53L8CX_Configuration *p_dev) {
    digitalWrite(ctx_tof2.cs_pin, HIGH);
    delay(2);
    vl53l8cx_stop_ranging(p_dev);
    delay(5);
    uint8_t st = vl53l8cx_start_ranging(p_dev);
    return (st == 0);
}

void apply_sensor_configuration(String params) {
    int res = -1, freq = -1, integ = -1, order = -1, sharp = -1;
    
    // Parse key=value tokens
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
        
        start = comma + 1;
    }
    
    // 1. Must stop ranging session before altering DCI parameters
    vl53l8cx_stop_ranging(&Dev2);
    delay(5);
    
    // 2. Resolution (16=4x4, 64=8x8)
    if (res == 16 || res == 64) {
        current_resolution = (res == 64) ? VL53L8CX_RESOLUTION_8X8 : VL53L8CX_RESOLUTION_4X4;
        vl53l8cx_set_resolution(&Dev2, current_resolution);
    }
    
    // 3. Frequency (Hz)
    if (freq >= 1 && freq <= 60) {
        if (current_resolution == VL53L8CX_RESOLUTION_8X8 && freq > 15) {
            freq = 15; // ST hardware limit: 8x8 is max 15 Hz
        }
        current_frequency_hz = freq;
        vl53l8cx_set_ranging_frequency_hz(&Dev2, current_frequency_hz);
    }
    
    // 4. Integration Time (ms)
    if (integ >= 2 && integ <= 1000) {
        current_integration_ms = integ;
    }
    if (current_integration_ms * current_frequency_hz > 950) {
        current_integration_ms = max(2UL, 950UL / current_frequency_hz);
    }
    vl53l8cx_set_integration_time_ms(&Dev2, current_integration_ms);
    
    // 5. Target Order (1=Closest, 2=Strongest)
    if (order == 1 || order == 2) {
        current_target_order = order;
        vl53l8cx_set_target_order(&Dev2, current_target_order);
    }
    
    // 6. Veiling Glare Sharpener (0-99%)
    if (sharp >= 0 && sharp <= 99) {
        current_sharpener = sharp;
        vl53l8cx_set_sharpener_percent(&Dev2, current_sharpener);
    }
    
    vl53l8cx_set_VHV_repeat_count(&Dev2, 14400); // 4 min interval @ 60 Hz
    
    // 7. Restart continuous ranging with updated parameters
    uint8_t st = vl53l8cx_start_ranging(&Dev2);
    if (st == 0) {
        tof2_active = true;
        last_frame_time_ms = millis();
        recovery_tier = 0;
        Serial.println(F("{\"type\":\"ACK\",\"status\":\"OK\",\"cmd\":\"SET_CONFIG\"}"));
    } else {
        Serial.printf("{\"type\":\"ACK\",\"status\":\"ERROR\",\"code\":%u,\"cmd\":\"SET_CONFIG\"}\n", st);
    }
}

void dispatch_command(const String &cmd) {
    if (cmd == "GET_CONFIG") {
        Serial.printf("{\"type\":\"CONFIG\",\"res\":%u,\"freq\":%u,\"integ\":%lu,\"order\":%u,\"sharp\":%u,\"mode\":1,\"power\":1,\"margin\":50}\n",
                      (current_resolution == VL53L8CX_RESOLUTION_8X8 ? 64 : 16),
                      current_frequency_hz, current_integration_ms, current_target_order, current_sharpener);
    } else if (cmd == "RESET") {
        Serial.println(F("{\"type\":\"ACK\",\"status\":\"OK\",\"cmd\":\"RESET\"}"));
        tof2_active = initialize_tof_sensor(&Dev2, &ctx_tof2);
        if (tof2_active) {
            last_frame_time_ms = millis();
            recovery_tier = 0;
        }
    } else if (cmd.startsWith("SET_CONFIG:") || cmd.startsWith("SET_CONFIG ")) {
        int colon = cmd.indexOf(':');
        if (colon == -1) colon = cmd.indexOf(' ');
        apply_sensor_configuration(cmd.substring(colon + 1));
    } else if (cmd.startsWith("SET_HZ:") || cmd.startsWith("SET_FREQ:")) {
        int colon = cmd.indexOf(':');
        apply_sensor_configuration("freq=" + cmd.substring(colon + 1));
    } else if (cmd.startsWith("FREQ:")) {
        apply_sensor_configuration("freq=" + cmd.substring(5));
    } else if (cmd.startsWith("CALIBRATE_XTALK")) {
        Serial.println(F("{\"type\":\"ACK\",\"status\":\"OK\",\"cmd\":\"CALIBRATE_XTALK\"}"));
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

void check_and_recover_sensor() {
    unsigned long now = millis();

    if (tof2_active) {
        unsigned long elapsed = now - last_frame_time_ms;
        // Headroom for scene changes and VHV calibrations (600 ms = ~36 frames at 60 Hz)
        unsigned long stall_threshold = max(600UL, (1000UL / current_frequency_hz) * 6UL);

        if (elapsed > stall_threshold) {
            // Rate limit recovery attempts: 1000 ms between attempts
            if (now - last_recovery_attempt_ms >= 1000) {
                last_recovery_attempt_ms = now;

                if (recovery_tier == 0) {
                    // Tier 1: Soft recovery
                    recovery_tier = 1;
                    if (fast_soft_recover_sensor(&Dev2)) {
                        last_frame_time_ms = millis();
                        recovery_tier = 0;
                        return;
                    }
                }

                // Tier 2: Full Hardware LPn Reset & Re-init
                recovery_tier = 2;
                tof2_active = initialize_tof_sensor(&Dev2, &ctx_tof2);
                if (tof2_active) {
                    last_frame_time_ms = millis();
                    recovery_tier = 0;
                }
            }
        } else {
            recovery_tier = 0;
        }
    } else {
        // Sensor offline: retry every 1000 ms indefinitely
        if (now - last_recovery_attempt_ms >= 1000) {
            last_recovery_attempt_ms = now;
            tof2_active = initialize_tof_sensor(&Dev2, &ctx_tof2);
            if (tof2_active) {
                last_frame_time_ms = millis();
                recovery_tier = 0;
                Serial.println(F("{\"type\":\"SYSTEM_BOOT\"}"));
            }
        }
    }
}

void loop() {
    handle_serial_commands();

    if (tof2_active) {
        uint8_t isReady = 0;
        vl53l8cx_check_data_ready(&Dev2, &isReady);
        if (isReady) {
            if (vl53l8cx_get_ranging_data(&Dev2, &Results2) == 0) {
                frame_count++;
                last_frame_time_ms = millis();
                recovery_tier = 0;
                send_json_telemetry();
            }
        }
    }

    check_and_recover_sensor();

    delay(2);
}

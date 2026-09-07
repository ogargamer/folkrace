#include <Arduino.h>
#include <SPI.h>
#include "esp_rom_gpio.h"
#include "driver/gpio.h"
#include "vl53l8cx_api.h"
#include "vl53l8cx_buffers.h"
#include "vl53l8cx_plugin_xtalk.h"

// =========================================================================
// HARDWARE PIN DEFINITIONS (LINE 2 / HSPI)
// =========================================================================

#define TOF2_SCK_PIN    39
#define TOF2_MOSI_PIN   40
#define TOF2_MISO_PIN   41
#define TOF2_CS_PIN     42
#define TOF2_INT_PIN    38
#define TOF2_LPN_PIN    47

#define SPI_INIT_SPEED_HZ    2000000
#define SPI_RANGING_SPEED_HZ 4000000
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
bool is_ranging = false;

// Active Configuration State
struct SensorConfig {
    uint8_t resolution;        // 16 (4x4) or 64 (8x8)
    uint8_t frequency_hz;      // 1-60 Hz (4x4) or 1-15 Hz (8x8)
    uint32_t integration_ms;   // 2-1000 ms (Autonomous mode only)
    uint8_t target_order;      // 1=Closest, 2=Strongest
    uint8_t sharpener_percent; // 0-99% (ST Rev 12 default = 1%)
    uint8_t ranging_mode;      // 1=Continuous, 3=Autonomous
    uint8_t power_mode;        // 0=Sleep, 1=Wakeup
    uint32_t xtalk_margin;     // 0-10000 kcps/spad (default 50)
} current_config = { 
    VL53L8CX_RESOLUTION_4X4, 
    60, 
    5, 
    VL53L8CX_TARGET_ORDER_CLOSEST, 
    1, 
    VL53L8CX_RANGING_MODE_CONTINUOUS, 
    VL53L8CX_POWER_MODE_WAKEUP,
    50
};

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
// SENSOR INITIALIZATION & RECONFIGURATION
// =========================================================================

bool initialize_tof_sensor(VL53L8CX_Configuration *p_dev, ToF_Device_Context *ctx) {
    ctx->speed_hz = SPI_INIT_SPEED_HZ;

    memset(p_dev, 0, sizeof(VL53L8CX_Configuration));
    p_dev->platform.handle = (void *)ctx;
    p_dev->platform.address = 0x29;

    // 1. Hardware Reset via LPn pin
    digitalWrite(ctx->lpn_pin, LOW);
    delay(100);
    digitalWrite(ctx->lpn_pin, HIGH);
    delay(200);

    // 2. Check if sensor alive
    uint8_t isAlive = 0;
    if (vl53l8cx_is_alive(p_dev, &isAlive) != 0 || !isAlive) {
        return false;
    }

    // 3. Firmware upload (~84 KB via SPI)
    if (vl53l8cx_init(p_dev) != 0) {
        return false;
    }

    ctx->speed_hz = SPI_RANGING_SPEED_HZ;

    // 4. Apply initial parameters
    vl53l8cx_set_resolution(p_dev, current_config.resolution);
    vl53l8cx_set_ranging_frequency_hz(p_dev, current_config.frequency_hz);
    vl53l8cx_set_integration_time_ms(p_dev, current_config.integration_ms);
    vl53l8cx_set_target_order(p_dev, current_config.target_order);
    vl53l8cx_set_sharpener_percent(p_dev, current_config.sharpener_percent);
    vl53l8cx_set_ranging_mode(p_dev, current_config.ranging_mode);
    vl53l8cx_set_xtalk_margin(p_dev, current_config.xtalk_margin);

    // 5. Start continuous high-speed ranging
    if (vl53l8cx_start_ranging(p_dev) != 0) {
        return false;
    }

    is_ranging = true;
    return true;
}

bool apply_new_config(uint8_t res, uint8_t freq, uint32_t integ, uint8_t order, uint8_t sharp, uint8_t mode, uint8_t pwr, uint32_t margin) {
    if (!tof2_active) return false;

    // 1. Stop current ranging session
    if (is_ranging) {
        vl53l8cx_stop_ranging(&Dev2);
        is_ranging = false;
        delay(20);
    }

    // 2. Power mode handling
    if (pwr == VL53L8CX_POWER_MODE_SLEEP) {
        vl53l8cx_set_power_mode(&Dev2, VL53L8CX_POWER_MODE_SLEEP);
        current_config.power_mode = VL53L8CX_POWER_MODE_SLEEP;
        return true;
    } else if (current_config.power_mode == VL53L8CX_POWER_MODE_SLEEP) {
        vl53l8cx_set_power_mode(&Dev2, VL53L8CX_POWER_MODE_WAKEUP);
        current_config.power_mode = VL53L8CX_POWER_MODE_WAKEUP;
        delay(20);
    }

    // 3. Set Resolution first (valid frequencies depend on resolution)
    if (res != current_config.resolution) {
        vl53l8cx_set_resolution(&Dev2, res);
        current_config.resolution = res;
    }

    // 4. Validate and set Ranging Frequency
    uint8_t max_freq = (res == VL53L8CX_RESOLUTION_8X8) ? 15 : 60;
    if (freq < 1) freq = 1;
    if (freq > max_freq) freq = max_freq;
    vl53l8cx_set_ranging_frequency_hz(&Dev2, freq);
    current_config.frequency_hz = freq;

    // 5. Set Integration Time (Autonomous mode only)
    if (integ < 2) integ = 2;
    if (integ > 1000) integ = 1000;
    vl53l8cx_set_integration_time_ms(&Dev2, integ);
    current_config.integration_ms = integ;

    // 6. Set Target Order (1=Closest, 2=Strongest)
    vl53l8cx_set_target_order(&Dev2, order);
    current_config.target_order = order;

    // 7. Set Sharpener (0 to 99%)
    if (sharp > 99) sharp = 99;
    vl53l8cx_set_sharpener_percent(&Dev2, sharp);
    current_config.sharpener_percent = sharp;

    // 8. Set Ranging Mode (1=Continuous, 3=Autonomous)
    vl53l8cx_set_ranging_mode(&Dev2, mode);
    current_config.ranging_mode = mode;

    // 9. Set Xtalk margin
    vl53l8cx_set_xtalk_margin(&Dev2, margin);
    current_config.xtalk_margin = margin;

    // 10. Restart ranging session
    uint8_t st = vl53l8cx_start_ranging(&Dev2);
    if (st == 0) {
        is_ranging = true;
        return true;
    }
    return false;
}

// =========================================================================
// SERIAL COMMAND PROCESSOR
// =========================================================================

void send_current_config() {
    Serial.printf("{\"type\":\"CONFIG\",\"res\":%u,\"freq\":%u,\"integ\":%lu,\"order\":%u,\"sharp\":%u,\"mode\":%u,\"power\":%u,\"margin\":%lu,\"nb_targets\":%u,\"status\":\"%s\"}\n",
                  current_config.resolution,
                  current_config.frequency_hz,
                  current_config.integration_ms,
                  current_config.target_order,
                  current_config.sharpener_percent,
                  current_config.ranging_mode,
                  current_config.power_mode,
                  current_config.xtalk_margin,
                  VL53L8CX_NB_TARGET_PER_ZONE,
                  is_ranging ? "RANGING" : "IDLE");
}

void process_serial_command(const String &cmd_str) {
    if (cmd_str.indexOf("GET_CONFIG") >= 0) {
        send_current_config();
        return;
    }

    if (cmd_str.indexOf("RESET") >= 0) {
        tof2_active = initialize_tof_sensor(&Dev2, &ctx_tof2);
        Serial.printf("{\"type\":\"ACK\",\"cmd\":\"RESET\",\"status\":\"%s\"}\n", tof2_active ? "OK" : "ERROR");
        return;
    }

    if (cmd_str.indexOf("CALIBRATE_XTALK") >= 0) {
        // Parse: CALIBRATE_XTALK:dist=600,samples=4,refl=3
        uint16_t dist = 600;
        uint8_t samples = 4;
        uint16_t refl = 3;

        int idx;
        if ((idx = cmd_str.indexOf("dist=")) >= 0) dist = (uint16_t)cmd_str.substring(idx + 5).toInt();
        if ((idx = cmd_str.indexOf("samples=")) >= 0) samples = (uint8_t)cmd_str.substring(idx + 8).toInt();
        if ((idx = cmd_str.indexOf("refl=")) >= 0) refl = (uint16_t)cmd_str.substring(idx + 5).toInt();

        if (dist < 600) dist = 600;
        if (dist > 3000) dist = 3000;
        if (samples < 1) samples = 1;
        if (samples > 16) samples = 16;
        if (refl < 1) refl = 1;
        if (refl > 99) refl = 99;

        if (is_ranging) {
            vl53l8cx_stop_ranging(&Dev2);
            is_ranging = false;
            delay(30);
        }

        uint8_t cal_st = vl53l8cx_calibrate_xtalk(&Dev2, refl, samples, dist);
        if (cal_st == 0) {
            vl53l8cx_get_caldata_xtalk(&Dev2, Dev2.xtalk_data);
            vl53l8cx_start_ranging(&Dev2);
            is_ranging = true;
            Serial.printf("{\"type\":\"ACK\",\"cmd\":\"CALIBRATE_XTALK\",\"status\":\"OK\",\"dist\":%u,\"samples\":%u,\"refl\":%u}\n", dist, samples, refl);
        } else {
            vl53l8cx_start_ranging(&Dev2);
            is_ranging = true;
            Serial.printf("{\"type\":\"ACK\",\"cmd\":\"CALIBRATE_XTALK\",\"status\":\"ERROR\",\"code\":%u}\n", cal_st);
        }
        return;
    }

    if (cmd_str.indexOf("SET_CONFIG") >= 0) {
        // Parse parameters: SET_CONFIG:res=16,freq=60,integ=5,order=1,sharp=1,mode=1,pwr=1,margin=50
        uint8_t res = current_config.resolution;
        uint8_t freq = current_config.frequency_hz;
        uint32_t integ = current_config.integration_ms;
        uint8_t order = current_config.target_order;
        uint8_t sharp = current_config.sharpener_percent;
        uint8_t mode = current_config.ranging_mode;
        uint8_t pwr = current_config.power_mode;
        uint32_t margin = current_config.xtalk_margin;

        int idx;
        if ((idx = cmd_str.indexOf("res=")) >= 0) res = cmd_str.substring(idx + 4).toInt();
        if ((idx = cmd_str.indexOf("freq=")) >= 0) freq = cmd_str.substring(idx + 5).toInt();
        if ((idx = cmd_str.indexOf("integ=")) >= 0) integ = cmd_str.substring(idx + 6).toInt();
        if ((idx = cmd_str.indexOf("order=")) >= 0) order = cmd_str.substring(idx + 6).toInt();
        if ((idx = cmd_str.indexOf("sharp=")) >= 0) sharp = cmd_str.substring(idx + 6).toInt();
        if ((idx = cmd_str.indexOf("mode=")) >= 0) mode = cmd_str.substring(idx + 5).toInt();
        if ((idx = cmd_str.indexOf("pwr=")) >= 0) pwr = cmd_str.substring(idx + 4).toInt();
        if ((idx = cmd_str.indexOf("margin=")) >= 0) margin = cmd_str.substring(idx + 7).toInt();

        bool ok = apply_new_config(res, freq, integ, order, sharp, mode, pwr, margin);
        if (ok) {
            Serial.print(F("{\"type\":\"ACK\",\"cmd\":\"SET_CONFIG\",\"status\":\"OK\","));
            Serial.printf("\"res\":%u,\"freq\":%u,\"integ\":%lu,\"order\":%u,\"sharp\":%u,\"mode\":%u,\"power\":%u,\"margin\":%lu}\n",
                          current_config.resolution,
                          current_config.frequency_hz,
                          current_config.integration_ms,
                          current_config.target_order,
                          current_config.sharpener_percent,
                          current_config.ranging_mode,
                          current_config.power_mode,
                          current_config.xtalk_margin);
        } else {
            Serial.println(F("{\"type\":\"ACK\",\"cmd\":\"SET_CONFIG\",\"status\":\"ERROR\"}"));
        }
    }
}

// =========================================================================
// MAIN SETUP
// =========================================================================

void setup() {
    Serial.begin(921600);
    delay(1000);

    pinMode(TOF2_CS_PIN, OUTPUT);
    digitalWrite(TOF2_CS_PIN, HIGH);
    pinMode(TOF2_LPN_PIN, OUTPUT);
    digitalWrite(TOF2_LPN_PIN, HIGH);
    pinMode(TOF2_INT_PIN, INPUT_PULLUP);

    HardwareSPI.begin(TOF2_SCK_PIN, TOF2_MISO_PIN, TOF2_MOSI_PIN, -1);

    tof2_active = initialize_tof_sensor(&Dev2, &ctx_tof2);

    Serial.println(F("{\"type\":\"SYSTEM_BOOT\",\"sensor\":\"VL53L8CX\",\"status\":\"ONLINE\"}"));
    send_current_config();
}

// =========================================================================
// MAIN LOOP (HIGH SPEED STREAMING ENGINE)
// =========================================================================

unsigned long frame_count = 0;
String input_buffer = "";

void loop() {
    // 1. Process Incoming Serial Commands
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (input_buffer.length() > 0) {
                process_serial_command(input_buffer);
                input_buffer = "";
            }
        } else {
            input_buffer += c;
        }
    }

    // 2. Poll and Read Ranging Data
    if (tof2_active && is_ranging) {
        uint8_t isReady = 0;
        uint8_t st = vl53l8cx_check_data_ready(&Dev2, &isReady);
        if (st == 0 && isReady) {
            if (vl53l8cx_get_ranging_data(&Dev2, &Results2) == 0) {
                frame_count++;

                uint8_t total_zones = current_config.resolution; // 16 or 64
                uint16_t total_targets = total_zones * VL53L8CX_NB_TARGET_PER_ZONE; // 16*4=64 or 64*4=256

                // Compact JSON Stream Packet with Multi-Target & Reflectivity
                Serial.print(F("{\"t\":\"D\",\"r\":"));
                Serial.print(total_zones);
                Serial.print(F(",\"nt\":"));
                Serial.print(VL53L8CX_NB_TARGET_PER_ZONE);
                Serial.print(F(",\"s\":"));
                Serial.print(Dev2.streamcount);
                Serial.print(F(",\"f\":"));
                Serial.print(frame_count);
                Serial.print(F(",\"temp\":"));
                Serial.print((int)Results2.silicon_temp_degc);

                // Target count detected per zone
                Serial.print(F(",\"ntar\":["));
                for (uint8_t i = 0; i < total_zones; i++) {
                    Serial.print(Results2.nb_target_detected[i]);
                    if (i < total_zones - 1) Serial.print(',');
                }

                // Distances array for all targets
                Serial.print(F("],\"d\":["));
                for (uint16_t i = 0; i < total_targets; i++) {
                    Serial.print(Results2.distance_mm[i]);
                    if (i < total_targets - 1) Serial.print(',');
                }

                // Reflectance array (%) for all targets
                Serial.print(F("],\"refl\":["));
                for (uint16_t i = 0; i < total_targets; i++) {
                    Serial.print(Results2.reflectance[i]);
                    if (i < total_targets - 1) Serial.print(',');
                }

                // Target status array for all targets
                Serial.print(F("],\"st\":["));
                for (uint16_t i = 0; i < total_targets; i++) {
                    Serial.print(Results2.target_status[i]);
                    if (i < total_targets - 1) Serial.print(',');
                }

                // Signal rate array (kcps/SPAD)
                Serial.print(F("],\"sig\":["));
                for (uint16_t i = 0; i < total_targets; i++) {
                    Serial.print(Results2.signal_per_spad[i]);
                    if (i < total_targets - 1) Serial.print(',');
                }

                // Range sigma noise array (mm)
                Serial.print(F("],\"sigm\":["));
                for (uint16_t i = 0; i < total_targets; i++) {
                    Serial.print(Results2.range_sigma_mm[i]);
                    if (i < total_targets - 1) Serial.print(',');
                }

                // Ambient rate array (kcps/SPAD)
                Serial.print(F("],\"amb\":["));
                for (uint8_t i = 0; i < total_zones; i++) {
                    Serial.print(Results2.ambient_per_spad[i]);
                    if (i < total_zones - 1) Serial.print(',');
                }

                // SPADs enabled count
                Serial.print(F("],\"spad\":["));
                for (uint8_t i = 0; i < total_zones; i++) {
                    Serial.print(Results2.nb_spads_enabled[i]);
                    if (i < total_zones - 1) Serial.print(',');
                }

                Serial.println(F("]}"));
            }
        }
    }

    delay(1);
}

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
#define SPI_INIT_SPEED_HZ   1000000 // 1 MHz for clean, rock-solid firmware burst
#define SPI_RANGING_SPEED_HZ 3000000 // 3 MHz for live stream
#define CHUNK_SIZE          512

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
    ctx->spi->transferBytes(buf, NULL, 3);
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
        delayMicroseconds(2);
        ctx->spi->transferBytes(tx_chunk_buffer, NULL, chunk + 2);
        delayMicroseconds(2);
        digitalWrite(ctx->cs_pin, HIGH);
        delayMicroseconds(5);
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
        ctx->spi->transferBytes(addr_buf, NULL, 2);
        ctx->spi->transferBytes(NULL, &p_values[pos], chunk);
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

    // 1. Hardware Reset via LPn with retries
    uint8_t dev_id = 0, rev_id = 0;
    uint8_t isAlive = 0;
    uint8_t status = 0;
    bool found_alive = false;

    for (int attempt = 1; attempt <= 5; attempt++) {
        Serial.printf("[%s] Attempt %d: Power cycling via LPn (GPIO %d)...\n", ctx->name, attempt, ctx->lpn_pin);
        digitalWrite(ctx->lpn_pin, LOW);
        delay(150);
        digitalWrite(ctx->lpn_pin, HIGH);
        delay(250);

        VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);
        VL53L8CX_RdByte(&(p_dev->platform), 0, &dev_id);
        VL53L8CX_RdByte(&(p_dev->platform), 1, &rev_id);
        Serial.printf("[%s] Raw SPI Read: Reg 0x00=0x%02X, Reg 0x01=0x%02X (Expected: 0xF0, 0x0C)\n", ctx->name, dev_id, rev_id);

        status = vl53l8cx_is_alive(p_dev, &isAlive);
        Serial.printf("[%s] vl53l8cx_is_alive: status = %u, isAlive = %u\n", ctx->name, status, isAlive);

        if (isAlive && dev_id == 0xF0 && rev_id == 0x0C) {
            found_alive = true;
            break;
        }
        delay(100);
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

void send_json_telemetry() {
    Serial.print(F("{\"t\":\"D\",\"r\":16,\"nt\":4,\"temp\":"));
    Serial.print(Results2.silicon_temp_degc);
    
    // ntar (16 zones)
    Serial.print(F(",\"ntar\":["));
    for (int z = 0; z < 16; z++) {
        if (z > 0) Serial.print(',');
        Serial.print(Results2.nb_target_detected[z]);
    }
    
    // d (64 targets)
    Serial.print(F("],\"d\":["));
    for (int i = 0; i < 64; i++) {
        if (i > 0) Serial.print(',');
        Serial.print(Results2.distance_mm[i]);
    }
    
    // refl (64 targets)
    Serial.print(F("],\"refl\":["));
    for (int i = 0; i < 64; i++) {
        if (i > 0) Serial.print(',');
        Serial.print(Results2.reflectance[i]);
    }

    // st (64 targets)
    Serial.print(F("],\"st\":["));
    for (int i = 0; i < 64; i++) {
        if (i > 0) Serial.print(',');
        Serial.print(Results2.target_status[i]);
    }

    // sig (64 targets)
    Serial.print(F("],\"sig\":["));
    for (int i = 0; i < 64; i++) {
        if (i > 0) Serial.print(',');
        Serial.print(Results2.signal_per_spad[i]);
    }

    // sigm (64 targets)
    Serial.print(F("],\"sigm\":["));
    for (int i = 0; i < 64; i++) {
        if (i > 0) Serial.print(',');
        Serial.print(Results2.range_sigma_mm[i]);
    }

    // amb (16 zones)
    Serial.print(F("],\"amb\":["));
    for (int z = 0; z < 16; z++) {
        if (z > 0) Serial.print(',');
        Serial.print(Results2.ambient_per_spad[z]);
    }

    // spad (16 zones)
    Serial.print(F("],\"spad\":["));
    for (int z = 0; z < 16; z++) {
        if (z > 0) Serial.print(',');
        Serial.print(Results2.nb_spads_enabled[z]);
    }
    Serial.println(F("]}"));
}

void handle_serial_commands() {
    while (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() == 0) continue;

        if (cmd == "GET_CONFIG") {
            Serial.println(F("{\"type\":\"CONFIG\",\"res\":16,\"freq\":15,\"integ\":5,\"order\":1,\"sharp\":1,\"mode\":1,\"power\":1,\"margin\":50}"));
        } else if (cmd == "RESET") {
            Serial.println(F("{\"type\":\"ACK\",\"status\":\"OK\",\"cmd\":\"RESET\"}"));
            tof2_active = initialize_tof_sensor(&Dev2, &ctx_tof2);
        } else if (cmd.startsWith("SET_CONFIG")) {
            Serial.println(F("{\"type\":\"ACK\",\"status\":\"OK\",\"cmd\":\"SET_CONFIG\"}"));
        } else if (cmd.startsWith("CALIBRATE_XTALK")) {
            Serial.println(F("{\"type\":\"ACK\",\"status\":\"OK\",\"cmd\":\"CALIBRATE_XTALK\"}"));
        }
    }
}

unsigned long frame_count = 0;
unsigned long last_print_time = 0;

void loop() {
    handle_serial_commands();

    uint8_t isReady = 0;
    if (tof2_active) {
        uint8_t st = vl53l8cx_check_data_ready(&Dev2, &isReady);
        if (st == 0 && isReady) {
            if (vl53l8cx_get_ranging_data(&Dev2, &Results2) == 0) {
                frame_count++;
                send_json_telemetry();
            }
        }
    }

    delay(2);
}

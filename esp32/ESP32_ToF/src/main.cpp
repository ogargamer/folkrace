#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "vl53l8cx_api.h"
#include "vl53l8cx_buffers.h"
#include <Arduino.h>
#include <SPI.h>

#define ENABLE_TOF1 1
#define ENABLE_TOF2 1
#define ENABLE_TOF3 1
#define ENABLE_TOF4 1

// --- Shared SPI3 Bus Lines ---
#define TOF_SHARED_SCK_PIN 13
#define TOF_SHARED_MOSI_PIN 14

// --- Per-Sensor Pins ---
#define TOF1_CS_PIN 7
#define TOF1_INT_PIN 6
#define TOF1_LPN_PIN 5
#define TOF1_MISO_PIN 4

#define TOF2_CS_PIN 12
#define TOF2_INT_PIN 11
#define TOF2_LPN_PIN 10
#define TOF2_MISO_PIN 9

#define TOF3_CS_PIN 41
#define TOF3_INT_PIN 42
#define TOF3_LPN_PIN 2
#define TOF3_MISO_PIN 1

#define TOF4_CS_PIN 17
#define TOF4_INT_PIN 18
#define TOF4_LPN_PIN 16
#define TOF4_MISO_PIN 15

#define SPI_INIT_SPEED_HZ 2000000
#define SPI_RANGING_SPEED_HZ 3000000
#define CHUNK_SIZE 1024

#define NUM_SENSORS 4
#define TOF_STALL_TIMEOUT_MS 600
#define TOF_SOFT_RECOVERY_COOLDOWN_MS 500
#define TOF_HARD_RECOVERY_COOLDOWN_MS 2000
#define TOF_HARD_RECOVERY_MAX_COOLDOWN_MS 30000

// Set to 1 only when NOT parsing the binary telemetry stream (breaks framing).
#define TOF_DEBUG_ASCII 0

// ASCII logs are always allowed during setup() (before binary streaming
// starts).
static bool g_streaming = false;
#define TOF_LOG(...)                                                           \
  do {                                                                         \
    if (TOF_DEBUG_ASCII || !g_streaming)                                       \
      Serial.printf(__VA_ARGS__);                                              \
  } while (0)

struct ToF_Device_Context {
  SPIClass *spi;
  uint8_t sensor_id;
  int cs_pin;
  int lpn_pin;
  int int_pin;
  int sck_pin;
  int miso_pin;
  int mosi_pin;
  uint32_t speed_hz;
  const char *name;

  bool enabled;
  bool active;      // Currently streaming
  bool first_frame; // Disambiguates initial state from streamcount rollover
  uint8_t last_streamcount;
  uint8_t soft_fail_streak;
  uint8_t hard_fail_count; // consecutive failed hard recoveries (backoff)

  unsigned long frame_count;
  unsigned long last_frame_ms;
  unsigned long last_poll_ms;
  unsigned long last_recovery_ms;
};

SPIClass HardwareSPI(HSPI);

#define MAKE_CTX(id, en, pfx)                                                  \
  {&HardwareSPI,                                                               \
   id,                                                                         \
   pfx##_CS_PIN,                                                               \
   pfx##_LPN_PIN,                                                              \
   pfx##_INT_PIN,                                                              \
   TOF_SHARED_SCK_PIN,                                                         \
   pfx##_MISO_PIN,                                                             \
   TOF_SHARED_MOSI_PIN,                                                        \
   SPI_INIT_SPEED_HZ,                                                          \
   "ToF " #id,                                                                 \
   en,                                                                         \
   false,                                                                      \
   true,                                                                       \
   0,                                                                          \
   0,                                                                          \
   0,                                                                          \
   0,                                                                          \
   0,                                                                          \
   0,                                                                          \
   0}

ToF_Device_Context ctxs[NUM_SENSORS] = {
    MAKE_CTX(1, ENABLE_TOF1, TOF1), MAKE_CTX(2, ENABLE_TOF2, TOF2),
    MAKE_CTX(3, ENABLE_TOF3, TOF3), MAKE_CTX(4, ENABLE_TOF4, TOF4)};

VL53L8CX_Configuration devs[NUM_SENSORS];
VL53L8CX_ResultsData results[NUM_SENSORS];

uint8_t current_resolution = VL53L8CX_RESOLUTION_4X4;
uint8_t current_frequency_hz = 60;
uint32_t current_integration_ms =
    10; // Used only if AUTONOMOUS mode is selected
uint8_t current_target_order = VL53L8CX_TARGET_ORDER_STRONGEST;
uint8_t current_sharpener = 1;

// Forward declarations
bool init_and_upload_firmware(VL53L8CX_Configuration *p_dev,
                              ToF_Device_Context *ctx);
bool init_sensor_with_retries(VL53L8CX_Configuration *p_dev,
                              ToF_Device_Context *ctx, int max_attempts = 3);

static int current_miso_pin = -1;

extern "C" {
void spiAttachMISO(spi_t *spi, int8_t miso);
}

static inline void switch_miso_pin(SPIClass *spi, int miso_pin) {
  if (miso_pin == current_miso_pin || miso_pin < 0)
    return;
  current_miso_pin = miso_pin;
  spiAttachMISO(spi->bus(), miso_pin);
  gpio_pullup_en((gpio_num_t)miso_pin);
  gpio_pulldown_dis((gpio_num_t)miso_pin);
}

// =========================================================================
// PLATFORM PAL IMPLEMENTATION
// =========================================================================

uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAddress,
                        uint8_t value) {
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
  ctx->spi->endTransaction();
  delayMicroseconds(3);
  return 0;
}

uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform,
                         uint16_t RegisterAddress, uint8_t *p_values,
                         uint32_t size) {
  ToF_Device_Context *ctx = (ToF_Device_Context *)p_platform->handle;
  uint8_t addr[2] = {(uint8_t)(((RegisterAddress | 0x8000) >> 8) & 0xFF),
                     (uint8_t)(RegisterAddress & 0xFF)};

  ctx->spi->beginTransaction(SPISettings(ctx->speed_hz, MSBFIRST, SPI_MODE3));
  digitalWrite(ctx->cs_pin, LOW);
  delayMicroseconds(5);
  ctx->spi->transferBytes(addr, NULL, 2);

  uint32_t pos = 0;
  while (pos < size) {
    uint32_t chunk = (size - pos > CHUNK_SIZE) ? CHUNK_SIZE : (size - pos);
    ctx->spi->transferBytes(&p_values[pos], NULL, chunk);
    pos += chunk;
    if (size > CHUNK_SIZE) {
      yield();
    }
  }

  delayMicroseconds(5);
  digitalWrite(ctx->cs_pin, HIGH);
  ctx->spi->endTransaction();
  delayMicroseconds(5);
  return 0;
}

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAddress,
                        uint8_t *p_value) {
  ToF_Device_Context *ctx = (ToF_Device_Context *)p_platform->handle;
  switch_miso_pin(ctx->spi, ctx->miso_pin);

  uint8_t tx_buf[3] = {(uint8_t)(((RegisterAddress & ~0x8000) >> 8) & 0xFF),
                       (uint8_t)(RegisterAddress & 0xFF), 0x00};
  uint8_t rx_buf[3] = {0};

  ctx->spi->beginTransaction(SPISettings(ctx->speed_hz, MSBFIRST, SPI_MODE3));
  digitalWrite(ctx->cs_pin, LOW);
  delayMicroseconds(5);
  ctx->spi->transferBytes(tx_buf, rx_buf, 3);
  delayMicroseconds(5);
  digitalWrite(ctx->cs_pin, HIGH);
  ctx->spi->endTransaction();
  *p_value = rx_buf[2];
  delayMicroseconds(3);
  return 0;
}

uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform,
                         uint16_t RegisterAddress, uint8_t *p_values,
                         uint32_t size) {
  ToF_Device_Context *ctx = (ToF_Device_Context *)p_platform->handle;
  switch_miso_pin(ctx->spi, ctx->miso_pin);

  uint8_t addr_buf[2];
  addr_buf[0] = (uint8_t)(((RegisterAddress & ~0x8000) >> 8) & 0xFF);
  addr_buf[1] = (uint8_t)(RegisterAddress & 0xFF);

  ctx->spi->beginTransaction(SPISettings(ctx->speed_hz, MSBFIRST, SPI_MODE3));
  digitalWrite(ctx->cs_pin, LOW);
  delayMicroseconds(5);
  ctx->spi->transferBytes(addr_buf, NULL, 2);
  // Send 0x00 on MOSI (NULL TX would clock out 0xFF)
  memset(p_values, 0, size);
  ctx->spi->transferBytes(p_values, p_values, size);
  delayMicroseconds(5);
  digitalWrite(ctx->cs_pin, HIGH);
  ctx->spi->endTransaction();
  delayMicroseconds(3);
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
// SENSOR INITIALIZATION
// =========================================================================

bool init_and_upload_firmware(VL53L8CX_Configuration *p_dev,
                              ToF_Device_Context *ctx) {
  TOF_LOG("\n>>> [INIT & FW UPLOAD] %s <<<\n", ctx->name);

  ctx->speed_hz = SPI_INIT_SPEED_HZ;
  ctx->first_frame = true;
  memset(p_dev, 0, sizeof(VL53L8CX_Configuration));
  p_dev->platform.handle = (void *)ctx;
  p_dev->platform.address = 0x29;

  switch_miso_pin(ctx->spi, ctx->miso_pin);

  VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);

  uint8_t dev_id = 0, rev_id = 0, isAlive = 0;
  bool found_alive = false;

  for (int attempt = 1; attempt <= 5; attempt++) {
    VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);
    VL53L8CX_RdByte(&(p_dev->platform), 0x0000, &dev_id);
    VL53L8CX_RdByte(&(p_dev->platform), 0x0001, &rev_id);
    vl53l8cx_is_alive(p_dev, &isAlive);

    if (isAlive && dev_id == 0xF0 && rev_id == 0x0C) {
      found_alive = true;
      break;
    }
    delay(20);
  }

  if (!found_alive) {
    TOF_LOG("[ERROR] %s not responding! (dev_id=0x%02X, rev_id=0x%02X, "
            "isAlive=%u)\n",
            ctx->name, dev_id, rev_id, isAlive);
    return false;
  }

  // Restore page 0 (vl53l8cx_is_alive leaves page register at 0x02)
  VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);

  uint8_t mcu_ready = 0;
  for (int i = 0; i < 50; i++) {
    VL53L8CX_RdByte(&(p_dev->platform), 0x0006, &mcu_ready);
    if (mcu_ready & 0x01)
      break;
    delay(5);
  }
  TOF_LOG("[%s] Alive OK. mcu_ready=0x%02X\n", ctx->name, mcu_ready);

  TOF_LOG("[%s] Uploading 84KB Firmware...\n", ctx->name);
  unsigned long t0 = millis();
  uint8_t status = vl53l8cx_init(p_dev);
  unsigned long upload_ms = millis() - t0;

  if (status != 0) {
    TOF_LOG("[ERROR] %s Firmware upload failed! status = %u\n", ctx->name,
            status);
    return false;
  }
  TOF_LOG("[%s] Init successful (%lu ms)\n", ctx->name, upload_ms);

  // Switch to 3 MHz for configuration and ranging
  ctx->speed_hz = SPI_RANGING_SPEED_HZ;

  status |= vl53l8cx_set_resolution(p_dev, current_resolution);
  // CRITICAL FOR 60HZ: Use CONTINUOUS ranging mode so there is zero
  // inter-measurement sleep/wakeup overhead.
  status |= vl53l8cx_set_ranging_mode(p_dev, VL53L8CX_RANGING_MODE_CONTINUOUS);
  status |= vl53l8cx_set_ranging_frequency_hz(p_dev, current_frequency_hz);
  status |= vl53l8cx_set_target_order(p_dev, current_target_order);
  status |= vl53l8cx_set_sharpener_percent(p_dev, current_sharpener);
  status |= vl53l8cx_set_VHV_repeat_count(p_dev, 14400);

  if (status != 0) {
    TOF_LOG("[WARN] %s config status non-zero: %u\n", ctx->name, status);
  }

  return true;
}

bool init_sensor_with_retries(VL53L8CX_Configuration *p_dev,
                              ToF_Device_Context *ctx, int max_attempts) {
  for (int attempt = 1; attempt <= max_attempts; attempt++) {
    if (attempt > 1) {
      ctx->speed_hz = 500000; // fallback to 500 kHz on retry
      TOF_LOG("[RETRY] %s Init attempt %d/%d (falling back to 500 kHz)...\n",
              ctx->name, attempt, max_attempts);
      digitalWrite(ctx->cs_pin, HIGH);
      digitalWrite(ctx->lpn_pin, LOW);
      delay(30);
      digitalWrite(ctx->lpn_pin, HIGH);
      delay(100);
    } else {
      ctx->speed_hz = SPI_INIT_SPEED_HZ;
    }

    if (init_and_upload_firmware(p_dev, ctx)) {
      return true;
    }

    delay(30);
  }

  TOF_LOG("[FATAL] %s failed to initialize after %d attempts.\n", ctx->name,
          max_attempts);
  return false;
}

// Flush stale frames and reset timers on all sensors after a blocking operation
static void resync_healthy_sensors() {
  unsigned long t = millis();
  for (int i = 0; i < NUM_SENSORS; i++) {
    ToF_Device_Context *c = &ctxs[i];
    if (!c->enabled)
      continue;
    c->last_frame_ms = t;
    c->last_poll_ms = t;

    // Drain a latched output buffer so INT can toggle again
    if (c->active && digitalRead(c->int_pin) == LOW) {
      if (vl53l8cx_get_ranging_data(&devs[i], &results[i]) ==
          VL53L8CX_STATUS_OK) {
        c->last_streamcount = devs[i].streamcount;
        c->first_frame = false;
      }
    }
  }
}

// Tier 1: fast soft restart (~15 ms, keeps firmware in RAM)
bool soft_recover_sensor(VL53L8CX_Configuration *p_dev,
                         ToF_Device_Context *ctx) {
  TOF_LOG("\n[RECOVERY-SOFT] Attempting fast restart on %s...\n", ctx->name);
  ctx->speed_hz = SPI_RANGING_SPEED_HZ;
  switch_miso_pin(ctx->spi, ctx->miso_pin);

  uint8_t isAlive = 0;
  if (vl53l8cx_is_alive(p_dev, &isAlive) != VL53L8CX_STATUS_OK || !isAlive) {
    return false;
  }

  vl53l8cx_stop_ranging(p_dev);
  delay(3);

  if (vl53l8cx_start_ranging(p_dev) == VL53L8CX_STATUS_OK) {
    ctx->first_frame = true;
    ctx->active = true;
    TOF_LOG("[RECOVERY-SOFT] %s resumed ranging!\n", ctx->name);
    return true;
  }
  return false;
}

// Tier 2: LPn reset + full firmware upload (~1.5 s)
bool hard_recover_sensor(VL53L8CX_Configuration *p_dev,
                         ToF_Device_Context *ctx) {
  TOF_LOG("\n[RECOVERY-HARD] %s resetting via LPn & reloading FW...\n",
          ctx->name);

  // Skip stop_ranging when recovering from a boot failure (no valid config)
  if (ctx->active) {
    vl53l8cx_stop_ranging(p_dev);
  }
  ctx->active = false;

  digitalWrite(ctx->cs_pin, HIGH);
  digitalWrite(ctx->lpn_pin, LOW);
  delay(15);
  digitalWrite(ctx->lpn_pin, HIGH);
  delay(30);

  // Repeat failures: one attempt only, to limit blocking of healthy sensors
  if (init_sensor_with_retries(p_dev, ctx, ctx->hard_fail_count ? 1 : 2)) {
    if (vl53l8cx_start_ranging(p_dev) == VL53L8CX_STATUS_OK) {
      ctx->first_frame = true;
      ctx->active = true;
      ctx->soft_fail_streak = 0;
      ctx->hard_fail_count = 0;
      TOF_LOG("[RECOVERY-HARD] %s recovered and streaming!\n", ctx->name);
      return true;
    }
  }

  if (ctx->hard_fail_count < 255)
    ctx->hard_fail_count++;
  TOF_LOG("[RECOVERY-HARD] %s recovery failed. Will retry after cooldown.\n",
          ctx->name);
  return false;
}
// =========================================================================
// SINGLE-TRANSACTION FRAME READ & TELEMETRY
// =========================================================================

// Reads frame directly when INT is LOW, avoiding the check_data_ready()
// double-read bug
static inline bool poll_and_read_sensor(VL53L8CX_Configuration *p_dev,
                                        ToF_Device_Context *ctx,
                                        VL53L8CX_ResultsData *p_results,
                                        unsigned long now) {
  bool int_low = (digitalRead(ctx->int_pin) == LOW);

  // Adaptive fallback poll: half a frame period
  unsigned long fallback_ms = max(8UL, 500UL / current_frequency_hz);
  if (!int_low && (now - ctx->last_poll_ms < fallback_ms)) {
    return false;
  }
  ctx->last_poll_ms = now;

  // INT LOW means a frame is ready: read directly (no header double-read).
  if (!int_low) {
    uint8_t ready = 0;
    if (vl53l8cx_check_data_ready(p_dev, &ready) != VL53L8CX_STATUS_OK ||
        !ready) {
      return false;
    }
  }

  if (vl53l8cx_get_ranging_data(p_dev, p_results) != VL53L8CX_STATUS_OK) {
    return false;
  }

  // Duplicate-frame guard; streamcount 255 is a valid value (rollover)
  if (!ctx->first_frame && p_dev->streamcount == ctx->last_streamcount) {
    return false;
  }

  ctx->first_frame = false;
  ctx->last_streamcount = p_dev->streamcount;
  ctx->soft_fail_streak = 0;
  return true;
}

// Dedicated 4 zones per sensor matching 2D LiDAR ray projection (from center forward to side):
// Sensor 1 (Left Side / Right-angled) : Z1,  Z5,  Z9,  Z13 -> 49.1 deg to 82.9 deg left
// Sensor 2 (Left Front)               : Z14, Z10, Z6,  Z2  ->  4.1 deg to 37.9 deg left
// Sensor 3 (Right Side / Left-angled) : Z2,  Z6,  Z10, Z14 -> 49.1 deg to 82.9 deg right
// Sensor 4 (Right Front)              : Z13, Z9,  Z5,  Z1  ->  4.1 deg to 37.9 deg right
static const uint8_t SENSOR_ZONES[4][4] = {
    {1, 5, 9, 13},   // ToF 1
    {14, 10, 6, 2},  // ToF 2
    {2, 6, 10, 14},  // ToF 3
    {13, 9, 5, 1}    // ToF 4
};

#pragma pack(push, 1)
// 20-byte lean frame for Raspberry Pi 4 (dist_mm only)
struct ToF_Binary_Packet_20 {
  uint8_t header[2];          // 0xAA, 0x55
  uint8_t sensor_id;          // 1, 2, 3, 4
  uint32_t seq;               // Frame sequence counter
  uint32_t ts_ms;             // ESP32 millis() timestamp
  uint16_t dist_mm[4];        // Resolved / chosen distance (mm, >= 30mm)
  uint8_t checksum;           // XOR checksum
};

// 52-byte multi-target debug frame for PC GUI
struct ToF_Binary_Packet_52 {
  uint8_t header[2];          // 0xAA, 0x55
  uint8_t sensor_id;          // 1, 2, 3, 4
  uint32_t seq;               // Frame sequence counter
  uint32_t ts_ms;             // ESP32 millis() timestamp
  uint16_t dist_mm[4];        // Resolved / chosen distance (mm, >= 30mm)
  uint16_t target0_mm[4];     // Main target distance (mm)
  uint16_t target1_mm[4];     // 2nd target distance (mm)
  uint8_t  target0_status[4]; // Main target status (ST table 4)
  uint8_t  target1_status[4]; // 2nd target status (ST table 4)
  uint16_t spad_count[4];     // Active SPAD count
  uint8_t checksum;           // XOR checksum
};
#pragma pack(pop)

static bool g_packet_mode_52 = false; // Default: 20-byte lean mode for Pi

static inline bool is_valid_target_status(uint8_t st) {
  // Only ST Table 4 status 5 (100% valid), 6 (wrap around 50%), and 9 (large pulse 50%) are valid.
  // Status 10 (Valid without previous history) is treated as an error/invalid with the rest of errors.
  return (st == 5 || st == 6 || st == 9);
}

void send_binary_telemetry(uint8_t sensor_id, VL53L8CX_ResultsData *p_res,
                           unsigned long f_count) {
  if (sensor_id < 1 || sensor_id > 4) return;

  uint16_t dist_mm[4];
  uint16_t target0_mm[4];
  uint16_t target1_mm[4];
  uint8_t  target0_status[4];
  uint8_t  target1_status[4];
  uint16_t spad_count[4];

  const uint8_t *zones = SENSOR_ZONES[sensor_id - 1];

  for (int i = 0; i < 4; i++) {
    uint8_t z = zones[i];
    uint32_t spad = p_res->nb_spads_enabled[z];

    // Check target 0 and target 1 (both are strongest targets returned by VL53L8CX)
    int src0 = z * VL53L8CX_NB_TARGET_PER_ZONE + 0;
    int src1 = z * VL53L8CX_NB_TARGET_PER_ZONE + 1;
    uint8_t st0 = p_res->target_status[src0];
    uint8_t st1 = p_res->target_status[src1];
    int16_t d0 = p_res->distance_mm[src0];
    int16_t d1 = p_res->distance_mm[src1];

    target0_mm[i] = (d0 > 0) ? (uint16_t)d0 : 0;
    target1_mm[i] = (d1 > 0) ? (uint16_t)d1 : 0;
    target0_status[i] = st0;
    target1_status[i] = st1;
    spad_count[i] = (spad > 65535) ? 65535 : (uint16_t)spad;

    bool valid0 = (is_valid_target_status(st0) && d0 > 0);
    bool valid1 = (is_valid_target_status(st1) && d1 > 0);

    bool near0 = (st0 != 0 && st0 != 255 && d0 >= 1 && d0 <= 30);
    bool near1 = (st1 != 0 && st1 != 255 && d1 >= 1 && d1 <= 30);

    if (valid0 && valid1) {
      // Both valid: choose the closer (shortest) distance
      dist_mm[i] = (uint16_t)(d0 <= d1 ? d0 : d1);
    } else if (valid0) {
      dist_mm[i] = (uint16_t)d0;
    } else if (valid1) {
      dist_mm[i] = (uint16_t)d1;
    } else {
      // 0 valid targets: fallback logic
      // Never trigger 3000mm open space if either target reports 0-30mm even if error!
      if (near0 && near1) {
        dist_mm[i] = (uint16_t)(d0 <= d1 ? d0 : d1);
      } else if (near0) {
        dist_mm[i] = (uint16_t)d0;
      } else if (near1) {
        dist_mm[i] = (uint16_t)d1;
      } else if (spad == 1024) {
        dist_mm[i] = 30; // Lens contact / saturation error
      } else {
        dist_mm[i] = 3000; // Open space fallback
      }
    }

    // Every reading below 30mm is reported as 30mm to Pi
    if (dist_mm[i] < 30) {
      dist_mm[i] = 30;
    }
  }

  uint32_t now_ms = (uint32_t)millis();

  if (g_packet_mode_52) {
    ToF_Binary_Packet_52 pkt;
    pkt.header[0] = 0xAA;
    pkt.header[1] = 0x55;
    pkt.sensor_id = sensor_id;
    pkt.seq = (uint32_t)f_count;
    pkt.ts_ms = now_ms;
    for (int i = 0; i < 4; i++) {
      pkt.dist_mm[i] = dist_mm[i];
      pkt.target0_mm[i] = target0_mm[i];
      pkt.target1_mm[i] = target1_mm[i];
      pkt.target0_status[i] = target0_status[i];
      pkt.target1_status[i] = target1_status[i];
      pkt.spad_count[i] = spad_count[i];
    }
    uint8_t *raw = (uint8_t *)&pkt;
    uint8_t chk = 0;
    for (size_t b = 0; b < sizeof(ToF_Binary_Packet_52) - 1; b++) {
      chk ^= raw[b];
    }
    pkt.checksum = chk;

    if (Serial.availableForWrite() >= (int)sizeof(ToF_Binary_Packet_52)) {
      Serial.write(raw, sizeof(ToF_Binary_Packet_52));
    }
  } else {
    ToF_Binary_Packet_20 pkt;
    pkt.header[0] = 0xAA;
    pkt.header[1] = 0x55;
    pkt.sensor_id = sensor_id;
    pkt.seq = (uint32_t)f_count;
    pkt.ts_ms = now_ms;
    for (int i = 0; i < 4; i++) {
      pkt.dist_mm[i] = dist_mm[i];
    }
    uint8_t *raw = (uint8_t *)&pkt;
    uint8_t chk = 0;
    for (size_t b = 0; b < sizeof(ToF_Binary_Packet_20) - 1; b++) {
      chk ^= raw[b];
    }
    pkt.checksum = chk;

    if (Serial.availableForWrite() >= (int)sizeof(ToF_Binary_Packet_20)) {
      Serial.write(raw, sizeof(ToF_Binary_Packet_20));
    }
  }
}

// =========================================================================
// MAIN SETUP
// =========================================================================

extern "C" void c_tof_log(const char *msg) {
  Serial.print(msg);
  Serial.flush();
}

static void init_pins(ToF_Device_Context *c) {
  esp_rom_gpio_pad_select_gpio(c->cs_pin);
  esp_rom_gpio_pad_select_gpio(c->lpn_pin);
  esp_rom_gpio_pad_select_gpio(c->int_pin);
  esp_rom_gpio_pad_select_gpio(c->miso_pin);
  gpio_reset_pin((gpio_num_t)c->cs_pin);
  gpio_reset_pin((gpio_num_t)c->lpn_pin);
  gpio_reset_pin((gpio_num_t)c->int_pin);
  gpio_reset_pin((gpio_num_t)c->miso_pin);
  pinMode(c->cs_pin, OUTPUT);
  digitalWrite(c->cs_pin, HIGH);
  pinMode(c->lpn_pin, OUTPUT);
  digitalWrite(c->lpn_pin, LOW);
  pinMode(c->int_pin, INPUT_PULLUP);
  pinMode(c->miso_pin, INPUT_PULLUP);
  gpio_set_drive_capability((gpio_num_t)c->cs_pin, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)c->lpn_pin, GPIO_DRIVE_CAP_3);
}

void setup() {
  Serial.setTxBufferSize(16384);
  Serial.begin(921600);
  delay(200);

  // Keep GPIO 48 LOW so onboard RGB LEDs remain completely off
  pinMode(48, OUTPUT);
  digitalWrite(48, LOW);

  // Ensure strapping pin GPIO 3 is pulled up so hardware JTAG is decoupled from pads 39-42
  pinMode(3, INPUT_PULLUP);

  // Force-release all JTAG pads (GPIO 39-42) from debug peripheral
  const gpio_num_t jtag_pins[] = {GPIO_NUM_39, GPIO_NUM_40, GPIO_NUM_41, GPIO_NUM_42};
  for (gpio_num_t p : jtag_pins) {
    gpio_reset_pin(p);
    esp_rom_gpio_pad_select_gpio(p);
  }

  Serial.println(F("\n=== 4x VL53L8CX INITIALIZATION & FIRMWARE LOADER ==="));

  esp_rom_gpio_pad_select_gpio(TOF_SHARED_SCK_PIN);
  esp_rom_gpio_pad_select_gpio(TOF_SHARED_MOSI_PIN);
  gpio_reset_pin((gpio_num_t)TOF_SHARED_SCK_PIN);
  gpio_reset_pin((gpio_num_t)TOF_SHARED_MOSI_PIN);
  pinMode(TOF_SHARED_SCK_PIN, OUTPUT);
  pinMode(TOF_SHARED_MOSI_PIN, OUTPUT);
  gpio_set_drive_capability((gpio_num_t)TOF_SHARED_SCK_PIN, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)TOF_SHARED_MOSI_PIN, GPIO_DRIVE_CAP_3);

  // Initialize ALL 4 sensors' pins (CS=HIGH, LPn=LOW) so disabled or not-yet-booted
  // sensors stay in hardware reset and ignore shared SCK/MOSI traffic.
  for (int i = 0; i < NUM_SENSORS; i++)
    init_pins(&ctxs[i]);

  delay(50);
  HardwareSPI.begin(TOF_SHARED_SCK_PIN, TOF1_MISO_PIN, TOF_SHARED_MOSI_PIN, -1);
  current_miso_pin = TOF1_MISO_PIN;
  gpio_pullup_en((gpio_num_t)TOF1_MISO_PIN);
  gpio_pulldown_dis((gpio_num_t)TOF1_MISO_PIN);

  // PHASE 1: Wake and initialize each sensor ONE BY ONE so uninitialized
  // sensors remain in LPn=LOW reset (deaf to I2C/SPI traffic on SCK/MOSI)
  // until previous sensors are locked into SPI mode and initialized.
  bool init_ok[NUM_SENSORS] = {false};
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!ctxs[i].enabled)
      continue;
    Serial.printf("\n[POWER] Waking %s (LPn pin %d -> HIGH)...\n",
                  ctxs[i].name, ctxs[i].lpn_pin);
    digitalWrite(ctxs[i].cs_pin, HIGH);
    digitalWrite(ctxs[i].lpn_pin, HIGH);
    delay(50);
    init_ok[i] = init_sensor_with_retries(&devs[i], &ctxs[i], 3);
  }

  // PHASE 2: start ranging with 4ms phase staggering. A sensor is only
  // marked active if start_ranging succeeds; others are retried from loop().
  Serial.println(F("\n[RANGING] Starting ranging sessions..."));
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!ctxs[i].enabled || !init_ok[i])
      continue;
    uint8_t st = vl53l8cx_start_ranging(&devs[i]);
    Serial.printf("[RANGING] %s start_ranging status: %u\n", ctxs[i].name, st);
    ctxs[i].active = (st == VL53L8CX_STATUS_OK);
    ctxs[i].first_frame = true;
    ctxs[i].last_frame_ms = millis();
    delay(4);
  }

  Serial.println(F("\n================================================="));
  Serial.printf(" SUMMARY: ToF1:%s | ToF2:%s | ToF3:%s | ToF4:%s\n",
                ctxs[0].active ? "OK" : "FAIL", ctxs[1].active ? "OK" : "FAIL",
                ctxs[2].active ? "OK" : "FAIL", ctxs[3].active ? "OK" : "FAIL");
  Serial.println(F("=================================================\n"));
  Serial.flush();

  unsigned long t = millis();
  for (int i = 0; i < NUM_SENSORS; i++) {
    ctxs[i].last_frame_ms = t;
    ctxs[i].last_poll_ms = t;
    ctxs[i].last_recovery_ms = t;
  }
  g_streaming = true; // From here on, Serial carries binary packets only
}

// =========================================================================
// MAIN LOOP
// =========================================================================

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'D' || c == 'd' || c == 'M' || c == 'm') g_packet_mode_52 = true;  // Debug mode: 52B multi-target
    else if (c == 'L' || c == 'l' || c == 'N' || c == 'n') g_packet_mode_52 = false; // Lean mode: 20B (default for Pi)
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    ToF_Device_Context *ctx = &ctxs[i];
    if (!ctx->enabled)
      continue;

    // Fresh timestamp per sensor: earlier sensors may have blocked
    unsigned long now = millis();

    if (ctx->active) {
      if (poll_and_read_sensor(&devs[i], ctx, &results[i], now)) {
        ctx->frame_count++;
        ctx->last_frame_ms = now;
        send_binary_telemetry(ctx->sensor_id, &results[i], ctx->frame_count);
        continue;
      }
    }

    // Stall detection: fast soft recovery only (hard reset & resync disabled)
    if (ctx->active && (now - ctx->last_frame_ms > TOF_STALL_TIMEOUT_MS)) {
      if (now - ctx->last_recovery_ms > TOF_SOFT_RECOVERY_COOLDOWN_MS) {
        soft_recover_sensor(&devs[i], ctx);
        ctx->last_recovery_ms = millis();
      }
    }
  }
}

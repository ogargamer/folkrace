#ifndef MCU_TELEMETRY_PROTOCOL_H
#define MCU_TELEMETRY_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// FOLKRACE LIGHTWEIGHT BINARY UART TELEMETRY PROTOCOL
// =============================================================================
// Designed for ultra-low latency & zero-copy direct memory transmission.
// The MCU doesn't do ANY string formatting or conversion:
//   Serial.write((const uint8_t *)&packet, sizeof(packet));
//
// Transmission overhead:
//   - Chassis Dynamics Packet: ~54 bytes @ 100 Hz = 5.4 KB/s (< 6% of 921.6k Baud)
//   - ToF Distance Packet:    ~76 bytes @ 60 Hz  = 4.5 KB/s (< 5% of 921.6k Baud)
//   - Downlink Motion Command: ~12 bytes @ 60 Hz = 0.7 KB/s (< 1% of 921.6k Baud)
// =============================================================================

#define MCU_UART_SYNC_1          0xAA
#define MCU_UART_SYNC_2          0x55

#define PI_UART_SYNC_1           0x55
#define PI_UART_SYNC_2           0xAA

#define MSG_ID_CHASSIS_TELEM     0x01
#define MSG_ID_TOF_TELEM         0x02
#define MSG_ID_MOTION_CMD        0x10

#define FLAG_OVERRIDE_ACTIVE     (1 << 0)
#define FLAG_BENCH_FAKE_DATA     (1 << 1)
#define FLAG_IMU_ONLINE          (1 << 2)
#define FLAG_STM32_FRONT_ONLINE  (1 << 3)
#define FLAG_STM32_REAR_ONLINE   (1 << 4)

#pragma pack(push, 1)

// =============================================================================
// 1. CHASSIS & DYNAMICS PACKET (ESP32 Main -> Pi 4)
// Total Size: 54 Bytes
// =============================================================================
typedef struct {
    uint8_t  sync[2];             // 0xAA, 0x55
    uint8_t  msg_type;            // 0x01 (MSG_ID_CHASSIS_TELEM)
    uint8_t  payload_len;         // 49 bytes payload
    uint32_t seq;                 // Monotonically increasing sequence number
    uint32_t timestamp_us;        // Microseconds timestamp

    // IMU Attitude & Accelerometer
    int16_t  yaw_deg_x10;         // -180.0 to +180.0 deg (scaled x10, e.g. 125 = 12.5 deg)
    int16_t  pitch_deg_x10;       // -90.0 to +90.0 deg (scaled x10)
    int16_t  roll_deg_x10;        // -180.0 to +180.0 deg (scaled x10)
    int16_t  gyro_z_mrad_s;       // Yaw rate in milliradians/sec (rad/s * 1000)
    int16_t  accel_fwd_cm_s2;     // Forward linear acceleration (cm/s^2, 100 = 1.0 m/s^2)
    int16_t  accel_lat_cm_s2;     // Lateral linear acceleration (cm/s^2)

    // Kinematics & Actuator Setpoints
    int16_t  steer_cmd_deg_x10;   // Commanded center steering angle (deg * 10)
    int16_t  steer_act_deg_x10;   // Actual steering angle feedback (deg * 10)
    uint16_t servo_pulse_us;      // Active PWM pulse width in microseconds (1000..2000)
    int16_t  v_cmd_mm_s;          // Commanded forward speed in mm/s (e.g. 3000 = 3.0 m/s)
    int16_t  v_est_mm_s;          // Fused vehicle velocity in mm/s

    // 4-Wheel Motor Setpoints & Actual Measured RPMs
    int16_t  fl_rpm_cmd;          // Front-Left target RPM
    int16_t  fr_rpm_cmd;          // Front-Right target RPM
    int16_t  rl_rpm_cmd;          // Rear-Left target RPM
    int16_t  rr_rpm_cmd;          // Rear-Right target RPM
    int16_t  fl_rpm_act;          // Front-Left actual measured RPM (from STM32)
    int16_t  fr_rpm_act;          // Front-Right actual measured RPM (from STM32)
    int16_t  rl_rpm_act;          // Rear-Left actual measured RPM (from STM32)
    int16_t  rr_rpm_act;          // Rear-Right actual measured RPM (from STM32)

    // Slip Dynamics & Subsystem Status
    uint8_t  slip_state;          // 0=NORMAL, 1=UNDERSTEER, 2=OVERSTEER, 3=SPIN
    int8_t   slip_ratio_pct;      // Slip ratio in percent (-100% to +100%)
    uint8_t  flags;               // Hardware status bitmask (FLAG_*)
    uint8_t  checksum;            // XOR over all preceding bytes [0 .. 52]
} ChassisTelemetryPacket;


// =============================================================================
// 2. TOF MULTI-ZONE PERCEPTION PACKET (ESP32 ToF Hub -> Pi 4)
// Total Size: 74 Bytes
// =============================================================================
typedef struct {
    uint8_t  sync[2];             // 0xAA, 0x55
    uint8_t  msg_type;            // 0x02 (MSG_ID_TOF_TELEM)
    uint8_t  sensor_id;           // Sensor channel ID (1..5)
    uint32_t seq;                 // Frame counter
    uint32_t timestamp_us;        // Timestamp in microseconds

    // 16 Zones Filtered Readings
    uint16_t dist_mm[16];         // Target distance per zone in mm (0..4000)
    uint8_t  status[16];          // Target detection status (5,6,9 = valid)
    uint8_t  spad[16];            // SPAD count / signal quality per zone
    int8_t   silicon_temp;        // Sensor silicon temperature in deg C
    uint8_t  checksum;            // XOR over bytes [0 .. 72]
} ToFTelemetryPacket;


// =============================================================================
// 3. MOTION COMMAND PACKET (Pi 4 -> ESP32 Main)
// Total Size: 13 Bytes
// =============================================================================
typedef struct {
    uint8_t  sync[2];             // 0x55, 0xAA
    uint8_t  msg_type;            // 0x10 (MSG_ID_MOTION_CMD)
    uint32_t seq;                 // Command sequence counter
    int16_t  target_v_mm_s;       // Target vehicle speed in mm/s (-5000 .. +5000)
    int16_t  target_steer_mrad;   // Target steering angle in milliradians (-650 .. +650)
    uint8_t  run_mode;            // 0=IDLE, 1=DRIVE, 2=EMERGENCY_BRAKE, 3=RECOVERY
    uint8_t  checksum;            // XOR over bytes [0 .. 11]
} MotionCommandPacket;

#pragma pack(pop)

// Fast XOR Checksum Calculator (takes < 20 CPU cycles on Cortex-M4 / ESP32)
static inline uint8_t mcu_calc_checksum(const uint8_t *data, size_t len) {
    uint8_t chk = 0;
    for (size_t i = 0; i < len; i++) {
        chk ^= data[i];
    }
    return chk;
}

#ifdef __cplusplus
}
#endif

#endif // MCU_TELEMETRY_PROTOCOL_H


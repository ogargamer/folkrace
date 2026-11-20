#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <stm32g4xx_ll_cordic.h>
#include <stm32g4xx_ll_bus.h>
#include "config_pins.h"
#include "mt6816_spi.h"

// ============================================================================
// FOLKRACE — 20.0 kHz HARD REAL-TIME DUAL-MOTOR SIMPLEFOC ENGINE (STM32G474)
// ============================================================================
// Hardware: STM32G474CBT6 (170 MHz Cortex-M4F, HW FPU)
// Timers: TIM2 (M1 PWM 25kHz), TIM3 (M2 PWM 25kHz), TIM6 (20 kHz Control ISR)
// Sensors: Dual MT6816 14-Bit Encoders on Independent SPI1 & SPI2 @ 5.31 MHz
// Telemetry: Exact 26-Byte Binary Telemetry Frame @ 25 Hz + ASCII Console
// ============================================================================

// ----------------------------------------------------------------------------
// 26-Byte Binary Telemetry Frame (Exact match with Python GUI struct)
// ----------------------------------------------------------------------------
struct __attribute__((packed)) TelemetryPacket {
    uint8_t  header[2];      // 0xAA, 0x55 (Sync word)          [Bytes 0..1]
    uint16_t m1_raw_angle;   // 0 .. 16383 (Raw 14-bit count)    [Bytes 2..3]
    int16_t  m1_actual_rpm;  // Scaled x10 (e.g. 4000 = 400.0)  [Bytes 4..5]
    int16_t  m1_target_rpm;  // Scaled x10                       [Bytes 6..7]
    int16_t  m1_uq_mv;       // Phase voltage Uq (mV)            [Bytes 8..9]
    uint8_t  m1_status;      // Bitfield: En, LUT, Mag, Par, St  [Byte 10]
    uint16_t m2_raw_angle;   // 0 .. 16383 (Raw 14-bit count)    [Bytes 11..12]
    int16_t  m2_actual_rpm;  // Scaled x10                       [Bytes 13..14]
    int16_t  m2_target_rpm;  // Scaled x10                       [Bytes 15..16]
    int16_t  m2_uq_mv;       // Phase voltage Uq (mV)            [Bytes 17..18]
    uint8_t  m2_status;      // Bitfield: En, LUT, Mag, Par, St  [Byte 19]
    uint16_t foc_khz_x10;    // 200 = 20.0 kHz                   [Bytes 20..21]
    uint8_t  isr_lat_us;     // Peak ISR latency in µs           [Byte 22]
    uint8_t  spikes_count;   // Jitter spikes counter            [Byte 23]
    uint8_t  calib_progress; // 0 .. 100% calibration progress   [Byte 24]
    uint8_t  checksum;       // XOR sum over bytes 2..24         [Byte 25]
};
static_assert(sizeof(TelemetryPacket) == 26, "TelemetryPacket must be exactly 26 bytes");

// ----------------------------------------------------------------------------
// ESP32 High-Speed 400 Hz UART Protocol Definitions
// ----------------------------------------------------------------------------
#define FOC_TRACE_SAMPLE_HZ        500
#define FOC_TRACE_CAPACITY         5120

struct __attribute__((packed)) FocTraceSample {
    uint16_t m1_raw_angle;
    uint16_t m2_raw_angle;
    int16_t m1_target_rpm;
    int16_t m2_target_rpm;
    int16_t m1_actual_rpm;
    int16_t m2_actual_rpm;
    int16_t m1_uq_mv;
    int16_t m2_uq_mv;
};
static_assert(sizeof(FocTraceSample) == 16, "FocTraceSample must be exactly 16 bytes");

struct __attribute__((packed)) FocTraceHeader {
    uint8_t sync[2];
    uint16_t version;
    uint16_t sample_hz;
    uint16_t sample_count;
    uint32_t overwritten;
    uint32_t reserved;
};
static_assert(sizeof(FocTraceHeader) == 16, "FocTraceHeader must be exactly 16 bytes");

static volatile FocTraceSample foc_trace[FOC_TRACE_CAPACITY];
static volatile uint16_t foc_trace_write_index = 0;
static volatile uint16_t foc_trace_count = 0;
static volatile uint32_t foc_trace_overwritten = 0;
static volatile bool foc_trace_active = false;
static constexpr uint32_t FOC_TRACE_DECIMATION = FOC_COMMUTATION_HZ / FOC_TRACE_SAMPLE_HZ;

static uint32_t last_esp_packet_ms = 0;
static bool esp_link_active = false;
static volatile uint32_t g_esp_rx_seq_isr = 0;
static volatile bool g_esp_link_active_isr = false;
volatile bool g_estop_active = false;
static constexpr size_t ESP_COMMAND_BYTE_BUDGET = 128;
static uint32_t esp_rx_bytes = 0;
static uint32_t esp_pkts_ok = 0;
static uint32_t esp_chk_err = 0;
static uint32_t esp_wdog_trips = 0;
static uint32_t esp_max_gap_ms = 0;
static uint32_t esp_usart1_ore = 0;
static uint32_t esp_usart1_fe = 0;
static uint32_t esp_usart1_ne = 0;
static uint32_t esp_drive_zero = 0;

#define STM32_CMD_SYNC_1           0xAA
#define STM32_CMD_SYNC_2           0x55
#define STM32_FEEDBACK_SYNC_1      0xAA
#define STM32_FEEDBACK_SYNC_2      0x55

enum STM32CommandMode : uint8_t {
    STM32_CMD_DRIVE      = 0x00, // Regular 400Hz velocity+torque command
    STM32_CMD_ESTOP      = 0x01, // Immediate E-stop: cut PWM, zero voltage, disable drivers
    STM32_CMD_ENABLE     = 0x02, // Enable motor drivers
    STM32_CMD_DISABLE    = 0x03, // Disable motor drivers (Hi-Z, non-latched stop)
    STM32_CMD_CALIB_M1   = 0x10, // Start Motor 1 Calibration (param1=align_voltage)
    STM32_CMD_CALIB_M2   = 0x11, // Start Motor 2 Calibration (param1=align_voltage)
    STM32_CMD_SET_PID    = 0x20, // Set velocity PID (param1=Kp, param2=Ki)
    STM32_CMD_SAVE_FLASH = 0x30, // Save active calibration & gains to STM32 Flash
    STM32_CMD_ENTER_DFU  = 0xDF  // Reboot into ROM DFU bootloader
};

struct __attribute__((packed)) STM32MotorCommandPacket {
    uint8_t header[2];       // 0xAA, 0x55 (STM32_CMD_SYNC_1, STM32_CMD_SYNC_2)
    uint8_t cmd_mode;        // STM32CommandMode
    float   param1;          // m1 target RPM / Kp / align_voltage
    float   param2;          // m2 target RPM / Ki
    int16_t m1_vtrq_min_mv;  // M1 min torque voltage (mV)
    int16_t m1_vtrq_max_mv;  // M1 max torque voltage (mV)
    int16_t m1_vtrq_ff_mv;   // M1 feedforward torque voltage (mV)
    int16_t m2_vtrq_min_mv;  // M2 min torque voltage (mV)
    int16_t m2_vtrq_max_mv;  // M2 max torque voltage (mV)
    int16_t m2_vtrq_ff_mv;   // M2 feedforward torque voltage (mV)
    uint8_t checksum;        // XOR of bytes 2..22
};
static_assert(sizeof(STM32MotorCommandPacket) == 24, "STM32MotorCommandPacket must be 24 bytes");

#define STM32_STATUS_M1_ENABLED     (1 << 0)
#define STM32_STATUS_M2_ENABLED     (1 << 1)
#define STM32_STATUS_CALIB_ACTIVE   (1 << 2)
#define STM32_STATUS_STALL_WARNING  (1 << 3)
#define STM32_STATUS_LATCHED_FAULT  (1 << 4)
#define STM32_STATUS_ENC1_OK        (1 << 5)
#define STM32_STATUS_ENC2_OK        (1 << 6)

struct __attribute__((packed)) STM32FeedbackPacket {
    uint8_t header[2];      // 0xAA, 0x55
    uint8_t status_flags;   // Bitfield (enabled, calib, stall, fault, enc_ok)
    float   m1_actual_rpm;  // Actual shaft RPM
    float   m2_actual_rpm;  // Actual shaft RPM
    uint8_t calib_progress; // 0..100%
    uint8_t checksum;       // XOR over bytes 2..11 (status_flags, m1, m2, calib_progress)
};
static_assert(sizeof(STM32FeedbackPacket) == 13, "STM32FeedbackPacket must be 13 bytes");

// Hardware Serial instance connected to ESP32 on USART1 (PA10 RX, PA9 TX)
HardwareSerial SerialESP(PIN_ESP_UART_RX, PIN_ESP_UART_TX);

// ----------------------------------------------------------------------------
// Global Peripherals & Motor Instances
// ----------------------------------------------------------------------------
// Dedicated hardware SPI2 instance: constructor order is (MOSI, MISO, SCK)
SPIClass SPI_2(PIN_SPI2_MOSI, PIN_SPI2_MISO, PIN_SPI2_SCK);

MT6816Sensor encoder1(PIN_M1_CS, &SPI, PIN_SPI1_SCK, PIN_SPI1_MISO, PIN_SPI1_MOSI);
MT6816Sensor encoder2(PIN_M2_CS, &SPI_2, PIN_SPI2_SCK, PIN_SPI2_MISO, PIN_SPI2_MOSI);

// Fast direct-register PWM driver bypassing float divisions and switch-cases
class FastBLDCDriver3PWM : public BLDCDriver3PWM {
public:
    TIM_TypeDef *timer_inst;
    float pwm_scale;

    FastBLDCDriver3PWM(int phA, int phB, int phC, int en, TIM_TypeDef *tim)
        : BLDCDriver3PWM(phA, phB, phC, en), timer_inst(tim), pwm_scale(0.0f) {}

    int init() override {
        int res = BLDCDriver3PWM::init();
        if (res && timer_inst) {
            // Precompute scale = (ARR + 1) / voltage_power_supply
            pwm_scale = (float)(timer_inst->ARR + 1) / voltage_power_supply;

            // CRITICAL FIX: SimpleFOC's _configure3PWM() does NOT enable Output Compare Preload
            // (OC1PE / OC2PE / OC3PE) on 3-PWM timers! Without shadow register preload, writing
            // CCR1/2/3 inside the 20 kHz ISR asynchronously against the 25 kHz PWM counter
            // causes mid-cycle comparator crossings and narrow glitch pulses (audible static/radio crackle).
            timer_inst->CCMR1 |= (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
            timer_inst->CCMR2 |= TIM_CCMR2_OC3PE;
            timer_inst->CR1   |= TIM_CR1_ARPE;
            timer_inst->EGR    = TIM_EGR_UG;
        }
        return res;
    }

    void setPwm(float Ua, float Ub, float Uc) override {
        Ua = _constrain(Ua, 0.0f, voltage_limit);
        Ub = _constrain(Ub, 0.0f, voltage_limit);
        Uc = _constrain(Uc, 0.0f, voltage_limit);

        uint32_t duty_a = (uint32_t)(Ua * pwm_scale);
        uint32_t duty_b = (uint32_t)(Ub * pwm_scale);
        uint32_t duty_c = (uint32_t)(Uc * pwm_scale);

        if (timer_inst == TIM2) {
            TIM2->CR1 |= TIM_CR1_UDIS;
            TIM2->CCR1 = duty_a; // PA5 (TIM2 CH1)
            TIM2->CCR2 = duty_b; // PA1 (TIM2 CH2)
            TIM2->CCR3 = duty_c; // PA2 (TIM2 CH3)
            TIM2->CR1 &= ~TIM_CR1_UDIS;
        } else if (timer_inst == TIM3) {
            TIM3->CR1 |= TIM_CR1_UDIS;
            TIM3->CCR3 = duty_a; // PB0 (TIM3 CH3)
            TIM3->CCR2 = duty_b; // PA4 (TIM3 CH2)
            TIM3->CCR1 = duty_c; // PA6 (TIM3 CH1)
            TIM3->CR1 &= ~TIM_CR1_UDIS;
        }
    }
};

FastBLDCDriver3PWM driver1(M1_PWM_A, M1_PWM_B, M1_PWM_C, M1_EN, TIM2);
FastBLDCDriver3PWM driver2(M2_PWM_A, M2_PWM_B, M2_PWM_C, M2_EN, TIM3);

BLDCMotor motor1 = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCMotor motor2 = BLDCMotor(MOTOR_POLE_PAIRS);

HardwareTimer *foc_timer = nullptr;

// ----------------------------------------------------------------------------
// System & Real-Time Operational State
// ----------------------------------------------------------------------------
volatile bool m1_enabled = false;
volatile bool m2_enabled = false;
volatile bool m1_calibrated = false;
volatile bool m2_calibrated = false;

volatile float m1_target_rpm = 0.0f;
volatile float m2_target_rpm = 0.0f;

// ----------------------------------------------------------------------------
// 5 kHz Decoupled Back-EMF + Slew-Limited Torque-Envelope Control State
// ----------------------------------------------------------------------------
struct MotorTorqueCtrl {
    // Mailbox updated atomically from main loop
    float cmd_omega_rad_s = 0.0f;
    float cmd_vtrq_min    = -MOTOR_VOLTAGE_LIMIT;
    float cmd_vtrq_max    =  MOTOR_VOLTAGE_LIMIT;
    float cmd_vtrq_ff     = 0.0f;
    volatile uint32_t cmd_seq = 0;

    // ISR-owned state (5 kHz)
    uint32_t isr_last_seq = 0;
    float omega_ref_interp = 0.0f;
    float omega_step_per_tick = 0.0f;
    uint8_t interp_ticks_rem = 0;
    float integral_v = 0.0f;
    float err_prev = 0.0f;
    float v_trq_slewed = 0.0f;
    float last_v_bemf = 0.0f;

    // Config & guard state
    float i_clamp_v = DEFAULT_PID_I_CLAMP_V;
    float stall_vtrq_cap = MOTOR_VOLTAGE_LIMIT;
    float ke_q = BEMF_KE_SAFETY_SCALE / (MOTOR_KV_RATING * _SQRT3 * _RPM_TO_RADS);

    void updateKe(float kv_rating) {
        float kv = (kv_rating > 10.0f) ? kv_rating : MOTOR_KV_RATING;
        ke_q = BEMF_KE_SAFETY_SCALE / (kv * _SQRT3 * _RPM_TO_RADS);
    }

    void resetIsrState(float current_omega = 0.0f) {
        isr_last_seq = cmd_seq;
        omega_ref_interp = current_omega;
        omega_step_per_tick = 0.0f;
        interp_ticks_rem = 0;
        integral_v = 0.0f;
        err_prev = 0.0f;
        v_trq_slewed = 0.0f;
        last_v_bemf = ke_q * current_omega;
    }
};

static MotorTorqueCtrl m1_ctrl;
static MotorTorqueCtrl m2_ctrl;

static inline void setMotorCommand(MotorTorqueCtrl &ctrl,
                                   volatile float &target_rpm_var,
                                   float target_rpm,
                                   float v_min,
                                   float v_max,
                                   float v_ff) {
    if (isnan(target_rpm) || isinf(target_rpm)) target_rpm = 0.0f;
    if (isnan(v_min) || isinf(v_min)) v_min = -MOTOR_VOLTAGE_LIMIT;
    if (isnan(v_max) || isinf(v_max)) v_max =  MOTOR_VOLTAGE_LIMIT;
    if (isnan(v_ff)  || isinf(v_ff))  v_ff  = 0.0f;
    if (v_min > v_max) {
        float mid = 0.5f * (v_min + v_max);
        v_min = mid;
        v_max = mid;
    }
    float omega = target_rpm * _RPM_TO_RADS;
    __disable_irq();
    target_rpm_var = target_rpm;
    ctrl.cmd_omega_rad_s = omega;
    ctrl.cmd_vtrq_min = v_min;
    ctrl.cmd_vtrq_max = v_max;
    ctrl.cmd_vtrq_ff = v_ff;
    ctrl.cmd_seq++;
    __enable_irq();
}

static inline void resetMotorTorqueCtrl(MotorTorqueCtrl &ctrl,
                                        volatile float &target_rpm_var,
                                        float current_omega = 0.0f,
                                        bool reset_bounds = true) {
    __disable_irq();
    if (reset_bounds) {
        target_rpm_var = 0.0f;
        ctrl.cmd_omega_rad_s = 0.0f;
        ctrl.cmd_vtrq_min = -MOTOR_VOLTAGE_LIMIT;
        ctrl.cmd_vtrq_max =  MOTOR_VOLTAGE_LIMIT;
        ctrl.cmd_vtrq_ff  = 0.0f;
        ctrl.cmd_seq++;
    }
    ctrl.stall_vtrq_cap = MOTOR_VOLTAGE_LIMIT;
    ctrl.resetIsrState(current_omega);
    __enable_irq();
}

volatile float m1_base_zero_angle = 0.0f;
volatile float m2_base_zero_angle = 0.0f;
volatile float m1_lag_comp_us = 35.0f;
volatile float m2_lag_comp_us = 35.0f;
static float m1_lag_filt_vel = 0.0f;
static float m2_lag_filt_vel = 0.0f;

volatile bool m1_calib_active = false;
volatile bool m2_calib_active = false;
volatile uint8_t live_calib_progress = 0; // 0..100%

// DWT Latency & Real-Time Profiling Variables
volatile uint32_t foc_tick_count = 0;
volatile uint32_t isr_peak_cycles = 0;
volatile uint32_t isr_last_cycles = 0;
volatile uint32_t isr_alltime_peak_cycles = 0;
volatile uint32_t isr_last_c0 = 0;
volatile uint32_t prof_period_last = 8500;
volatile uint32_t prof_period_min = 8500;
volatile uint32_t prof_period_max = 8500;
volatile uint32_t prof_period_jitter_max = 0;
volatile uint8_t  isr_jitter_spikes = 0;

volatile uint32_t prof_spi_max = 0;
volatile uint32_t prof_update_max = 0;
volatile uint32_t prof_m1_foc_max = 0;
volatile uint32_t prof_m2_foc_max = 0;
volatile uint32_t prof_move_max = 0;
volatile uint32_t prof_total_max = 0;

volatile uint32_t prof_spi_last = 0;
volatile uint32_t prof_update_last = 0;
volatile uint32_t prof_m1_foc_last = 0;
volatile uint32_t prof_m2_foc_last = 0;
volatile uint32_t prof_move_last = 0;
volatile uint32_t prof_total_last = 0;

// Stall & Safety Watchdog Tracking
// ----------------------------------------------------------------------------
// Industry-Grade Stall & Thermal Protection FSM
// ----------------------------------------------------------------------------
enum class StallState : uint8_t {
    NORMAL          = 0,
    WARNING         = 1,
    FOLDBACK        = 2,
    UNWEDGE_DITHER  = 3,
    LATCHED_FAULT   = 4
};

struct IndustrialStallGuard {
    StallState state = StallState::NORMAL;
    float energy_accumulator = 0.0f;
    uint32_t state_timer = 0;
    uint8_t recovery_attempts = 0;
    uint8_t enc_fault_ticks = 0;
    bool latched_fault = false;

    void reset() {
        state = StallState::NORMAL;
        energy_accumulator = 0.0f;
        state_timer = 0;
        recovery_attempts = 0;
        enc_fault_ticks = 0;
        latched_fault = false;
    }
};

IndustrialStallGuard stall_guard1;
IndustrialStallGuard stall_guard2;
bool stall_watchdog_enabled = true;
uint32_t m1_stall_timer = 0;
uint32_t m2_stall_timer = 0;
bool m1_is_stalled = false;
bool m2_is_stalled = false;

// Telemetry & Diagnostic Timers
uint32_t last_telemetry_time = 0;
uint32_t last_led_time = 0;
bool led_state = false;
bool show_telemetry = true;

// Serial Line Buffer for Command Parsing
char rx_line[256];
uint16_t rx_idx = 0;

// ----------------------------------------------------------------------------
// Ultra-Lightweight Zero-Allocation Number Parsers (Eliminates libc _strtod_l & sscanf)
// ----------------------------------------------------------------------------
static inline const char *skip_ws(const char *s) {
    while (*s && (*s == ' ' || *s == '\t')) s++;
    return s;
}

static inline int fast_atoi(const char *s, const char **endptr = nullptr) {
    s = skip_ws(s);
    int sign = 1;
    if (*s == '-') { sign = -1; s++; }
    else if (*s == '+') { s++; }
    int val = 0;
    while (*s >= '0' && *s <= '9') {
        val = val * 10 + (*s - '0');
        s++;
    }
    if (endptr) *endptr = s;
    return sign * val;
}

static inline float fast_atof(const char *s, const char **endptr = nullptr) {
    s = skip_ws(s);
    float sign = 1.0f;
    if (*s == '-') { sign = -1.0f; s++; }
    else if (*s == '+') { s++; }
    float val = 0.0f;
    while (*s >= '0' && *s <= '9') {
        val = val * 10.0f + (float)(*s - '0');
        s++;
    }
    if (*s == '.') {
        s++;
        float frac = 0.1f;
        while (*s >= '0' && *s <= '9') {
            val += (float)(*s - '0') * frac;
            frac *= 0.1f;
            s++;
        }
    }
    if (endptr) *endptr = s;
    return sign * val;
}

// ----------------------------------------------------------------------------
// High-Speed Math & Hardware Acceleration Overrides (STM32G474)
// ----------------------------------------------------------------------------
void initHardwareCORDIC() {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);
    LL_CORDIC_Config(CORDIC,
                     LL_CORDIC_FUNCTION_COSINE,
                     LL_CORDIC_PRECISION_6CYCLES,
                     LL_CORDIC_SCALE_0,
                     LL_CORDIC_NBWRITE_1,
                     LL_CORDIC_NBREAD_2,
                     LL_CORDIC_INSIZE_32BITS,
                     LL_CORDIC_OUTSIZE_32BITS);
}

// Fast single-precision angle normalization (replaces slow double-precision fmod)
float _normalizeAngle(float angle) {
    float a = angle - _2PI * (int)(angle * (1.0f / _2PI));
    return (a >= 0.0f) ? a : (a + _2PI);
}

static inline float normalizeAngleRad(float angle) {
    return _normalizeAngle(angle);
}

static inline float wrapAngleRad(float angle) {
    while (angle > (float)_PI) angle -= (float)_2PI;
    while (angle < -(float)_PI) angle += (float)_2PI;
    return angle;
}

// Hardware CORDIC trigonometric accelerator (simultaneous sin & cos in hardware)
void _sincos(float a, float *s, float *c) {
    float angle = a;
    while (angle > (float)_PI) angle -= (float)_2PI;
    while (angle < -(float)_PI) angle += (float)_2PI;

    // Convert angle to Q1.31: angle * (2^31 / PI) = angle * 683565275.5764f
    int32_t arg_q31 = (int32_t)(angle * 683565275.5764f);

    CORDIC->WDATA = arg_q31;
    while (!LL_CORDIC_IsActiveFlag_RRDY(CORDIC));
    int32_t c_raw = (int32_t)CORDIC->RDATA;
    int32_t s_raw = (int32_t)CORDIC->RDATA;

    constexpr float q31_to_f = 1.0f / 2147483648.0f;
    *c = (float)c_raw * q31_to_f;
    *s = (float)s_raw * q31_to_f;
}

float _sin(float a) {
    float s, c;
    _sincos(a, &s, &c);
    return s;
}

float _cos(float a) {
    float s, c;
    _sincos(a, &s, &c);
    return c;
}

// Fast DWT-based micros counter via GNU ld --wrap (replaces SysTick/HAL_GetTick division polling)
extern "C" unsigned long __wrap__micros() {
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}

extern "C" float __wrap_sinf(float a) {
    return _sin(a);
}

extern "C" float __wrap_cosf(float a) {
    return _cos(a);
}

extern "C" void __wrap_sincosf(float a, float *s, float *c) {
    _sincos(a, s, c);
}

// ----------------------------------------------------------------------------
// STM32G474 170 MHz Range 1 Boost Clock Configuration
// Overrides default weak 150 MHz configuration to unleash full 170 MHz rated speed
// ----------------------------------------------------------------------------
extern "C" void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4; // 16 / 4 = 4 MHz
    RCC_OscInitStruct.PLL.PLLN = 85;             // 4 * 85 = 340 MHz (VCO)
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // 340 / 2 = 170 MHz!
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

// ----------------------------------------------------------------------------
// Immediate Bare-Metal Safety Kill, Cortex-M Fault Handlers & IWDG
// ----------------------------------------------------------------------------
static inline void hardwareKillMotorsImmediate() {
    GPIOA->BSRR = (GPIO_BSRR_BR3 | GPIO_BSRR_BR7); // PA3 (M1_EN), PA7 (M2_EN) LOW
    TIM2->CR1 &= ~TIM_CR1_UDIS;
    TIM2->CCR1 = 0; TIM2->CCR2 = 0; TIM2->CCR3 = 0;
    TIM2->EGR  = TIM_EGR_UG;
    TIM3->CR1 &= ~TIM_CR1_UDIS;
    TIM3->CCR1 = 0; TIM3->CCR2 = 0; TIM3->CCR3 = 0;
    TIM3->EGR  = TIM_EGR_UG;
    m1_enabled = false;
    m2_enabled = false;
    motor1.enabled = 0;
    motor2.enabled = 0;
    motor1.voltage.q = 0.0f;
    motor1.voltage.d = 0.0f;
    motor2.voltage.q = 0.0f;
    motor2.voltage.d = 0.0f;
}

extern "C" {
void HardFault_Handler(void)  { __disable_irq(); hardwareKillMotorsImmediate(); NVIC_SystemReset(); while (1) {} }
void MemManage_Handler(void)  { __disable_irq(); hardwareKillMotorsImmediate(); NVIC_SystemReset(); while (1) {} }
void BusFault_Handler(void)   { __disable_irq(); hardwareKillMotorsImmediate(); NVIC_SystemReset(); while (1) {} }
void UsageFault_Handler(void) { __disable_irq(); hardwareKillMotorsImmediate(); NVIC_SystemReset(); while (1) {} }
}

static inline void iwdgKick() {
    IWDG->KR = 0xAAAAU;
}

static void iwdgInit500ms() {
    IWDG->KR = 0xCCCCU;
    IWDG->KR = 0x5555U;
    IWDG->PR = 0x03U;
    IWDG->RLR = 500U;
    uint32_t t0 = millis();
    while ((IWDG->SR != 0U) && ((millis() - t0) < 50U)) {
    }
    IWDG->KR = 0xAAAAU;
}

// ----------------------------------------------------------------------------
// Direct Inlined Hardware FOC Commutation Loop (< 60 cycles / ~0.35 us)
// Completely eliminates SimpleFOC virtual dispatch, _micros() polling, and function overhead
// ----------------------------------------------------------------------------
static inline float fast_fminf(float a, float b) __attribute__((always_inline));
static inline float fast_fminf(float a, float b) { return (a < b) ? a : b; }

static inline float fast_fmaxf(float a, float b) __attribute__((always_inline));
static inline float fast_fmaxf(float a, float b) { return (a > b) ? a : b; }

static inline void fastLoopFOC(BLDCMotor &motor, MT6816Sensor &encoder, FastBLDCDriver3PWM &driver, TIM_TypeDef *tim) __attribute__((always_inline));

static inline void fastLoopFOC(BLDCMotor &motor, MT6816Sensor &encoder, FastBLDCDriver3PWM &driver, TIM_TypeDef *tim) {
    if (!motor.enabled) return;

    // 1. Electrical angle from calibrated mechanical angle
    float mech_angle = encoder.cached_calibrated_angle;
    float el_ang = (float)(motor.sensor_direction * motor.pole_pairs) * mech_angle - motor.zero_electric_angle;
    el_ang = _normalizeAngle(el_ang);
    motor.electrical_angle = el_ang;

    // 2. Hardware CORDIC sin & cos
    float sa, ca;
    _sincos(el_ang, &sa, &ca);

    // 3. Inverse Park Transform
    float uq = _constrain(motor.voltage.q, -motor.voltage_limit, motor.voltage_limit);
    float ud = motor.voltage.d;

    float u_alpha = ca * ud - sa * uq;
    float u_beta  = sa * ud + ca * uq;

    // 4. Clarke Transform
    float ua = u_alpha;
    float ub = -0.5f * u_alpha + 0.866025404f * u_beta;
    float uc = -0.5f * u_alpha - 0.866025404f * u_beta;

    // 5. SpaceVectorPWM Midpoint Centering
    float umin = fast_fminf(ua, fast_fminf(ub, uc));
    float umax = fast_fmaxf(ua, fast_fmaxf(ub, uc));
    float center = (driver.voltage_limit * 0.5f) - (umax + umin) * 0.5f;

    ua += center;
    ub += center;
    uc += center;

    // 6. Direct hardware timer CCR update with atomic shadow-register latching (TIM_CR1_UDIS)
    uint32_t duty_a = (uint32_t)(fast_fminf(fast_fmaxf(ua, 0.0f), driver.voltage_limit) * driver.pwm_scale);
    uint32_t duty_b = (uint32_t)(fast_fminf(fast_fmaxf(ub, 0.0f), driver.voltage_limit) * driver.pwm_scale);
    uint32_t duty_c = (uint32_t)(fast_fminf(fast_fmaxf(uc, 0.0f), driver.voltage_limit) * driver.pwm_scale);

    if (tim == TIM2) {
        TIM2->CR1 |= TIM_CR1_UDIS;
        TIM2->CCR1 = duty_a; // PA5 (TIM2 CH1)
        TIM2->CCR2 = duty_b; // PA1 (TIM2 CH2)
        TIM2->CCR3 = duty_c; // PA2 (TIM2 CH3)
        TIM2->CR1 &= ~TIM_CR1_UDIS;
    } else {
        TIM3->CR1 |= TIM_CR1_UDIS;
        TIM3->CCR3 = duty_a; // PB0 (TIM3 CH3)
        TIM3->CCR2 = duty_b; // PA4 (TIM3 CH2)
        TIM3->CCR1 = duty_c; // PA6 (TIM3 CH1)
        TIM3->CR1 &= ~TIM_CR1_UDIS;
    }
}

// ----------------------------------------------------------------------------
// Direct Inlined 5.0 kHz Staggered Velocity/Torque Control
// Decouples Ke*omega_meas, interpolates 400Hz->5kHz, clamps I, slews V_trq
// ----------------------------------------------------------------------------
static inline void fastMoveVelocity(BLDCMotor &motor, MT6816Sensor &encoder, MotorTorqueCtrl &ctrl) __attribute__((always_inline));

static inline void fastMoveVelocity(BLDCMotor &motor, MT6816Sensor &encoder, MotorTorqueCtrl &ctrl) {
    constexpr float dt = 0.0002f; // 5.0 kHz
    motor.shaft_angle = (float)motor.sensor_direction * encoder.getFastAngle() - motor.sensor_offset;
    float raw_vel = encoder.getVelocity();
    motor.shaft_velocity = (float)motor.sensor_direction * motor.LPF_velocity(raw_vel);
    if (!motor.enabled) {
        ctrl.resetIsrState(motor.shaft_velocity);
        return;
    }

    uint32_t seq = ctrl.cmd_seq;
    float cmd_omega = ctrl.cmd_omega_rad_s;
    float raw_vmin  = ctrl.cmd_vtrq_min;
    float raw_vmax  = ctrl.cmd_vtrq_max;
    float raw_vff   = ctrl.cmd_vtrq_ff;

    // 1. Linear interpolation of 400 Hz commands across 12.5 ticks (2.5 ms)
    if (seq != ctrl.isr_last_seq) {
        ctrl.isr_last_seq = seq;
        ctrl.omega_step_per_tick = (cmd_omega - ctrl.omega_ref_interp) * (1.0f / CMD_INTERP_TICKS);
        ctrl.interp_ticks_rem = 13U;
    }
    if (ctrl.interp_ticks_rem > 0U) {
        float next_omega = ctrl.omega_ref_interp + ctrl.omega_step_per_tick;
        if ((ctrl.omega_step_per_tick >= 0.0f && next_omega >= cmd_omega) ||
            (ctrl.omega_step_per_tick <= 0.0f && next_omega <= cmd_omega) ||
            ctrl.interp_ticks_rem == 1U) {
            ctrl.omega_ref_interp = cmd_omega;
            ctrl.interp_ticks_rem = 0U;
        } else {
            ctrl.omega_ref_interp = next_omega;
            ctrl.interp_ticks_rem--;
        }
    } else {
        ctrl.omega_ref_interp = cmd_omega;
    }
    motor.target = cmd_omega;
    motor.shaft_velocity_sp = ctrl.omega_ref_interp;

    // 2. Back-EMF decoupling using measured velocity (SVPWM phase-peak Ke)
    float v_bemf = ctrl.ke_q * motor.shaft_velocity;
    ctrl.last_v_bemf = v_bemf;

    // 3. Torque voltage envelope constrained by motor limit & stall foldback cap
    float v_cap = fast_fminf(motor.voltage_limit, ctrl.stall_vtrq_cap);
    float v_min = _constrain(raw_vmin, -v_cap, v_cap);
    float v_max = _constrain(raw_vmax, -v_cap, v_cap);
    if (v_min > v_max) {
        float mid = 0.5f * (v_min + v_max);
        v_min = mid;
        v_max = mid;
    }

    // 4. Overspeed-guarded feedforward + viscous-LSD PI with tight I-clamp
    float err = ctrl.omega_ref_interp - motor.shaft_velocity;
    float p_term = motor.PID_velocity.P * err;

    float v_ff_eff = raw_vff;
    if (v_ff_eff > 0.0f && err < 0.0f) {
        float scale = 1.0f + err * (1.0f / FF_OVERSPEED_TAPER_RAD_S);
        v_ff_eff *= (scale > 0.0f) ? scale : 0.0f;
    } else if (v_ff_eff < 0.0f && err > 0.0f) {
        float scale = 1.0f - err * (1.0f / FF_OVERSPEED_TAPER_RAD_S);
        v_ff_eff *= (scale > 0.0f) ? scale : 0.0f;
    }

    float i_lo = fast_fmaxf(-ctrl.i_clamp_v, v_min);
    float i_hi = fast_fminf( ctrl.i_clamp_v, v_max);
    float integral_new = ctrl.integral_v;
    if (i_lo > i_hi) {
        integral_new = 0.5f * (v_min + v_max);
    } else {
        float cand_i = ctrl.integral_v + motor.PID_velocity.I * dt * 0.5f * (err + ctrl.err_prev);
        cand_i = _constrain(cand_i, i_lo, i_hi);
        float pre_sat = v_ff_eff + p_term + cand_i;
        if ((pre_sat > v_max && err > 0.0f && cand_i > ctrl.integral_v) ||
            (pre_sat < v_min && err < 0.0f && cand_i < ctrl.integral_v)) {
            integral_new = _constrain(ctrl.integral_v, i_lo, i_hi);
        } else {
            integral_new = cand_i;
        }
    }
    ctrl.integral_v = integral_new;

    float d_term = (motor.PID_velocity.D > 0.0f)
                       ? (motor.PID_velocity.D * (err - ctrl.err_prev) * (1.0f / dt))
                       : 0.0f;
    ctrl.err_prev = err;

    float v_trq_target = _constrain(v_ff_eff + p_term + integral_new + d_term, v_min, v_max);

    // 5. Slew-rate limit V_trq ONLY (never total V_q)
    float ramp = motor.PID_velocity.output_ramp;
    if (ramp > 0.0f) {
        float max_dv = ramp * dt;
        float dv = v_trq_target - ctrl.v_trq_slewed;
        if (dv > max_dv) {
            ctrl.v_trq_slewed += max_dv;
        } else if (dv < -max_dv) {
            ctrl.v_trq_slewed -= max_dv;
        } else {
            ctrl.v_trq_slewed = v_trq_target;
        }
    } else {
        ctrl.v_trq_slewed = v_trq_target;
    }
    ctrl.v_trq_slewed = _constrain(ctrl.v_trq_slewed, -v_cap, v_cap);

    // 6. Locked 90-deg commutation (Iq only, Vd = 0)
    motor.current_sp = ctrl.v_trq_slewed;
    motor.voltage.q = _constrain(v_bemf + ctrl.v_trq_slewed, -motor.voltage_limit, motor.voltage_limit);
    motor.voltage.d = 0.0f;
}

// ----------------------------------------------------------------------------
// Hard Real-Time 20.0 kHz Hardware Timer ISR (TIM6, NVIC Priority 0)
// ----------------------------------------------------------------------------
void focTimerISR() {
    uint32_t c0 = DWT->CYCCNT;
    if (isr_last_c0 != 0) {
        uint32_t period = c0 - isr_last_c0;
        prof_period_last = period;
        if (period < prof_period_min) prof_period_min = period;
        if (period > prof_period_max) prof_period_max = period;
        uint32_t jit = (period >= 8500) ? (period - 8500) : (8500 - period);
        if (jit > prof_period_jitter_max) prof_period_jitter_max = jit;
        // Count inter-ISR period jitter spike if period deviates by > 5.0 us (850 cycles)
        if (jit > 850) {
            isr_jitter_spikes++;
        }
    }
    isr_last_c0 = c0;
    foc_tick_count++;

    // 1. Encoder SPI Read Engine
    if (m1_calib_active) {
        // High-Precision Calibration Isolation for Motor 1:
        // Read ONLY Encoder 1 on SPI1. SPI2 and Encoder 2 are completely silenced
        // (CS2 held HIGH, zero 10.6 MHz clock switching on the PCB, zero digital cross-talk).
        // ISR duration drops to < 1.8 us (< 3.6% CPU load).
        readEncoderSingleFast(encoder1, SPI1);
        encoder1.update();
        uint32_t d_tot = DWT->CYCCNT - c0;
        isr_last_cycles = d_tot;
        if (d_tot > isr_peak_cycles) isr_peak_cycles = d_tot;
        return;
    } else if (m2_calib_active) {
        // High-Precision Calibration Isolation for Motor 2:
        // Read ONLY Encoder 2 on SPI2. SPI1 and Encoder 1 are completely silenced.
        readEncoderSingleFast(encoder2, SPI2);
        encoder2.update();
        uint32_t d_tot = DWT->CYCCNT - c0;
        isr_last_cycles = d_tot;
        if (d_tot > isr_peak_cycles) isr_peak_cycles = d_tot;
        return;
    }

    // Normal Dual-Motor Operation: Concurrent Parallel Dual-SPI Read (< 3.6 us total)
    readDualEncodersParallel(encoder1, encoder2);
    uint32_t c1 = DWT->CYCCNT;

    encoder1.update();
    encoder2.update();
    uint32_t c2 = DWT->CYCCNT;

    // 2. Staggered 5.0 kHz Velocity Filtering & Decoupled Torque Control (downsample = 4, 200 us period)
    uint32_t phase = foc_tick_count & 3U;
    if (phase == 0U && !m1_calib_active) {
        if (m1_enabled) {
            fastMoveVelocity(motor1, encoder1, m1_ctrl);
        } else {
            motor1.shaft_velocity = (float)motor1.sensor_direction * motor1.LPF_velocity(encoder1.getVelocity());
            m1_ctrl.resetIsrState(motor1.shaft_velocity);
        }
    } else if (phase == 2U && !m2_calib_active) {
        if (m2_enabled) {
            fastMoveVelocity(motor2, encoder2, m2_ctrl);
        } else {
            motor2.shaft_velocity = (float)motor2.sensor_direction * motor2.LPF_velocity(encoder2.getVelocity());
            m2_ctrl.resetIsrState(motor2.shaft_velocity);
        }
    } else if (phase == 1U) {
        // 5.0 kHz (200 us) ISR-level ESP32 command-age watchdog: disarms motors in
        // <= 100 ms even if loop() stalls while TIM6 keeps ticking.
        static uint32_t s_isr_last_esp_seq = 0;
        static uint32_t s_isr_esp_silent_ticks = 0;
        if (g_esp_link_active_isr) {
            uint32_t rx_seq = g_esp_rx_seq_isr;
            if (rx_seq != s_isr_last_esp_seq) {
                s_isr_last_esp_seq = rx_seq;
                s_isr_esp_silent_ticks = 0;
            } else if (m1_enabled || m2_enabled) {
                s_isr_esp_silent_ticks++;
                if (s_isr_esp_silent_ticks >= 500U) {
                    esp_wdog_trips++;
                    g_estop_active = true;
                    hardwareKillMotorsImmediate();
                    m1_ctrl.cmd_omega_rad_s = 0.0f;
                    m1_ctrl.cmd_vtrq_min = -MOTOR_VOLTAGE_LIMIT;
                    m1_ctrl.cmd_vtrq_max = MOTOR_VOLTAGE_LIMIT;
                    m1_ctrl.cmd_vtrq_ff = 0.0f;
                    m1_ctrl.cmd_seq++;
                    m1_ctrl.resetIsrState(motor1.shaft_velocity);
                    m2_ctrl.cmd_omega_rad_s = 0.0f;
                    m2_ctrl.cmd_vtrq_min = -MOTOR_VOLTAGE_LIMIT;
                    m2_ctrl.cmd_vtrq_max = MOTOR_VOLTAGE_LIMIT;
                    m2_ctrl.cmd_vtrq_ff = 0.0f;
                    m2_ctrl.cmd_seq++;
                    m2_ctrl.resetIsrState(motor2.shaft_velocity);
                    m1_target_rpm = 0.0f;
                    m2_target_rpm = 0.0f;
                    s_isr_esp_silent_ticks = 0;
                }
            } else {
                s_isr_esp_silent_ticks = 0;
            }
        }
    }
    uint32_t c3 = DWT->CYCCNT;

    // 3. Motor 1 Commutation Loop (with Smooth Filtered Sensor Lag Compensation)
    if (m1_enabled && !m1_calib_active) {
        if (m1_lag_comp_us > 0.0f && m1_base_zero_angle != 0.0f) {
            // Crisp 2.0 ms low-pass filter on lag compensation velocity (alpha = 0.024f @ 20 kHz)
            // Eliminates torque-commutation feedback ripple with no sluggish transient lag
            m1_lag_filt_vel += 0.024f * (motor1.shaft_velocity - m1_lag_filt_vel);
            if (fabsf(m1_lag_filt_vel) > 3.0f) {
                float lead_rad = m1_lag_filt_vel * (float)MOTOR_POLE_PAIRS * (m1_lag_comp_us * 1e-6f);
                motor1.zero_electric_angle = normalizeAngleRad(m1_base_zero_angle - lead_rad);
            } else {
                motor1.zero_electric_angle = m1_base_zero_angle;
            }
        }
        fastLoopFOC(motor1, encoder1, driver1, TIM2);
    }
    uint32_t c4 = DWT->CYCCNT;

    // 4. Motor 2 Commutation Loop (with Smooth Filtered Sensor Lag Compensation)
    if (m2_enabled && !m2_calib_active) {
        if (m2_lag_comp_us > 0.0f && m2_base_zero_angle != 0.0f) {
            m2_lag_filt_vel += 0.024f * (motor2.shaft_velocity - m2_lag_filt_vel);
            if (fabsf(m2_lag_filt_vel) > 3.0f) {
                float lead_rad = m2_lag_filt_vel * (float)MOTOR_POLE_PAIRS * (m2_lag_comp_us * 1e-6f);
                motor2.zero_electric_angle = normalizeAngleRad(m2_base_zero_angle - lead_rad);
            } else {
                motor2.zero_electric_angle = m2_base_zero_angle;
            }
        }
        fastLoopFOC(motor2, encoder2, driver2, TIM3);
    }
    uint32_t c5 = DWT->CYCCNT;

    if (foc_trace_active && (foc_tick_count % FOC_TRACE_DECIMATION) == 0U) {
        uint16_t index = foc_trace_write_index;
        volatile FocTraceSample &sample = foc_trace[index];
        sample.m1_raw_angle = encoder1.last_raw_14bit;
        sample.m2_raw_angle = encoder2.last_raw_14bit;
        sample.m1_target_rpm = (int16_t)_constrain(m1_target_rpm, -32768.0f, 32767.0f);
        sample.m2_target_rpm = (int16_t)_constrain(m2_target_rpm, -32768.0f, 32767.0f);
        sample.m1_actual_rpm = (int16_t)_constrain(motor1.shaftVelocity() * 9.54929659f, -32768.0f, 32767.0f);
        sample.m2_actual_rpm = (int16_t)_constrain(motor2.shaftVelocity() * 9.54929659f, -32768.0f, 32767.0f);
        sample.m1_uq_mv = (int16_t)_constrain(motor1.voltage.q * 1000.0f, -32768.0f, 32767.0f);
        sample.m2_uq_mv = (int16_t)_constrain(motor2.voltage.q * 1000.0f, -32768.0f, 32767.0f);

        foc_trace_write_index = (uint16_t)((index + 1U) % FOC_TRACE_CAPACITY);
        if (foc_trace_count < FOC_TRACE_CAPACITY) {
            foc_trace_count++;
        } else {
            foc_trace_overwritten++;
        }
    }

    // 5. Profiling & Jitter Tracking
    uint32_t d_spi = c1 - c0;
    uint32_t d_upd = c2 - c1;
    uint32_t d_mov = c3 - c2;
    uint32_t d_m1  = c4 - c3;
    uint32_t d_m2  = c5 - c4;
    uint32_t d_tot = c5 - c0;

    prof_spi_last = d_spi;
    prof_update_last = d_upd;
    prof_m1_foc_last = d_m1;
    prof_m2_foc_last = d_m2;
    prof_move_last = d_mov;
    prof_total_last = d_tot;

    if (d_spi > prof_spi_max) prof_spi_max = d_spi;
    if (d_upd > prof_update_max) prof_update_max = d_upd;
    if (d_m1 > prof_m1_foc_max) prof_m1_foc_max = d_m1;
    if (d_m2 > prof_m2_foc_max) prof_m2_foc_max = d_m2;
    if (d_mov > prof_move_max) prof_move_max = d_mov;
    if (d_tot > prof_total_max) prof_total_max = d_tot;

    isr_last_cycles = d_tot;
    if (d_tot > isr_peak_cycles) {
        isr_peak_cycles = d_tot;
    }
    if (d_tot > isr_alltime_peak_cycles) {
        isr_alltime_peak_cycles = d_tot;
    }
    // Jitter spike warning threshold: > 35.0 us @ 170 MHz = 5,950 cycles (50.0 us total tick budget)
    if (d_tot > 5950) {
        isr_jitter_spikes++;
    }
}

// ----------------------------------------------------------------------------
// Pre-Loaded Zero Angle Initialization Helper
// Motor Initialization Helper (Eliminates SimpleFOC open-loop alignSensor & virtual loopFOC)
// ----------------------------------------------------------------------------
void initMotorFOC(int motor_id) {
    if (motor_id == 1) {
        motor1.enable();
        if (M1_DEFAULT_ZERO_ELECTRIC_ANGLE != 0.0f) {
            motor1.zero_electric_angle = M1_DEFAULT_ZERO_ELECTRIC_ANGLE;
            m1_base_zero_angle = M1_DEFAULT_ZERO_ELECTRIC_ANGLE;
            motor1.sensor_direction = Direction::CW;
            motor1.initFOC();
            m1_calibrated = true;
            Serial.println(">>> M1: Silent alignment loaded from config (No jerk).");
        } else {
            Serial.println(">>> M1: Running standard SimpleFOC alignment...");
            if (motor1.initFOC()) {
                m1_base_zero_angle = motor1.zero_electric_angle;
                m1_calibrated = true;
                Serial.print(">>> M1 ALIGN OK: ZeroAngle=");
                Serial.print(motor1.zero_electric_angle, 4);
                Serial.print(" Dir=");
                Serial.println(motor1.sensor_direction == Direction::CW ? "CW" : "CCW");
            } else {
                m1_calibrated = false;
                m1_enabled = false;
                motor1.disable();
                Serial.println(">>> M1 ALIGN FAILED!");
            }
        }
    } else if (motor_id == 2) {
        motor2.enable();
        if (M2_DEFAULT_ZERO_ELECTRIC_ANGLE != 0.0f) {
            motor2.zero_electric_angle = M2_DEFAULT_ZERO_ELECTRIC_ANGLE;
            m2_base_zero_angle = M2_DEFAULT_ZERO_ELECTRIC_ANGLE;
            motor2.sensor_direction = Direction::CW;
            motor2.initFOC();
            m2_calibrated = true;
            Serial.println(">>> M2: Silent alignment loaded from config (No jerk).");
        } else {
            Serial.println(">>> M2: Running standard SimpleFOC alignment...");
            if (motor2.initFOC()) {
                m2_base_zero_angle = motor2.zero_electric_angle;
                m2_calibrated = true;
                Serial.print(">>> M2 ALIGN OK: ZeroAngle=");
                Serial.print(motor2.zero_electric_angle, 4);
                Serial.print(" Dir=");
                Serial.println(motor2.sensor_direction == Direction::CW ? "CW" : "CCW");
            } else {
                m2_calibrated = false;
                m2_enabled = false;
                motor2.disable();
                Serial.println(">>> M2 ALIGN FAILED!");
            }
        }
    }
    BLDCMotor &motor = (motor_id == 1) ? motor1 : motor2;
    FastBLDCDriver3PWM &driver = (motor_id == 1) ? driver1 : driver2;
    MT6816Sensor &encoder = (motor_id == 1) ? encoder1 : encoder2;
    MotorTorqueCtrl &ctrl = (motor_id == 1) ? m1_ctrl : m2_ctrl;
    volatile float &target_rpm_var = (motor_id == 1) ? m1_target_rpm : m2_target_rpm;
    volatile bool &calibrated = (motor_id == 1) ? m1_calibrated : m2_calibrated;
    volatile bool &enabled = (motor_id == 1) ? m1_enabled : m2_enabled;

    ctrl.updateKe(motor.KV_rating);
    driver.enable();
    motor.enable();
    motor.voltage.q = 0.0f;
    motor.voltage.d = 0.0f;
    encoder.update();
    motor.shaft_angle = (float)motor.sensor_direction * encoder.getFastAngle() - motor.sensor_offset;
    resetMotorTorqueCtrl(ctrl, target_rpm_var, motor.shaft_velocity, true);
    enabled = true;
    calibrated = true;
    Serial.print("[OK]:INIT_M"); Serial.println(motor_id);
}

// ----------------------------------------------------------------------------
// Calibration Helpers & Angle Normalization
// ----------------------------------------------------------------------------
void sendBinaryTelemetry(); // Forward declaration for real-time telemetry streaming

// ----------------------------------------------------------------------------
// Global Emergency Stop & Calibration Abort Engine
// ----------------------------------------------------------------------------
static bool s_calib_isolation_active = false;
static bool s_saved_telemetry_state = true;

void disableBothDrivers() {
    m1_enabled = false;
    m2_enabled = false;
    resetMotorTorqueCtrl(m1_ctrl, m1_target_rpm, motor1.shaft_velocity, true);
    resetMotorTorqueCtrl(m2_ctrl, m2_target_rpm, motor2.shaft_velocity, true);

    motor1.voltage.q = 0.0f;
    motor1.voltage.d = 0.0f;
    motor2.voltage.q = 0.0f;
    motor2.voltage.d = 0.0f;

    motor1.setPhaseVoltage(0.0f, 0.0f, 0.0f);
    motor2.setPhaseVoltage(0.0f, 0.0f, 0.0f);

    motor1.disable();
    motor2.disable();
    driver1.disable();
    driver2.disable();
    digitalWrite(M1_EN, LOW);
    digitalWrite(M2_EN, LOW);

    stall_guard1.reset();
    stall_guard2.reset();
}

void triggerGlobalEmergencyStop() {
    bool already_stopped = g_estop_active && !m1_enabled && !m2_enabled &&
                           !m1_calib_active && !m2_calib_active &&
                           !s_calib_isolation_active;
    g_estop_active = true;
    m1_calib_active = false;
    m2_calib_active = false;
    live_calib_progress = 0;
    s_calib_isolation_active = false;
    show_telemetry = s_saved_telemetry_state;

    disableBothDrivers();

    motor1.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor1.PID_velocity.limit = motor1.voltage_limit;
    motor2.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor2.PID_velocity.limit = motor2.voltage_limit;

    while (Serial.available()) Serial.read();

    if (!already_stopped) {
        Serial.println("\n>>> 🛑 GLOBAL EMERGENCY STOP: ALL MOTORS SHUT DOWN UNCONDITIONALLY! <<<");
        Serial.println("[CALIB_FAILED]:ABORTED_BY_ESTOP");
    }
}

void processESP32Link();
void sendESP32Feedback();

static inline bool checkCalibrationAbort() {
    iwdgKick();
    if (g_estop_active) return true;
    processESP32Link();
    if (g_estop_active) return true;
    if (esp_link_active && (millis() >= last_esp_packet_ms) &&
        (millis() - last_esp_packet_ms > 250)) {
        triggerGlobalEmergencyStop();
        return true;
    }
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        // Dedicated single-byte instant emergency stop hotkey: ESC (ASCII 27)
        if (c == 27) {
            rx_idx = 0;
            triggerGlobalEmergencyStop();
            return true;
        }
        // Accumulate into incoming line buffer so commands like x, STOP, or ESTOP work cleanly
        if (c == '\n' || c == '\r') {
            if (rx_idx > 0) {
                rx_line[rx_idx] = '\0';
                if (strcasecmp(rx_line, "x") == 0 || strcasecmp(rx_line, "STOP") == 0 ||
                    strcasecmp(rx_line, "ESTOP") == 0 || strcasecmp(rx_line, "HALT") == 0 ||
                    strcasecmp(rx_line, "ABORT") == 0) {
                    rx_idx = 0;
                    triggerGlobalEmergencyStop();
                    return true;
                }
                rx_idx = 0;
            }
        } else if (rx_idx < sizeof(rx_line) - 1) {
            rx_line[rx_idx++] = c;
        }
    }
    return false;
}

static inline bool safeDelayWithAbort(uint32_t ms) {
    uint32_t start = millis();
    while (millis() - start < ms) {
        if (checkCalibrationAbort()) return true;
        uint32_t rem = ms - (millis() - start);
        delay(rem > 10 ? 10 : rem);
    }
    return checkCalibrationAbort();
}

// ----------------------------------------------------------------------------
// Dedicated Calibration Isolation & Environment Shield
// ----------------------------------------------------------------------------
void enterCalibrationIsolation(int motor_id) {
    if (s_calib_isolation_active) return;
    s_calib_isolation_active = true;
    s_saved_telemetry_state = show_telemetry;
    show_telemetry = false; // Mute all binary telemetry streaming

    int other_id = (motor_id == 1) ? 2 : 1;
    BLDCMotor &other_motor = (motor_id == 1) ? motor2 : motor1;
    BLDCDriver3PWM &other_driver = (motor_id == 1) ? driver2 : driver1;
    MotorTorqueCtrl &other_ctrl = (motor_id == 1) ? m2_ctrl : m1_ctrl;
    volatile bool &other_enabled = (motor_id == 1) ? m2_enabled : m1_enabled;
    volatile float &other_target_rpm = (motor_id == 1) ? m2_target_rpm : m1_target_rpm;

    // 1. Completely disable the other motor and inverter hardware (Hi-Z, zero switching/ripple/EMI)
    other_enabled = false;
    resetMotorTorqueCtrl(other_ctrl, other_target_rpm, other_motor.shaft_velocity, true);
    other_motor.voltage.q = 0.0f;
    other_motor.voltage.d = 0.0f;
    other_motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
    other_motor.disable();
    other_driver.disable(); // Shuts down gate drivers/inverter half-bridges
    digitalWrite((motor_id == 1) ? M2_EN : M1_EN, LOW); // Enforce hardware EN pin LOW

    // 2. Silence inactive encoder SPI bus (hold CS HIGH, MISO Hi-Z, zero 10.6 MHz clock noise)
    if (motor_id == 1) {
        encoder2.csHigh();
        stall_guard2.reset();
        m1_calib_active = true;
        m2_calib_active = false;
    } else {
        encoder1.csHigh();
        stall_guard1.reset();
        m1_calib_active = false;
        m2_calib_active = true;
    }
    live_calib_progress = 0;

    Serial.print("[CALIB_START]:M"); Serial.println(motor_id);
    Serial.println("[CALIB_PROG]:0%");
    Serial.flush();
}

void exitCalibrationIsolation(int motor_id) {
    if (!s_calib_isolation_active) return;
    s_calib_isolation_active = false;
    m1_calib_active = false;
    m2_calib_active = false;
    live_calib_progress = 0;

    // Restore telemetry streaming state
    show_telemetry = s_saved_telemetry_state;

    Serial.print("[CALIB_END]:M"); Serial.println(motor_id);
    Serial.flush();
}

struct CalibrationIsolationGuard {
    int m_id;
    bool was_active_before;
    CalibrationIsolationGuard(int motor_id) : m_id(motor_id), was_active_before(s_calib_isolation_active) {
        if (!was_active_before) {
            enterCalibrationIsolation(m_id);
        }
    }
    ~CalibrationIsolationGuard() {
        if (!was_active_before) {
            exitCalibrationIsolation(m_id);
        }
    }
};

// Circular 1D inverse-distance interpolation from measured samples to 128 raw LUT segments
static float interpolateLUTAtAngle(float target_rad, const float* sample_angles, const float* sample_errors, int n) {
    target_rad = normalizeAngleRad(target_rad);
    float best_dist1 = 1e9f, best_dist2 = 1e9f;
    int idx1 = 0, idx2 = 0;

    for (int i = 0; i < n; i++) {
        float d = fabsf(wrapAngleRad(sample_angles[i] - target_rad));
        if (d < best_dist1) {
            best_dist2 = best_dist1;
            idx2 = idx1;
            best_dist1 = d;
            idx1 = i;
        } else if (d < best_dist2) {
            best_dist2 = d;
            idx2 = i;
        }
    }

    float total_dist = best_dist1 + best_dist2;
    if (total_dist < 1e-6f) return sample_errors[idx1];
    float w1 = best_dist2 / total_dist;
    float w2 = best_dist1 / total_dist;
    return (w1 * sample_errors[idx1] + w2 * sample_errors[idx2]);
}

// ----------------------------------------------------------------------------
// LUT Data Dump & Serialization Helper
// ----------------------------------------------------------------------------
void dumpLUT(int motor_id) {
    MT6816Sensor &encoder = (motor_id == 1) ? encoder1 : encoder2;
    Serial.print("[LUT_DATA_START]:M"); Serial.println(motor_id);
    for (int line = 0; line < 8; line++) {
        Serial.print("LUT "); Serial.print(motor_id); Serial.print(" "); Serial.print(line); Serial.print(" ");
        for (int j = 0; j < 16; j++) {
            Serial.print(encoder.lut_errors[line * 16 + j], 6);
            if (j < 15) Serial.print(",");
        }
        Serial.println();
    }
    Serial.print("[LUT_DATA_END]:M"); Serial.println(motor_id);
}

// ----------------------------------------------------------------------------
// JSON Configuration Exporter (Capturable by Python GUI)
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// Persistent Flash Storage (Direct STM32 Hardware Flash Page Management)
// ----------------------------------------------------------------------------
#define FOC_CONFIG_MAGIC 0x464F4331UL // "FOC1"

struct __attribute__((aligned(8))) PersistentFOCConfig {
    uint32_t magic;
    float    m1_zero_angle;
    int8_t   m1_direction;
    float    m1_lag_comp_us;
    float    m1_kv_rating;
    float    m1_pid_p;
    float    m1_pid_i;
    float    m1_pid_d;
    float    m1_lpf_tf;
    float    m1_velocity_limit;
    float    m1_dwt_ms;
    bool     m1_lut_active;
    float    m1_lut_errors[128];

    float    m2_zero_angle;
    int8_t   m2_direction;
    float    m2_lag_comp_us;
    float    m2_kv_rating;
    float    m2_pid_p;
    float    m2_pid_i;
    float    m2_pid_d;
    float    m2_lpf_tf;
    float    m2_velocity_limit;
    float    m2_dwt_ms;
    bool     m2_lut_active;
    float    m2_lut_errors[128];

    uint32_t checksum;
};

static uint32_t getFlashConfigPageAddress(uint32_t *out_page_idx = nullptr) {
    uint16_t flash_kb = *(const uint16_t *)0x1FFF75E0UL; // Electronic signature register
    if (flash_kb < 128 || flash_kb > 1024) flash_kb = 128;
    uint32_t total_pages = flash_kb / 2; // 2KB per page on STM32G4
    uint32_t target_page = total_pages - 1;
    if (out_page_idx) *out_page_idx = target_page;
    return 0x08000000UL + (target_page * 2048UL);
}

static uint32_t computeConfigChecksum(const PersistentFOCConfig &cfg) {
    uint32_t chk = 0x55AA55AAUL;
    const uint8_t *p = (const uint8_t *)&cfg;
    size_t len = offsetof(PersistentFOCConfig, checksum);
    for (size_t i = 0; i < len; i++) {
        chk = (chk << 1) ^ p[i];
    }
    return chk;
}

extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;

bool saveConfigToFlash() {
    uint32_t page_idx = 63;
    uint32_t page_addr = getFlashConfigPageAddress(&page_idx);

    uint32_t flash_binary_end = (uint32_t)&_sidata + ((uint32_t)&_edata - (uint32_t)&_sdata);
    if (flash_binary_end >= page_addr) {
        Serial.print("[FLASH_ERR] Binary encroaches on Page "); Serial.print(page_idx);
        Serial.print(" (end 0x"); Serial.print(flash_binary_end, HEX);
        Serial.print(" >= page 0x"); Serial.print(page_addr, HEX); Serial.println(")");
        return false;
    }

    PersistentFOCConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.magic = FOC_CONFIG_MAGIC;

    cfg.m1_zero_angle = motor1.zero_electric_angle;
    cfg.m1_direction = (motor1.sensor_direction == Direction::CW) ? 1 : -1;
    cfg.m1_lag_comp_us = m1_lag_comp_us;
    cfg.m1_kv_rating = motor1.KV_rating;
    cfg.m1_pid_p = motor1.PID_velocity.P;
    cfg.m1_pid_i = motor1.PID_velocity.I;
    cfg.m1_pid_d = motor1.PID_velocity.D;
    cfg.m1_lpf_tf = motor1.LPF_velocity.Tf;
    cfg.m1_velocity_limit = motor1.velocity_limit;
    cfg.m1_dwt_ms = encoder1.window_ms;
    cfg.m1_lut_active = encoder1.is_lut_active;
    for (int i = 0; i < 128; i++) {
        cfg.m1_lut_errors[i] = encoder1.lut_errors[i];
    }

    cfg.m2_zero_angle = motor2.zero_electric_angle;
    cfg.m2_direction = (motor2.sensor_direction == Direction::CW) ? 1 : -1;
    cfg.m2_lag_comp_us = m2_lag_comp_us;
    cfg.m2_kv_rating = motor2.KV_rating;
    cfg.m2_pid_p = motor2.PID_velocity.P;
    cfg.m2_pid_i = motor2.PID_velocity.I;
    cfg.m2_pid_d = motor2.PID_velocity.D;
    cfg.m2_lpf_tf = motor2.LPF_velocity.Tf;
    cfg.m2_velocity_limit = motor2.velocity_limit;
    cfg.m2_dwt_ms = encoder2.window_ms;
    cfg.m2_lut_active = encoder2.is_lut_active;
    for (int i = 0; i < 128; i++) {
        cfg.m2_lut_errors[i] = encoder2.lut_errors[i];
    }

    cfg.checksum = computeConfigChecksum(cfg);

    // Disable motor PWM and pause 20kHz FOC interrupt during flash write
    motor1.disable();
    motor2.disable();
    driver1.disable();
    driver2.disable();
    digitalWrite(M1_EN, LOW);
    digitalWrite(M2_EN, LOW);
    motor1.voltage.q = 0.0f;
    motor1.voltage.d = 0.0f;
    motor2.voltage.q = 0.0f;
    motor2.voltage.d = 0.0f;

    stall_guard1.reset();
    stall_guard2.reset();
    motor1.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor1.PID_velocity.limit = motor1.voltage_limit;
    motor2.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor2.PID_velocity.limit = motor2.voltage_limit;
    m1_enabled = false;
    m2_enabled = false;
    resetMotorTorqueCtrl(m1_ctrl, m1_target_rpm, motor1.shaft_velocity, true);
    resetMotorTorqueCtrl(m2_ctrl, m2_target_rpm, motor2.shaft_velocity, true);

    iwdgKick();
    if (foc_timer) foc_timer->pause();
    __disable_irq();

    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t pageError = 0;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.Page = page_idx;
    EraseInitStruct.NbPages = 1;

    iwdgKick();
    status = HAL_FLASHEx_Erase(&EraseInitStruct, &pageError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        isr_last_c0 = 0;
        __enable_irq();
        if (foc_timer) foc_timer->resume();
        Serial.print("[FLASH_ERR]:ERASE:"); Serial.println(page_idx);
        return false;
    }

    // Program in 64-bit double words
    size_t total_bytes = (sizeof(PersistentFOCConfig) + 7) & ~7;
    const uint64_t *src = (const uint64_t *)&cfg;
    for (size_t offset = 0; offset < total_bytes; offset += 8) {
        iwdgKick();
        uint64_t dword = *src++;
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, page_addr + offset, dword);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            isr_last_c0 = 0;
            __enable_irq();
            if (foc_timer) foc_timer->resume();
            Serial.print("[FLASH_ERR]:PROG:"); Serial.println((unsigned int)offset);
            return false;
        }
    }

    HAL_FLASH_Lock();
    isr_last_c0 = 0;
    __enable_irq();
    if (foc_timer) foc_timer->resume();
    iwdgKick();

    // Verify written data
    if (memcmp((const void *)page_addr, &cfg, sizeof(PersistentFOCConfig)) != 0) {
        Serial.println("[FLASH_ERR]:VERIFY");
        return false;
    }

    Serial.print("[FLASH]:SAVED:PAGE="); Serial.println(page_idx);
    return true;
}

bool loadConfigFromFlash() {
    uint32_t page_idx = 63;
    uint32_t page_addr = getFlashConfigPageAddress(&page_idx);
    const PersistentFOCConfig *ptr = (const PersistentFOCConfig *)page_addr;

    if (ptr->magic != FOC_CONFIG_MAGIC) {
        Serial.println("[FLASH_ERR]:NO_SAVED_PROFILE");
        return false;
    }
    if (computeConfigChecksum(*ptr) != ptr->checksum) {
        Serial.println("[FLASH_ERR]:CHECKSUM_MISMATCH");
        return false;
    }

    motor1.zero_electric_angle = ptr->m1_zero_angle;
    m1_base_zero_angle = ptr->m1_zero_angle;
    motor1.sensor_direction = (ptr->m1_direction == 1) ? Direction::CW : Direction::CCW;
    m1_lag_comp_us = ptr->m1_lag_comp_us;
    if (ptr->m1_kv_rating > 10.0f) motor1.KV_rating = ptr->m1_kv_rating;
    m1_ctrl.updateKe(motor1.KV_rating);
    if (ptr->m1_pid_p > 0.0f) motor1.PID_velocity.P = ptr->m1_pid_p;
    if (ptr->m1_pid_i > 0.0f) motor1.PID_velocity.I = ptr->m1_pid_i;
    motor1.PID_velocity.D = ptr->m1_pid_d;
    if (ptr->m1_lpf_tf > 0.0f) motor1.LPF_velocity.Tf = ptr->m1_lpf_tf;
    if (ptr->m1_velocity_limit > 0.0f) motor1.velocity_limit = ptr->m1_velocity_limit;
    if (ptr->m1_dwt_ms >= 0.2f && ptr->m1_dwt_ms <= 6.0f) encoder1.setDWTWindowMs(ptr->m1_dwt_ms);
    encoder1.is_lut_active = ptr->m1_lut_active;
    for (int i = 0; i < 128; i++) {
        encoder1.lut_errors[i] = ptr->m1_lut_errors[i];
    }
    m1_calibrated = (ptr->m1_zero_angle != -12345.0f && ptr->m1_zero_angle != 0.0f);

    motor2.zero_electric_angle = ptr->m2_zero_angle;
    m2_base_zero_angle = ptr->m2_zero_angle;
    motor2.sensor_direction = (ptr->m2_direction == 1) ? Direction::CW : Direction::CCW;
    m2_lag_comp_us = ptr->m2_lag_comp_us;
    if (ptr->m2_kv_rating > 10.0f) motor2.KV_rating = ptr->m2_kv_rating;
    m2_ctrl.updateKe(motor2.KV_rating);
    if (ptr->m2_pid_p > 0.0f) motor2.PID_velocity.P = ptr->m2_pid_p;
    if (ptr->m2_pid_i > 0.0f) motor2.PID_velocity.I = ptr->m2_pid_i;
    motor2.PID_velocity.D = ptr->m2_pid_d;
    if (ptr->m2_lpf_tf > 0.0f) motor2.LPF_velocity.Tf = ptr->m2_lpf_tf;
    if (ptr->m2_velocity_limit > 0.0f) motor2.velocity_limit = ptr->m2_velocity_limit;
    if (ptr->m2_dwt_ms >= 0.2f && ptr->m2_dwt_ms <= 6.0f) encoder2.setDWTWindowMs(ptr->m2_dwt_ms);
    encoder2.is_lut_active = ptr->m2_lut_active;
    for (int i = 0; i < 128; i++) {
        encoder2.lut_errors[i] = ptr->m2_lut_errors[i];
    }
    m2_calibrated = (ptr->m2_zero_angle != -12345.0f && ptr->m2_zero_angle != 0.0f);

    Serial.print("[FLASH]:LOADED:PAGE="); Serial.println(page_idx);
    return true;
}

// ----------------------------------------------------------------------------
// JSON Configuration Exporter (Capturable by Python GUI)
// ----------------------------------------------------------------------------
void dumpConfigJSON() {
    Serial.println("\n[CONFIG_START]");
    Serial.println("{");
    // Motor 1 Configuration & 128-pt LUT
    Serial.print("  \"zero_electric_angle\": "); Serial.print(motor1.zero_electric_angle, 6); Serial.println(",");
    Serial.print("  \"sensor_direction\": "); Serial.print((motor1.sensor_direction == Direction::CW) ? 1 : -1); Serial.println(",");
    Serial.print("  \"sensor_lag_comp_us\": "); Serial.print(m1_lag_comp_us, 1); Serial.println(",");
    Serial.print("  \"motor_kv_rating\": "); Serial.print(motor1.KV_rating, 2); Serial.println(",");
    Serial.print("  \"is_lut_calibrated\": "); Serial.println(encoder1.is_lut_active ? "true," : "false,");
    Serial.print("  \"pid_p\": "); Serial.print(motor1.PID_velocity.P, 4); Serial.println(",");
    Serial.print("  \"pid_i\": "); Serial.print(motor1.PID_velocity.I, 4); Serial.println(",");
    Serial.print("  \"pid_d\": "); Serial.print(motor1.PID_velocity.D, 4); Serial.println(",");
    Serial.print("  \"lpf_tf\": "); Serial.print(motor1.LPF_velocity.Tf, 6); Serial.println(",");
    Serial.print("  \"dwt_window_ms\": "); Serial.print(encoder1.window_ms, 2); Serial.println(",");
    Serial.print("  \"lut_128\": [");
    for (int i = 0; i < 128; i++) {
        Serial.print(encoder1.lut_errors[i], 6);
        if (i < 127) Serial.print(", ");
    }
    Serial.println("],");

    // Motor 2 Configuration & 128-pt LUT
    Serial.print("  \"m2_zero_electric_angle\": "); Serial.print(motor2.zero_electric_angle, 6); Serial.println(",");
    Serial.print("  \"m2_sensor_direction\": "); Serial.print((motor2.sensor_direction == Direction::CW) ? 1 : -1); Serial.println(",");
    Serial.print("  \"m2_sensor_lag_comp_us\": "); Serial.print(m2_lag_comp_us, 1); Serial.println(",");
    Serial.print("  \"m2_motor_kv_rating\": "); Serial.print(motor2.KV_rating, 2); Serial.println(",");
    Serial.print("  \"m2_is_lut_calibrated\": "); Serial.println(encoder2.is_lut_active ? "true," : "false,");
    Serial.print("  \"m2_pid_p\": "); Serial.print(motor2.PID_velocity.P, 4); Serial.println(",");
    Serial.print("  \"m2_pid_i\": "); Serial.print(motor2.PID_velocity.I, 4); Serial.println(",");
    Serial.print("  \"m2_pid_d\": "); Serial.print(motor2.PID_velocity.D, 4); Serial.println(",");
    Serial.print("  \"m2_lpf_tf\": "); Serial.print(motor2.LPF_velocity.Tf, 6); Serial.println(",");
    Serial.print("  \"m2_dwt_window_ms\": "); Serial.print(encoder2.window_ms, 2); Serial.println(",");
    Serial.print("  \"m2_lut_128\": [");
    for (int i = 0; i < 128; i++) {
        Serial.print(encoder2.lut_errors[i], 6);
        if (i < 127) Serial.print(", ");
    }
    Serial.println("]");
    Serial.println("}");
    Serial.println("[CONFIG_END]\n");
}

// ----------------------------------------------------------------------------
// Stepped 128-Point Calibration Sweep (Single Pass - Streamed to GUI)
// ----------------------------------------------------------------------------
bool runSteppedLUTSweepSinglePass(int motor_id, float align_voltage, bool *out_stalled = nullptr) {
    if (out_stalled) *out_stalled = false;
    float hold_voltage = fminf(align_voltage, CALIB_HOLD_VOLTAGE);
    Serial.print("[CALIB_LUT_START]:M"); Serial.print(motor_id);
    Serial.print(":V="); Serial.println(align_voltage, 2);

    BLDCMotor &motor = (motor_id == 1) ? motor1 : motor2;
    BLDCDriver3PWM &driver = (motor_id == 1) ? driver1 : driver2;
    MT6816Sensor &encoder = (motor_id == 1) ? encoder1 : encoder2;

    encoder.is_lut_active = false;
    driver.enable();

    // 1. Pre-flight MT6816 Health Check
    Serial.print("1. Checking MT6816 Sensor Health... ");
    if (safeDelayWithAbort(50)) return false;
    if (encoder.last_no_mag) {
        Serial.println("FAILED!");
        Serial.print("[ERR]:NO_MAG:M"); Serial.println(motor_id);
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        return false;
    }
    if (!encoder.last_parity_ok || encoder.parity_errors > 200) {
        Serial.println("FAILED!");
        Serial.print("[ERR]:PARITY:M"); Serial.print(motor_id);
        Serial.print(":"); Serial.println(encoder.parity_errors);
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        return false;
    }
    Serial.println("OK");

    // 2. Commutation Direction & Rotor Stall Verification
    Serial.print("2. Determining Sensor Direction & Motion Check... ");
    motor.setPhaseVoltage(align_voltage, 0.0f, _3PI_2);
    if (safeDelayWithAbort(100)) return false;
    motor.setPhaseVoltage(hold_voltage, 0.0f, _3PI_2);
    if (safeDelayWithAbort(300)) return false;

    float start_raw = encoder.getRawMechanicalAngle();

    const int DIR_STEPS = 200;
    for (int step = 0; step <= DIR_STEPS; step++) {
        if (checkCalibrationAbort()) {
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            return false;
        }
        float el_ang = _3PI_2 + 2.0f * _2PI * ((float)step / (float)DIR_STEPS);
        motor.setPhaseVoltage(align_voltage, 0.0f, el_ang);
        delayMicroseconds(2000);
    }
    if (safeDelayWithAbort(150)) return false;
    float mid_raw = encoder.getRawMechanicalAngle();

    for (int step = DIR_STEPS; step >= 0; step--) {
        if (checkCalibrationAbort()) {
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            return false;
        }
        float el_ang = _3PI_2 + 2.0f * _2PI * ((float)step / (float)DIR_STEPS);
        motor.setPhaseVoltage(align_voltage, 0.0f, el_ang);
        delayMicroseconds(2000);
    }
    if (safeDelayWithAbort(150)) return false;

    float d_ang = wrapAngleRad(mid_raw - start_raw);
    float moved_deg = fabsf(d_ang) * 57.2957795f;
    float expected_deg = (360.0f / (float)MOTOR_POLE_PAIRS) * 0.40f;

    if (fabsf(d_ang) < 0.05f) {
        if (out_stalled) *out_stalled = true;
        Serial.println("FAILED!");
        Serial.print("[ERR]:MOTION_CHECK:M"); Serial.print(motor_id);
        Serial.print(":moved="); Serial.print(moved_deg, 2);
        Serial.print(":expected="); Serial.println(expected_deg, 2);
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        return false;
    }

    Direction sensor_dir = (d_ang > 0.0f) ? Direction::CW : Direction::CCW;
    int dir_sign = (sensor_dir == Direction::CW) ? 1 : -1;
    Serial.println("OK");
    Serial.print("[CALIB_DIR]:M"); Serial.print(motor_id);
    Serial.print(":"); Serial.println(sensor_dir == Direction::CW ? "CW" : "CCW");

    // 3. Bidirectional 128-Step Sweep Across 360 Mechanical Degrees
    const int TOTAL_STEPS = CALIB_TOTAL_STEPS;
    const float step_mech_rad = _2PI / (float)TOTAL_STEPS;

    static float fwd_measured[CALIB_TOTAL_STEPS];
    static float rev_measured[CALIB_TOTAL_STEPS];
    static float raw_errors[CALIB_TOTAL_STEPS];
    static float mid_angles[CALIB_TOTAL_STEPS];
    static float temp_errors[CALIB_TOTAL_STEPS];

    // --- PASS 1: FORWARD (CW) SWEEP ---
    int fwd_stall_consecutive = 0;
    for (int i = 0; i < TOTAL_STEPS; i++) {
        if (checkCalibrationAbort()) {
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            return false;
        }

        float target_mech = (float)i * step_mech_rad;
        float target_elec = _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * target_mech;

        // Smooth micro-stepping transition (16 micro-steps)
        if (i > 0) {
            const int INTERP = 16;
            float prev_mech = (float)(i - 1) * step_mech_rad;
            float prev_elec = _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * prev_mech;
            for (int s = 1; s <= INTERP; s++) {
                if (checkCalibrationAbort()) {
                    motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
                    return false;
                }
                float el = prev_elec + (target_elec - prev_elec) * ((float)s / (float)INTERP);
                motor.setPhaseVoltage(align_voltage, 0.0f, el);
                delayMicroseconds(1000);
            }
        } else {
            motor.setPhaseVoltage(align_voltage, 0.0f, target_elec);
            if (safeDelayWithAbort(250)) return false;
        }

        motor.setPhaseVoltage(align_voltage, 0.0f, target_elec);
        delayMicroseconds(8000); // Rotor settle time

        // Decaying micro-dither stiction cancellation (8 steps)
        for (int d = 8; d >= 1; d--) {
            float dither = 0.08f * ((float)d / 8.0f);
            motor.setPhaseVoltage(align_voltage, 0.0f, target_elec + dither);
            delayMicroseconds(500);
            motor.setPhaseVoltage(align_voltage, 0.0f, target_elec - dither);
            delayMicroseconds(500);
        }

        motor.setPhaseVoltage(hold_voltage, 0.0f, target_elec);
        if (safeDelayWithAbort(CALIB_SWEEP_DAMP_SETTLE_MS)) return false;

        // Oversampled vector averaging (96 samples)
        float sum_sin = 0.0f, sum_cos = 0.0f;
        for (int s = 0; s < CALIB_SWEEP_OVERSAMPLE_COUNT; s++) {
            float a = encoder.getRawMechanicalAngle();
            sum_sin += sinf(a);
            sum_cos += cosf(a);
            delayMicroseconds(100);
        }
        fwd_measured[i] = normalizeAngleRad(atan2f(sum_sin, sum_cos));

        // Real-Time Stall Detection & Stiction Kick
        if (i > 0) {
            float expected_mech = normalizeAngleRad(fwd_measured[0] + (float)dir_sign * (float)i * step_mech_rad);
            float tracking_lag = fabsf(wrapAngleRad(fwd_measured[i] - expected_mech));
            float step_prog = wrapAngleRad(fwd_measured[i] - fwd_measured[i - 1]) * (float)dir_sign;

            if (step_prog < 0.008f) {
                fwd_stall_consecutive++;
                if (fwd_stall_consecutive >= 3 && fwd_stall_consecutive < 6) {
                    for (int k = 0; k < 2; k++) {
                        motor.setPhaseVoltage(align_voltage, 0.0f, target_elec + 0.10f);
                        if (safeDelayWithAbort(15)) return false;
                        motor.setPhaseVoltage(align_voltage, 0.0f, target_elec - 0.10f);
                        if (safeDelayWithAbort(15)) return false;
                    }
                    motor.setPhaseVoltage(hold_voltage, 0.0f, target_elec);
                    if (safeDelayWithAbort(20)) return false;

                    float r_sin = 0.0f, r_cos = 0.0f;
                    for (int s = 0; s < CALIB_SWEEP_OVERSAMPLE_COUNT; s++) {
                        float a = encoder.getRawMechanicalAngle();
                        r_sin += sinf(a);
                        r_cos += cosf(a);
                        delayMicroseconds(100);
                    }
                    fwd_measured[i] = normalizeAngleRad(atan2f(r_sin, r_cos));
                }
            } else {
                fwd_stall_consecutive = 0;
            }

            if ((tracking_lag > 0.18f && step_prog < 0.015f) || (fwd_stall_consecutive >= 6)) {
                if (out_stalled) *out_stalled = true;
                motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
                Serial.print("[CALIB_FAILED]:ROTOR_STALL_FWD:step="); Serial.println(i + 1);
                return false;
            }
        }

        live_calib_progress = (uint8_t)((i * 45) / TOTAL_STEPS);
        if ((i + 1) % 8 == 0 || i == TOTAL_STEPS - 1) {
            Serial.print("[CALIB_PROG]:"); Serial.print(live_calib_progress); Serial.println("%");
            sendESP32Feedback();
        }
    }

    // --- THERMAL COOL-DOWN PAUSE (5 seconds) ---
    motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
    for (int sec = CALIB_COOLDOWN_PAUSE_SEC; sec > 0; sec--) {
        Serial.print("Coil cool-down: "); Serial.print(sec); Serial.println("s remaining...");
        if (safeDelayWithAbort(1000)) return false;
    }

    // Re-engage at end of travel before reverse sweep
    float rev_start_mech = (float)(TOTAL_STEPS - 1) * step_mech_rad;
    float rev_start_elec = _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * rev_start_mech;
    motor.setPhaseVoltage(align_voltage, 0.0f, rev_start_elec);
    if (safeDelayWithAbort(100)) return false;
    motor.setPhaseVoltage(hold_voltage, 0.0f, rev_start_elec);
    if (safeDelayWithAbort(250)) return false;

    // --- PASS 2: REVERSE (CCW) SWEEP ---
    int rev_stall_consecutive = 0;
    for (int i = TOTAL_STEPS - 1; i >= 0; i--) {
        if (checkCalibrationAbort()) {
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            return false;
        }

        float target_mech = (float)i * step_mech_rad;
        float target_elec = _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * target_mech;

        if (i < TOTAL_STEPS - 1) {
            const int INTERP = 16;
            float prev_mech = (float)(i + 1) * step_mech_rad;
            float prev_elec = _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * prev_mech;
            for (int s = 1; s <= INTERP; s++) {
                if (checkCalibrationAbort()) {
                    motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
                    return false;
                }
                float el = prev_elec + (target_elec - prev_elec) * ((float)s / (float)INTERP);
                motor.setPhaseVoltage(align_voltage, 0.0f, el);
                delayMicroseconds(1000);
            }
        }

        motor.setPhaseVoltage(align_voltage, 0.0f, target_elec);
        delayMicroseconds(8000);

        // Decaying micro-dither (8 steps)
        for (int d = 8; d >= 1; d--) {
            float dither = 0.08f * ((float)d / 8.0f);
            motor.setPhaseVoltage(align_voltage, 0.0f, target_elec + dither);
            delayMicroseconds(500);
            motor.setPhaseVoltage(align_voltage, 0.0f, target_elec - dither);
            delayMicroseconds(500);
        }

        motor.setPhaseVoltage(hold_voltage, 0.0f, target_elec);
        if (safeDelayWithAbort(CALIB_SWEEP_DAMP_SETTLE_MS)) return false;

        float sum_sin = 0.0f, sum_cos = 0.0f;
        for (int s = 0; s < CALIB_SWEEP_OVERSAMPLE_COUNT; s++) {
            float a = encoder.getRawMechanicalAngle();
            sum_sin += sinf(a);
            sum_cos += cosf(a);
            delayMicroseconds(100);
        }
        rev_measured[i] = normalizeAngleRad(atan2f(sum_sin, sum_cos));

        if (i < TOTAL_STEPS - 1) {
            int step_idx_rev = (TOTAL_STEPS - 1 - i);
            float expected_rev = normalizeAngleRad(rev_measured[TOTAL_STEPS - 1] - (float)dir_sign * (float)step_idx_rev * step_mech_rad);
            float tracking_lag = fabsf(wrapAngleRad(rev_measured[i] - expected_rev));
            float step_prog = wrapAngleRad(rev_measured[i + 1] - rev_measured[i]) * (float)dir_sign;

            if (step_prog < 0.008f) {
                rev_stall_consecutive++;
                if (rev_stall_consecutive >= 3 && rev_stall_consecutive < 6) {
                    for (int k = 0; k < 2; k++) {
                        motor.setPhaseVoltage(align_voltage, 0.0f, target_elec + 0.10f);
                        if (safeDelayWithAbort(15)) return false;
                        motor.setPhaseVoltage(align_voltage, 0.0f, target_elec - 0.10f);
                        if (safeDelayWithAbort(15)) return false;
                    }
                    motor.setPhaseVoltage(hold_voltage, 0.0f, target_elec);
                    if (safeDelayWithAbort(20)) return false;

                    float r_sin = 0.0f, r_cos = 0.0f;
                    for (int s = 0; s < CALIB_SWEEP_OVERSAMPLE_COUNT; s++) {
                        float a = encoder.getRawMechanicalAngle();
                        r_sin += sinf(a);
                        r_cos += cosf(a);
                        delayMicroseconds(100);
                    }
                    rev_measured[i] = normalizeAngleRad(atan2f(r_sin, r_cos));
                }
            } else {
                rev_stall_consecutive = 0;
            }

            if ((tracking_lag > 0.18f && step_prog < 0.015f) || (rev_stall_consecutive >= 6)) {
                if (out_stalled) *out_stalled = true;
                motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
                Serial.print("[CALIB_FAILED]:ROTOR_STALL_REV:step="); Serial.println(i + 1);
                return false;
            }
        }

        live_calib_progress = 45 + (uint8_t)(((TOTAL_STEPS - 1 - i) * 45) / TOTAL_STEPS);
        if (i % 8 == 0 || i == 0) {
            Serial.print("[CALIB_PROG]:"); Serial.print(live_calib_progress); Serial.println("%");
            sendESP32Feedback();
        }
    }

    // 4. Midpoint Synthesis & Residual Calculation
    float start_mid = normalizeAngleRad(atan2f(sinf(fwd_measured[0]) + sinf(rev_measured[0]),
                                               cosf(fwd_measured[0]) + cosf(rev_measured[0])));

    for (int i = 0; i < TOTAL_STEPS; i++) {
        float mid_meas = normalizeAngleRad(atan2f(sinf(fwd_measured[i]) + sinf(rev_measured[i]),
                                                   cosf(fwd_measured[i]) + cosf(rev_measured[i])));
        float expected_meas = normalizeAngleRad(start_mid + (float)dir_sign * ((float)i * step_mech_rad));
        raw_errors[i] = wrapAngleRad(mid_meas - expected_meas);
        mid_angles[i] = mid_meas;
    }

    // Stream 128 midpoints in 8 compact lines for GUI Fourier harmonic fitting
    Serial.print("[SWEEP_START]:M"); Serial.println(motor_id);
    for (int chunk = 0; chunk < 8; chunk++) {
        Serial.print("[SWEEP_MID]:M"); Serial.print(motor_id);
        Serial.print(":"); Serial.print(chunk); Serial.print(":");
        for (int j = 0; j < 16; j++) {
            Serial.print(mid_angles[chunk * 16 + j], 5);
            if (j < 15) Serial.print(",");
        }
        Serial.println();
    }

    // 5. 3-Point Circular Median Filter & On-Chip Fourier Decomposition
    int anomaly_count = 0;
    for (int i = 0; i < TOTAL_STEPS; i++) {
        int prev_i = (i - 1 + TOTAL_STEPS) % TOTAL_STEPS;
        int next_i = (i + 1) % TOTAL_STEPS;
        float a = raw_errors[prev_i], b = raw_errors[i], c = raw_errors[next_i];
        float med = ((a >= b && a <= c) || (a <= b && a >= c)) ? a :
                    ((b >= a && b <= c) || (b <= a && b >= c)) ? b : c;
        temp_errors[i] = (fabsf(b - med) > 0.08f) ? med : b;
        if (fabsf(temp_errors[i]) > LUT_ANOMALY_THRESHOLD_RAD) anomaly_count++;
    }

    float sum_err = 0.0f;
    for (int i = 0; i < TOTAL_STEPS; i++) sum_err += temp_errors[i];
    float mean_err = sum_err / (float)TOTAL_STEPS;

    float f_a1 = 0.0f, f_b1 = 0.0f, f_a2 = 0.0f, f_b2 = 0.0f, f_a4 = 0.0f, f_b4 = 0.0f;
    for (int i = 0; i < TOTAL_STEPS; i++) {
        float err_c = temp_errors[i] - mean_err;
        float theta = mid_angles[i];
        f_a1 += err_c * cosf(theta);
        f_b1 += err_c * sinf(theta);
        f_a2 += err_c * cosf(2.0f * theta);
        f_b2 += err_c * sinf(2.0f * theta);
        f_a4 += err_c * cosf(4.0f * theta);
        f_b4 += err_c * sinf(4.0f * theta);
    }

    float norm = 2.0f / (float)TOTAL_STEPS;
    float a1 = f_a1 * norm, b1 = f_b1 * norm;
    float a2 = f_a2 * norm, b2 = f_b2 * norm;
    float a4 = f_a4 * norm, b4 = f_b4 * norm;

    float synth_max = -100.0f, synth_min = 100.0f;
    for (int s = 0; s < TOTAL_STEPS; s++) {
        float phi = (float)s * (_2PI / (float)TOTAL_STEPS);
        float synth = (a1 * cosf(phi) + b1 * sinf(phi)) +
                      (a2 * cosf(2.0f * phi) + b2 * sinf(2.0f * phi)) +
                      (a4 * cosf(4.0f * phi) + b4 * sinf(4.0f * phi));
        encoder.lut_errors[s] = synth;
        if (synth > synth_max) synth_max = synth;
        if (synth < synth_min) synth_min = synth;
    }

    float synth_pk_pk_rad = synth_max - synth_min;
    bool lut_valid = (anomaly_count <= 2) && (synth_pk_pk_rad <= LUT_PKPK_LIMIT_RAD);
    if (!lut_valid) {
        encoder.is_lut_active = false;
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        Serial.println("[CALIB_FAILED]:LUT_GEOMETRIC_LIMIT");
        return false;
    }

    encoder.is_lut_active = true;
    Serial.print("[SWEEP_DONE]:M"); Serial.println(motor_id);
    Serial.print("[LUT_READY]:M"); Serial.println(motor_id);
    return true;
}

// ----------------------------------------------------------------------------
// Multi-Pole Comprehensive Electrical Zero Alignment (Matching ESP32 Suite)
// ----------------------------------------------------------------------------
bool runComprehensiveElectricalAlignment(int motor_id, float align_voltage, bool *out_stalled = nullptr) {
    CalibrationIsolationGuard isolation_guard(motor_id);
    if (out_stalled) *out_stalled = false;
    float hold_voltage = fminf(align_voltage * 0.85f, 3.40f);
    BLDCMotor &motor = (motor_id == 1) ? motor1 : motor2;
    BLDCDriver3PWM &driver = (motor_id == 1) ? driver1 : driver2;
    MT6816Sensor &encoder = (motor_id == 1) ? encoder1 : encoder2;

    Serial.print("[CALIB_ELEC_START]:M"); Serial.print(motor_id);
    Serial.print(":V="); Serial.print(align_voltage, 2);
    Serial.print(":LUT="); Serial.println(encoder.is_lut_active ? "128PT" : "RAW");

    driver.enable();

    // 1. Determining Sensor Commutation Direction & Verifying Motion
    motor.setPhaseVoltage(align_voltage, 0.0f, _3PI_2);
    if (safeDelayWithAbort(400)) return false;
    float start_angle = encoder.getCalibratedAngleRad();

    const int DIR_STEPS = 400;
    for (int step = 0; step <= DIR_STEPS; step++) {
        if (checkCalibrationAbort()) {
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            return false;
        }
        float el_ang = _3PI_2 + 2.0f * _2PI * ((float)step / (float)DIR_STEPS);
        motor.setPhaseVoltage(align_voltage, 0.0f, el_ang);
        delayMicroseconds(2500);
    }
    if (safeDelayWithAbort(200)) return false;
    float mid_angle = encoder.getCalibratedAngleRad();

    for (int step = DIR_STEPS; step >= 0; step--) {
        if (checkCalibrationAbort()) {
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            return false;
        }
        float el_ang = _3PI_2 + 2.0f * _2PI * ((float)step / (float)DIR_STEPS);
        motor.setPhaseVoltage(align_voltage, 0.0f, el_ang);
        delayMicroseconds(2500);
    }
    if (safeDelayWithAbort(200)) return false;

    float d_ang_fwd = wrapAngleRad(mid_angle - start_angle);
    float moved_rad = fabsf(d_ang_fwd);
    float min_expected_rad = (2.0f * _2PI / (float)MOTOR_POLE_PAIRS) * 0.40f;

    if (moved_rad < min_expected_rad) {
        if (out_stalled) *out_stalled = true;
        Serial.print("[ERR]:MOTION_CHECK:M"); Serial.print(motor_id);
        Serial.print(":moved="); Serial.print(moved_rad * 57.2957795f, 2);
        Serial.print(":expected="); Serial.println(min_expected_rad * 57.2957795f, 2);
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        return false;
    }

    Direction sensor_dir = (d_ang_fwd > 0.0f) ? Direction::CW : Direction::CCW;
    motor.sensor_direction = sensor_dir;
    Serial.print("[CALIB_DIR]:M"); Serial.print(motor_id);
    Serial.print(":"); Serial.println(sensor_dir == Direction::CW ? "CW" : "CCW");

    // 2. Multi-Pole Full 360° Bidirectional Sweep
    const int num_poles = MOTOR_POLE_PAIRS;
    static float fwd_zeros[MOTOR_POLE_PAIRS];
    static float rev_zeros[MOTOR_POLE_PAIRS];
    static float fwd_mechs[MOTOR_POLE_PAIRS];
    static float rev_mechs[MOTOR_POLE_PAIRS];
    static float pole_zeros[MOTOR_POLE_PAIRS];
    bool stall_detected = false;
    int stall_count = 0;

    // PASS 1: FORWARD (CW) 360° SWEEP (Poles 0 -> 10)
    for (int p = 0; p < num_poles; p++) {
        if (checkCalibrationAbort()) {
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            return false;
        }

        float target_el = _3PI_2 + (float)p * _2PI;
        if (p > 0) {
            const int INTERP_STEPS = 120;
            float prev_el = _3PI_2 + (float)(p - 1) * _2PI;
            for (int s = 1; s <= INTERP_STEPS; s++) {
                if (checkCalibrationAbort()) {
                    motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
                    return false;
                }
                float el = prev_el + (target_el - prev_el) * ((float)s / (float)INTERP_STEPS);
                motor.setPhaseVoltage(align_voltage, 0.0f, el);
                delayMicroseconds(2500);
            }
        } else {
            motor.setPhaseVoltage(align_voltage, 0.0f, target_el);
            if (safeDelayWithAbort(300)) return false;
        }

        // 3-cycle decaying 30 Hz mechanical ring-down dither
        const float dither_amps[3] = {0.08f, 0.04f, 0.015f};
        for (int c = 0; c < 3; c++) {
            motor.setPhaseVoltage(align_voltage, 0.0f, target_el + dither_amps[c]);
            if (safeDelayWithAbort(15)) return false;
            motor.setPhaseVoltage(align_voltage, 0.0f, target_el - dither_amps[c]);
            if (safeDelayWithAbort(15)) return false;
        }
        motor.setPhaseVoltage(hold_voltage, 0.0f, target_el);
        if (safeDelayWithAbort(60)) return false;

        float mech = 0.0f;
        for (int s = 0; s < CALIB_ALIGN_OVERSAMPLE_COUNT; s++) {
            mech += encoder.getCalibratedAngleRad();
            delayMicroseconds(200);
        }
        mech /= (float)CALIB_ALIGN_OVERSAMPLE_COUNT;
        fwd_mechs[p] = mech;
        fwd_zeros[p] = normalizeAngleRad((float)((int)sensor_dir * num_poles) * mech);

        if (p > 0) {
            float step_deg = wrapAngleRad(fwd_mechs[p] - fwd_mechs[p - 1]) * (sensor_dir == Direction::CW ? 1.0f : -1.0f) * 57.2957795f;
            if (step_deg < 10.0f) {
                stall_detected = true;
                stall_count++;
            } else {
                stall_count = 0;
            }
        }
        if (stall_count >= 3) {
            if (out_stalled) *out_stalled = true;
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            Serial.print("[CALIB_FAILED]:ALIGN_STALL_CW:pole="); Serial.println(p + 1);
            return false;
        }
        sendESP32Feedback();
    }

    // PASS 2: REVERSE (CCW) 360° RETURN SWEEP (Poles 10 -> 0)
    stall_count = 0;
    for (int p = num_poles - 1; p >= 0; p--) {
        if (checkCalibrationAbort()) {
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            return false;
        }

        float target_el = _3PI_2 + (float)p * _2PI;
        if (p < num_poles - 1) {
            const int INTERP_STEPS = 120;
            float prev_el = _3PI_2 + (float)(p + 1) * _2PI;
            for (int s = 1; s <= INTERP_STEPS; s++) {
                if (checkCalibrationAbort()) {
                    motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
                    return false;
                }
                float el = prev_el + (target_el - prev_el) * ((float)s / (float)INTERP_STEPS);
                motor.setPhaseVoltage(align_voltage, 0.0f, el);
                delayMicroseconds(2500);
            }
        } else {
            motor.setPhaseVoltage(align_voltage, 0.0f, target_el);
            if (safeDelayWithAbort(150)) return false;
        }

        const float dither_amps[3] = {0.08f, 0.04f, 0.015f};
        for (int c = 0; c < 3; c++) {
            motor.setPhaseVoltage(align_voltage, 0.0f, target_el + dither_amps[c]);
            if (safeDelayWithAbort(15)) return false;
            motor.setPhaseVoltage(align_voltage, 0.0f, target_el - dither_amps[c]);
            if (safeDelayWithAbort(15)) return false;
        }
        motor.setPhaseVoltage(hold_voltage, 0.0f, target_el);
        if (safeDelayWithAbort(60)) return false;

        float mech = 0.0f;
        for (int s = 0; s < CALIB_ALIGN_OVERSAMPLE_COUNT; s++) {
            mech += encoder.getCalibratedAngleRad();
            delayMicroseconds(200);
        }
        mech /= (float)CALIB_ALIGN_OVERSAMPLE_COUNT;
        rev_mechs[p] = mech;
        rev_zeros[p] = normalizeAngleRad((float)((int)sensor_dir * num_poles) * mech);

        if (p < num_poles - 1) {
            float rev_step_deg = wrapAngleRad(rev_mechs[p] - rev_mechs[p + 1]) * (sensor_dir == Direction::CW ? -1.0f : 1.0f) * 57.2957795f;
            if (rev_step_deg < 10.0f) {
                stall_detected = true;
                stall_count++;
            } else {
                stall_count = 0;
            }
        }
        if (stall_count >= 3) {
            if (out_stalled) *out_stalled = true;
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            Serial.print("[CALIB_FAILED]:ALIGN_STALL_CCW:pole="); Serial.println(p + 1);
            return false;
        }
        sendESP32Feedback();
    }

    // Cumulative Motion Check
    float total_fwd_travel_deg = 0.0f;
    for (int p = 1; p < num_poles; p++) {
        total_fwd_travel_deg += wrapAngleRad(fwd_mechs[p] - fwd_mechs[p - 1]) * (sensor_dir == Direction::CW ? 1.0f : -1.0f) * 57.2957795f;
    }
    bool rotation_ok = (total_fwd_travel_deg >= 250.0f) && (stall_count == 0);

    float max_hyst_deg = 0.0f;
    float sum_sin = 0.0f, sum_cos = 0.0f;

    for (int p = 0; p < num_poles; p++) {
        float fwd_z = fwd_zeros[p];
        float rev_z = rev_zeros[p];
        float true_z = normalizeAngleRad(atan2f(sinf(fwd_z) + sinf(rev_z), cosf(fwd_z) + cosf(rev_z)));
        pole_zeros[p] = true_z;

        sum_sin += sinf(true_z);
        sum_cos += cosf(true_z);

        float hyst_deg = fabsf(wrapAngleRad(fwd_z - rev_z)) * 57.2957795f;
        if (hyst_deg > max_hyst_deg) max_hyst_deg = hyst_deg;
    }

    float mean_zero_angle = normalizeAngleRad(atan2f(sum_sin / (float)num_poles, sum_cos / (float)num_poles));

    float var_sum = 0.0f;
    for (int p = 0; p < num_poles; p++) {
        float diff = wrapAngleRad(pole_zeros[p] - mean_zero_angle);
        var_sum += diff * diff;
    }
    float std_dev_deg = sqrtf(var_sum / (float)num_poles) * 57.2957795f;

    if (stall_detected || !rotation_ok || total_fwd_travel_deg < 250.0f || std_dev_deg > 10.0f || max_hyst_deg > 10.0f) {
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        Serial.println("[CALIB_FAILED]:ROTOR_STALL_OR_MOTION_CHECK");
        return false;
    }

    const char* grade_str = "GRADE E";
    if (std_dev_deg < 2.0f && max_hyst_deg <= 2.0f) grade_str = "GRADE A";
    else if (std_dev_deg < 4.0f && max_hyst_deg <= 4.0f) grade_str = "GRADE B";
    else if (std_dev_deg < 6.0f && max_hyst_deg <= 6.0f) grade_str = "GRADE C";
    else if (std_dev_deg < 8.0f && max_hyst_deg <= 8.0f) grade_str = "GRADE D";

    motor.zero_electric_angle = mean_zero_angle;
    motor.sensor_direction = sensor_dir;
    if (motor_id == 1) {
        m1_base_zero_angle = mean_zero_angle;
        m1_calibrated = true;
    } else {
        m2_base_zero_angle = mean_zero_angle;
        m2_calibrated = true;
    }

    // Stream 11 pole raw measurements to GUI for certified table generation and statistics
    Serial.print("[ELEC_POLES]:M"); Serial.print(motor_id);
    Serial.print(":DIR="); Serial.print(sensor_dir == Direction::CW ? "CW" : "CCW");
    Serial.print(":DATA=");
    for (int p = 0; p < num_poles; p++) {
        Serial.print(fwd_mechs[p], 4); Serial.print(","); Serial.print(rev_mechs[p], 4);
        if (p < num_poles - 1) Serial.print(";");
    }
    Serial.println();

    Serial.print("[CALIB_DONE]:M"); Serial.print(motor_id);
    Serial.print(":GRADE="); Serial.print(grade_str);
    Serial.print(":ZERO="); Serial.println(mean_zero_angle, 5);
    return true;
}

// ----------------------------------------------------------------------------
// Master Auto-Calibration Orchestrator (128-pt Dual-Pass LUT + 11-Pole Zero + Dump)
// ----------------------------------------------------------------------------
void runMasterCalibrationSequence(int motor_id, float align_voltage) {
    CalibrationIsolationGuard isolation_guard(motor_id);
    BLDCMotor &motor = (motor_id == 1) ? motor1 : motor2;
    BLDCDriver3PWM &driver = (motor_id == 1) ? driver1 : driver2;
    MT6816Sensor &encoder = (motor_id == 1) ? encoder1 : encoder2;
    bool prev_enabled = (motor_id == 1) ? m1_enabled : m2_enabled;

    g_estop_active = false;
    encoder.clearLUT();

    bool calib_stalled = false;

    // Pass 1: 128-point LUT Sweep
    bool lut_ok = runSteppedLUTSweepSinglePass(motor_id, align_voltage, &calib_stalled);

    if (g_estop_active) {
        triggerGlobalEmergencyStop();
        return;
    }

    // Only retry with boost voltage if it stalled and NOT aborted by user
    if (!lut_ok && calib_stalled && align_voltage < CALIB_BOOST_VOLTAGE) {
        Serial.print("[CALIB_BOOST]:M"); Serial.print(motor_id);
        Serial.print(":V="); Serial.println(CALIB_BOOST_VOLTAGE, 2);
        if (safeDelayWithAbort(400)) {
            triggerGlobalEmergencyStop();
            return;
        }
        lut_ok = runSteppedLUTSweepSinglePass(motor_id, CALIB_BOOST_VOLTAGE, &calib_stalled);
        if (g_estop_active) {
            triggerGlobalEmergencyStop();
            return;
        }
    }

    if (!lut_ok) {
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        if (g_estop_active) {
            triggerGlobalEmergencyStop();
        } else {
            driver.disable();
            live_calib_progress = 0;
            Serial.print("[CALIB_FAILED]:M"); Serial.print(motor_id);
            Serial.println(calib_stalled ? ":STALL" : ":REJECT");
        }
        return;
    }

    // Thermal Cool-Down Pause before Electrical Alignment
    motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
    for (int sec = CALIB_COOLDOWN_PAUSE_SEC; sec > 0; sec--) {
        Serial.print("Coil cool-down: "); Serial.print(sec); Serial.println("s remaining...");
        if (safeDelayWithAbort(1000)) {
            triggerGlobalEmergencyStop();
            return;
        }
    }

    // Pass 2: 11-pole Electrical Alignment on Linearized Sensor
    bool align_stalled = false;
    bool align_ok = runComprehensiveElectricalAlignment(motor_id, align_voltage, &align_stalled);
    if (g_estop_active) {
        triggerGlobalEmergencyStop();
        return;
    }

    if (!align_ok) {
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        if (g_estop_active) {
            triggerGlobalEmergencyStop();
        } else {
            driver.disable();
            live_calib_progress = 0;
            Serial.print("[CALIB_FAILED]:M"); Serial.print(motor_id);
            Serial.println(align_stalled ? ":ALIGN_STALL" : ":ALIGN_REJECT");
        }
        return;
    }

    // Pass 3: Dump full LUT curve so GUI / Host can store it
    dumpLUT(motor_id);

    live_calib_progress = 100;
    Serial.println("[CALIB_PROG]:100%");

    motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
    motor.voltage.q = 0.0f;
    motor.voltage.d = 0.0f;
    if (!prev_enabled) {
        motor.disable();
        driver.disable();
        digitalWrite((motor_id == 1) ? M1_EN : M2_EN, LOW);
    }

    Serial.print("[CALIB_CERTIFIED]:M"); Serial.println(motor_id);
}

// ----------------------------------------------------------------------------
// 8-Line LUT Upload Parser (Safe Float Tokenizer - No _scanf_float dependency)
// ----------------------------------------------------------------------------
void parseLUTUpload(const char *cmd) {
    // Format: LUT <motor_id> <line_idx> <f0,f1,...,f15>
    const char *p = skip_ws(cmd);
    if (strncasecmp(p, "LUT", 3) != 0) return;
    p += 3;
    const char *next = nullptr;
    int m_id = fast_atoi(p, &next);
    if (next == p) return;
    p = next;
    int line_idx = fast_atoi(p, &next);
    if (next == p) return;
    p = skip_ws(next);

    if (line_idx < 0 || line_idx >= 8 || (m_id != 1 && m_id != 2)) {
        Serial.print("[ERR]:LUT_INDEX:"); Serial.print(m_id); Serial.print(":"); Serial.println(line_idx);
        return;
    }

    float vals[16];
    int count = 0;
    while (*p && count < 16) {
        vals[count++] = fast_atof(p, &next);
        if (next == p) break;
        p = skip_ws(next);
        if (*p == ',') p++;
        p = skip_ws(p);
    }

    if (count == 16) {
        if (m_id == 1) {
            encoder1.setLUTLine(line_idx, vals, 16);
            if (line_idx == 7) {
                encoder1.is_lut_active = true;
                Serial.println("[LUT_READY]:M1");
            } else {
                Serial.print("[LUT_ACK]:M1:"); Serial.println(line_idx);
            }
        } else if (m_id == 2) {
            encoder2.setLUTLine(line_idx, vals, 16);
            if (line_idx == 7) {
                encoder2.is_lut_active = true;
                Serial.println("[LUT_READY]:M2");
            } else {
                Serial.print("[LUT_ACK]:M2:"); Serial.println(line_idx);
            }
        }
    } else {
        Serial.print("[ERR]:LUT_COUNT:"); Serial.println(count);
    }
}

// ----------------------------------------------------------------------------
// Reset Parameters to GM3506 Defaults
// ----------------------------------------------------------------------------
void resetDefaults(int motor_id) {
    if (motor_id == 1 || motor_id == 0) {
        motor1.voltage_limit = MOTOR_VOLTAGE_LIMIT;
        motor1.PID_velocity.limit = motor1.voltage_limit;
        motor1.velocity_limit = MOTOR_VELOCITY_LIMIT;
        motor1.KV_rating = MOTOR_KV_RATING;
        motor1.PID_velocity.P = DEFAULT_PID_P;
        motor1.PID_velocity.I = DEFAULT_PID_I;
        motor1.PID_velocity.D = DEFAULT_PID_D;
        motor1.PID_velocity.output_ramp = DEFAULT_PID_RAMP;
        motor1.PID_velocity.Ts = 0.0002f;
        motor1.LPF_velocity.Tf = DEFAULT_LPF_TF;
        motor1.LPF_velocity.Ts = 0.0002f;
        m1_lag_comp_us = DEFAULT_LAG_COMP_US;
        m1_lag_filt_vel = 0.0f;
        m1_ctrl.i_clamp_v = DEFAULT_PID_I_CLAMP_V;
        m1_ctrl.updateKe(MOTOR_KV_RATING);
        resetMotorTorqueCtrl(m1_ctrl, m1_target_rpm, motor1.shaft_velocity, true);
        encoder1.setDWTWindowMs(DEFAULT_DWT_WINDOW_MS);
        encoder1.resetErrors();
        stall_guard1.reset();
    }
    if (motor_id == 2 || motor_id == 0) {
        motor2.voltage_limit = MOTOR_VOLTAGE_LIMIT;
        motor2.PID_velocity.limit = motor2.voltage_limit;
        motor2.velocity_limit = MOTOR_VELOCITY_LIMIT;
        motor2.KV_rating = MOTOR_KV_RATING;
        motor2.PID_velocity.P = DEFAULT_PID_P;
        motor2.PID_velocity.I = DEFAULT_PID_I;
        motor2.PID_velocity.D = DEFAULT_PID_D;
        motor2.PID_velocity.output_ramp = DEFAULT_PID_RAMP;
        motor2.PID_velocity.Ts = 0.0002f;
        motor2.LPF_velocity.Tf = DEFAULT_LPF_TF;
        motor2.LPF_velocity.Ts = 0.0002f;
        m2_lag_comp_us = DEFAULT_LAG_COMP_US;
        m2_lag_filt_vel = 0.0f;
        m2_ctrl.i_clamp_v = DEFAULT_PID_I_CLAMP_V;
        m2_ctrl.updateKe(MOTOR_KV_RATING);
        resetMotorTorqueCtrl(m2_ctrl, m2_target_rpm, motor2.shaft_velocity, true);
        encoder2.setDWTWindowMs(DEFAULT_DWT_WINDOW_MS);
        encoder2.resetErrors();
        stall_guard2.reset();
    }
    isr_jitter_spikes = 0;
    isr_alltime_peak_cycles = isr_last_cycles;
    prof_period_min = 8500;
    prof_period_max = 8500;
    prof_period_jitter_max = 0;
    Serial.println(">>> [RESET] Restored GM3506 12V Optimized Defaults & Cleared Parity Errors.");
}

// ----------------------------------------------------------------------------
// Text Help & Status Menu
// ----------------------------------------------------------------------------
void printHelp() {
    Serial.print("FOLKRACE G474 FOC [M1:");
    Serial.print(m1_enabled ? "ON" : "OFF");
    Serial.print(" M2:");
    Serial.print(m2_enabled ? "ON" : "OFF");
    Serial.println("]. Cmds: E/D[1/2], STOP, T[1/2] <rpm>, COAST[1/2], TRQ[1/2] <min> <max> [ff], ICLAMP[1/2] <V>, CALIB 1/2, SAVE, EXPORT");
}

// ----------------------------------------------------------------------------
// Command Interpreter
// ----------------------------------------------------------------------------
void startFocTrace() {
    foc_trace_active = false;
    foc_trace_write_index = 0;
    foc_trace_count = 0;
    foc_trace_overwritten = 0;
    foc_trace_active = true;
    Serial.println(">>> FOC TRACE STARTED (500 Hz, 10.24 s ring)");
}

void stopFocTrace() {
    foc_trace_active = false;
    Serial.print(">>> FOC TRACE STOPPED samples=");
    Serial.print(foc_trace_count);
    Serial.print(" overwritten=");
    Serial.println(foc_trace_overwritten);
}

void dumpFocTrace() {
    foc_trace_active = false;
    uint16_t count = foc_trace_count;
    uint16_t write_index = foc_trace_write_index;

    FocTraceHeader header = {{'F', 'T'}, 1, FOC_TRACE_SAMPLE_HZ, count, foc_trace_overwritten, 0};
    Serial.write(reinterpret_cast<const uint8_t *>(&header), sizeof(header));

    uint16_t first = (count < FOC_TRACE_CAPACITY)
        ? 0
        : (uint16_t)((write_index + FOC_TRACE_CAPACITY - count) % FOC_TRACE_CAPACITY);
    for (uint16_t offset = 0; offset < count; offset++) {
        uint16_t index = (uint16_t)((first + offset) % FOC_TRACE_CAPACITY);
        FocTraceSample sample;
        const volatile FocTraceSample &source = foc_trace[index];
        sample.m1_raw_angle = source.m1_raw_angle;
        sample.m2_raw_angle = source.m2_raw_angle;
        sample.m1_target_rpm = source.m1_target_rpm;
        sample.m2_target_rpm = source.m2_target_rpm;
        sample.m1_actual_rpm = source.m1_actual_rpm;
        sample.m2_actual_rpm = source.m2_actual_rpm;
        sample.m1_uq_mv = source.m1_uq_mv;
        sample.m2_uq_mv = source.m2_uq_mv;
        Serial.write(reinterpret_cast<const uint8_t *>(&sample), sizeof(sample));
    }
    Serial.flush();
    Serial.println(">>> FOC TRACE DUMP COMPLETE");
}

void processCommand(const char *cmd) {
    if (strlen(cmd) == 0) return;

    if (strcasecmp(cmd, "TRACE_START") == 0) {
        startFocTrace();
        return;
    }
    if (strcasecmp(cmd, "TRACE_STOP") == 0) {
        stopFocTrace();
        return;
    }
    if (strcasecmp(cmd, "TRACE_DUMP") == 0) {
        dumpFocTrace();
        return;
    }
    if (strcasecmp(cmd, "ESP_DIAG") == 0) {
        Serial.print("[ESP_DIAG]:RX="); Serial.print(esp_rx_bytes);
        Serial.print(":PKT="); Serial.print(esp_pkts_ok);
        Serial.print(":CHK_ERR="); Serial.print(esp_chk_err);
        Serial.print(":WDOG="); Serial.print(esp_wdog_trips);
        Serial.print(":MAX_GAP_MS="); Serial.print(esp_max_gap_ms);
        Serial.print(":ORE="); Serial.print(esp_usart1_ore);
        Serial.print(":FE="); Serial.print(esp_usart1_fe);
        Serial.print(":NE="); Serial.print(esp_usart1_ne);
        Serial.print(":DRV_ZERO="); Serial.print(esp_drive_zero);
        Serial.print(":LINK="); Serial.println(esp_link_active ? 1 : 0);
        esp_max_gap_ms = 0;
        return;
    }

    if (strcasecmp(cmd, "PROF") == 0) {
        float us_scale = 1.0f / (SystemCoreClock / 1000000.0f);
        Serial.print("[PROF]:SYSCLK="); Serial.print(SystemCoreClock);
        Serial.print(":PCLK1="); Serial.print(HAL_RCC_GetPCLK1Freq());
        Serial.print(":PCLK2="); Serial.print(HAL_RCC_GetPCLK2Freq());
        Serial.print(":SPI1="); Serial.print((uint32_t)SPI1->CR1, HEX); Serial.print(","); Serial.print((uint32_t)SPI1->CR2, HEX);
        Serial.print(":SPI2="); Serial.print((uint32_t)SPI2->CR1, HEX); Serial.print(","); Serial.print((uint32_t)SPI2->CR2, HEX);
        Serial.print(":TIM2="); Serial.print((uint32_t)TIM2->CCMR1, HEX); Serial.print(","); Serial.print((uint32_t)TIM2->CCMR2, HEX); Serial.print(","); Serial.print((uint32_t)TIM2->CR1, HEX);
        Serial.print(":TIM3="); Serial.print((uint32_t)TIM3->CCMR1, HEX); Serial.print(","); Serial.print((uint32_t)TIM3->CCMR2, HEX); Serial.print(","); Serial.print((uint32_t)TIM3->CR1, HEX);
        Serial.print(":SPI_LAST="); Serial.print(prof_spi_last); Serial.print(":SPI_MAX="); Serial.print(prof_spi_max);
        Serial.print(":UPD_LAST="); Serial.print(prof_update_last); Serial.print(":UPD_MAX="); Serial.print(prof_update_max);
        Serial.print(":MOVE_LAST="); Serial.print(prof_move_last); Serial.print(":MOVE_MAX="); Serial.print(prof_move_max);
        Serial.print(":M1_LAST="); Serial.print(prof_m1_foc_last); Serial.print(":M1_MAX="); Serial.print(prof_m1_foc_max);
        Serial.print(":M2_LAST="); Serial.print(prof_m2_foc_last); Serial.print(":M2_MAX="); Serial.print(prof_m2_foc_max);
        Serial.print(":TOT_LAST="); Serial.print(prof_total_last); Serial.print(":TOT_MAX="); Serial.print(prof_total_max);
        Serial.print(":ALL_PEAK="); Serial.print(isr_alltime_peak_cycles);
        Serial.print(":PER_LAST="); Serial.print(prof_period_last);
        Serial.print(":JIT_MAX="); Serial.print(prof_period_jitter_max * us_scale, 2);
        Serial.print(":PER_MIN="); Serial.print(prof_period_min * us_scale, 2);
        Serial.print(":PER_MAX="); Serial.print(prof_period_max * us_scale, 2);
        Serial.print(":SPIKES="); Serial.println((unsigned int)isr_jitter_spikes);
        Serial.print("[PROF]:SYSCLK="); Serial.print(SystemCoreClock);
        Serial.print(":PCLK1="); Serial.print(HAL_RCC_GetPCLK1Freq());
        Serial.print(":PCLK2="); Serial.print(HAL_RCC_GetPCLK2Freq());
        Serial.print(":SPI1="); Serial.print((uint32_t)SPI1->CR1, HEX); Serial.print(","); Serial.print((uint32_t)SPI1->CR2, HEX);
        Serial.print(":SPI2="); Serial.print((uint32_t)SPI2->CR1, HEX); Serial.print(","); Serial.print((uint32_t)SPI2->CR2, HEX);
        Serial.print(":TIM2="); Serial.print((uint32_t)TIM2->CCMR1, HEX); Serial.print(","); Serial.print((uint32_t)TIM2->CCMR2, HEX); Serial.print(","); Serial.print((uint32_t)TIM2->CR1, HEX);
        Serial.print(":TIM3="); Serial.print((uint32_t)TIM3->CCMR1, HEX); Serial.print(","); Serial.print((uint32_t)TIM3->CCMR2, HEX); Serial.print(","); Serial.print((uint32_t)TIM3->CR1, HEX);
        Serial.print(":SPI_LAST="); Serial.print(prof_spi_last); Serial.print(":SPI_MAX="); Serial.print(prof_spi_max);
        Serial.print(":UPD_LAST="); Serial.print(prof_update_last); Serial.print(":UPD_MAX="); Serial.print(prof_update_max);
        Serial.print(":MOVE_LAST="); Serial.print(prof_move_last); Serial.print(":MOVE_MAX="); Serial.print(prof_move_max);
        Serial.print(":M1_LAST="); Serial.print(prof_m1_foc_last); Serial.print(":M1_MAX="); Serial.print(prof_m1_foc_max);
        Serial.print(":M2_LAST="); Serial.print(prof_m2_foc_last); Serial.print(":M2_MAX="); Serial.print(prof_m2_foc_max);
        Serial.print(":TOT_LAST="); Serial.print(prof_total_last); Serial.print(":TOT_MAX="); Serial.print(prof_total_max);
        Serial.print(":ALL_PEAK="); Serial.print(isr_alltime_peak_cycles);
        Serial.print(":PER_LAST="); Serial.print(prof_period_last);
        Serial.print(":JIT_MAX="); Serial.print(prof_period_jitter_max * us_scale, 2);
        Serial.print(":PER_MIN="); Serial.print(prof_period_min * us_scale, 2);
        Serial.print(":PER_MAX="); Serial.print(prof_period_max * us_scale, 2);
        Serial.print(":SPIKES="); Serial.println((unsigned int)isr_jitter_spikes);
        prof_spi_max = 0; prof_update_max = 0; prof_m1_foc_max = 0; prof_m2_foc_max = 0; prof_move_max = 0; prof_total_max = 0;
        prof_period_min = 8500; prof_period_max = 8500; prof_period_jitter_max = 0;
        return;
    }

    // Check LUT toggle mode (e.g. 'LUT_MODE 1 1', 'LUT_MODE 2 0')
    if (strncasecmp(cmd, "LUT_MODE", 8) == 0) {
        const char *p = skip_ws(cmd + 8);
        const char *next = nullptr;
        int m = fast_atoi(p, &next);
        int en = fast_atoi(next);
        if (m == 1) encoder1.is_lut_active = (en != 0);
        else if (m == 2) encoder2.is_lut_active = (en != 0);
        Serial.print(">>> M"); Serial.print(m);
        Serial.print(" LUT Mode: "); Serial.println((en != 0) ? "128-Point LUT Active" : "Raw Angle");
        return;
    }

    // Check LUT dump command
    if (strncasecmp(cmd, "DUMP_LUT", 8) == 0 || strncasecmp(cmd, "LUT_DUMP", 8) == 0) {
        int m = fast_atoi(skip_ws(cmd + 8));
        if (m != 1 && m != 2) m = 1;
        dumpLUT(m);
        return;
    }

    // Check LUT line upload (e.g. 'LUT 1 0 0.001,...')
    if (strncasecmp(cmd, "LUT ", 4) == 0) {
        parseLUTUpload(cmd);
        return;
    }

    // Check CALIB command (e.g. 'CALIB 1', 'CALIB 2', 'CALIB 1 4.2')
    if (strncmp(cmd, "CALIB", 5) == 0 || strncmp(cmd, "calib", 5) == 0) {
        const char *p = skip_ws(cmd + 5);
        const char *next = nullptr;
        int m = fast_atoi(p, &next);
        if (m != 1 && m != 2) m = 1;
        float v = fast_atof(next);
        if (v <= 0.0f) v = CALIB_ALIGN_VOLTAGE;
        runMasterCalibrationSequence(m, v);
        return;
    }

    // Master sweep shorthand 'w1', 'W1', 'w2', 'W2' (Matching ESP32 'w' command)
    if ((cmd[0] == 'w' || cmd[0] == 'W') && (cmd[1] == '1' || cmd[1] == '2')) {
        int m = cmd[1] - '0';
        float v = fast_atof(cmd + 2);
        if (v <= 0.0f) v = CALIB_ALIGN_VOLTAGE;
        runMasterCalibrationSequence(m, v);
        return;
    }

    // Single-character or prefix commands
    if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "x") == 0 || strcmp(cmd, "X") == 0 || strcmp(cmd, " ") == 0 ||
        strcasecmp(cmd, "ESTOP") == 0 || strcasecmp(cmd, "EMERGENCY") == 0 || strcasecmp(cmd, "HALT") == 0) {
        triggerGlobalEmergencyStop();
        return;
    }

    if (strcmp(cmd, "E1") == 0) {
        g_estop_active = false;
        stall_guard1.reset();
        motor1.voltage_limit = MOTOR_VOLTAGE_LIMIT;
        motor1.PID_velocity.limit = motor1.voltage_limit;
        m1_ctrl.updateKe(motor1.KV_rating);
        resetMotorTorqueCtrl(m1_ctrl, m1_target_rpm, motor1.shaft_velocity, true);
        m1_enabled = true;
        driver1.enable();
        motor1.enable();
        if (!m1_calibrated) initMotorFOC(1);
        Serial.println(">>> M1: ENABLED");
        return;
    }
    if (strcmp(cmd, "E2") == 0) {
        g_estop_active = false;
        stall_guard2.reset();
        motor2.voltage_limit = MOTOR_VOLTAGE_LIMIT;
        motor2.PID_velocity.limit = motor2.voltage_limit;
        m2_ctrl.updateKe(motor2.KV_rating);
        resetMotorTorqueCtrl(m2_ctrl, m2_target_rpm, motor2.shaft_velocity, true);
        m2_enabled = true;
        driver2.enable();
        motor2.enable();
        if (!m2_calibrated) initMotorFOC(2);
        Serial.println(">>> M2: ENABLED");
        return;
    }
    if (strcmp(cmd, "E") == 0 || strcmp(cmd, "e") == 0) {
        g_estop_active = false;
        stall_guard1.reset();
        stall_guard2.reset();
        motor1.voltage_limit = MOTOR_VOLTAGE_LIMIT;
        motor1.PID_velocity.limit = motor1.voltage_limit;
        motor2.voltage_limit = MOTOR_VOLTAGE_LIMIT;
        motor2.PID_velocity.limit = motor2.voltage_limit;
        m1_ctrl.updateKe(motor1.KV_rating);
        m2_ctrl.updateKe(motor2.KV_rating);
        resetMotorTorqueCtrl(m1_ctrl, m1_target_rpm, motor1.shaft_velocity, true);
        resetMotorTorqueCtrl(m2_ctrl, m2_target_rpm, motor2.shaft_velocity, true);
        m1_enabled = true;
        m2_enabled = true;
        driver1.enable();
        driver2.enable();
        motor1.enable();
        motor2.enable();
        if (!m1_calibrated) initMotorFOC(1);
        if (!m2_calibrated) initMotorFOC(2);
        Serial.println(">>> BOTH MOTORS: ENABLED");
        return;
    }

    if (strcmp(cmd, "D1") == 0) {
        m1_enabled = false;
        motor1.disable();
        driver1.disable();
        digitalWrite(M1_EN, LOW);
        resetMotorTorqueCtrl(m1_ctrl, m1_target_rpm, motor1.shaft_velocity, true);
        motor1.voltage.q = 0.0f;
        motor1.voltage.d = 0.0f;
        stall_guard1.reset();
        motor1.voltage_limit = MOTOR_VOLTAGE_LIMIT;
        motor1.PID_velocity.limit = motor1.voltage_limit;
        Serial.println(">>> M1: DISABLED");
        return;
    }
    if (strcmp(cmd, "D2") == 0) {
        m2_enabled = false;
        motor2.disable();
        driver2.disable();
        digitalWrite(M2_EN, LOW);
        resetMotorTorqueCtrl(m2_ctrl, m2_target_rpm, motor2.shaft_velocity, true);
        motor2.voltage.q = 0.0f;
        motor2.voltage.d = 0.0f;
        stall_guard2.reset();
        motor2.voltage_limit = MOTOR_VOLTAGE_LIMIT;
        motor2.PID_velocity.limit = motor2.voltage_limit;
        Serial.println(">>> M2: DISABLED");
        return;
    }
    if (strcmp(cmd, "D") == 0 || strcmp(cmd, "d") == 0) {
        m1_enabled = false;
        m2_enabled = false;
        motor1.disable();
        motor2.disable();
        driver1.disable();
        driver2.disable();
        digitalWrite(M1_EN, LOW);
        digitalWrite(M2_EN, LOW);
        resetMotorTorqueCtrl(m1_ctrl, m1_target_rpm, motor1.shaft_velocity, true);
        resetMotorTorqueCtrl(m2_ctrl, m2_target_rpm, motor2.shaft_velocity, true);
        motor1.voltage.q = 0.0f;
        motor1.voltage.d = 0.0f;
        motor2.voltage.q = 0.0f;
        motor2.voltage.d = 0.0f;
        stall_guard1.reset();
        stall_guard2.reset();
        motor1.voltage_limit = MOTOR_VOLTAGE_LIMIT;
        motor1.PID_velocity.limit = motor1.voltage_limit;
        motor2.voltage_limit = MOTOR_VOLTAGE_LIMIT;
        motor2.PID_velocity.limit = motor2.voltage_limit;
        Serial.println(">>> BOTH MOTORS: DISABLED");
        return;
    }

    if (strncmp(cmd, "C1", 2) == 0 || strncmp(cmd, "c1", 2) == 0) {
        float v = fast_atof(cmd + 2);
        if (v <= 0.0f) v = CALIB_ALIGN_VOLTAGE;
        runComprehensiveElectricalAlignment(1, v);
        return;
    }
    if (strncmp(cmd, "C2", 2) == 0 || strncmp(cmd, "c2", 2) == 0) {
        float v = fast_atof(cmd + 2);
        if (v <= 0.0f) v = CALIB_ALIGN_VOLTAGE;
        runComprehensiveElectricalAlignment(2, v);
        return;
    }
    if (strcmp(cmd, "C") == 0 || strcmp(cmd, "c") == 0) {
        runComprehensiveElectricalAlignment(1, CALIB_ALIGN_VOLTAGE);
        runComprehensiveElectricalAlignment(2, CALIB_ALIGN_VOLTAGE);
        return;
    }

    if (strcasecmp(cmd, "R1") == 0 || strcasecmp(cmd, "RESET 1") == 0 || strcasecmp(cmd, "RESET1") == 0) { resetDefaults(1); return; }
    if (strcasecmp(cmd, "R2") == 0 || strcasecmp(cmd, "RESET 2") == 0 || strcasecmp(cmd, "RESET2") == 0) { resetDefaults(2); return; }
    if (strcasecmp(cmd, "R") == 0 || strcasecmp(cmd, "RESET") == 0 || strcasecmp(cmd, "CLR") == 0 || strcasecmp(cmd, "CLEAR") == 0) { resetDefaults(0); return; }
    if (strcasecmp(cmd, "RESET_ERRORS") == 0 || strcasecmp(cmd, "CLEAR_ERRORS") == 0 || strcasecmp(cmd, "ENC_RESET") == 0) {
        encoder1.resetErrors();
        encoder2.resetErrors();
        isr_jitter_spikes = 0;
        Serial.println(">>> [RESET] Cleared all encoder parity and SPI fault counters.");
        return;
    }
    if (strcasecmp(cmd, "REBOOT") == 0 || strcasecmp(cmd, "SYSRESET") == 0) {
        Serial.println(">>> 🔄 [SYSTEM] Rebooting STM32 MCU...");
        Serial.flush();
        delay(50);
        NVIC_SystemReset();
        return;
    }

    if (strcmp(cmd, "Z1") == 0) { encoder1.zeroCurrentPosition(); Serial.println(">>> M1: Tared."); return; }
    if (strcmp(cmd, "Z2") == 0) { encoder2.zeroCurrentPosition(); Serial.println(">>> M2: Tared."); return; }
    if (strcmp(cmd, "Z") == 0 || strcmp(cmd, "z") == 0) {
        encoder1.zeroCurrentPosition();
        encoder2.zeroCurrentPosition();
        Serial.println(">>> BOTH MOTORS: Tared.");
        return;
    }

    if (strcmp(cmd, "K") == 0 || strcmp(cmd, "k") == 0) {
        if (stall_guard1.latched_fault || stall_guard2.latched_fault) {
            stall_guard1.reset();
            stall_guard2.reset();
            m1_ctrl.stall_vtrq_cap = MOTOR_VOLTAGE_LIMIT;
            m2_ctrl.stall_vtrq_cap = MOTOR_VOLTAGE_LIMIT;
            motor1.voltage_limit = MOTOR_VOLTAGE_LIMIT;
            motor1.PID_velocity.limit = motor1.voltage_limit;
            motor2.voltage_limit = MOTOR_VOLTAGE_LIMIT;
            motor2.PID_velocity.limit = motor2.voltage_limit;
            Serial.println(">>> [STALL] Latched faults CLEARED. Stall watchdog active.");
        } else {
            stall_watchdog_enabled = !stall_watchdog_enabled;
            Serial.print(">>> Stall Watchdog: ");
            Serial.println(stall_watchdog_enabled ? "ENABLED" : "DISABLED");
        }
        return;
    }

    if (strcmp(cmd, "G") == 0 || strcmp(cmd, "g") == 0) {
        show_telemetry = !show_telemetry;
        Serial.print(">>> Telemetry Output: ");
        Serial.println(show_telemetry ? "ON" : "OFF");
        return;
    }

    if (strcmp(cmd, "ENC") == 0 || strcmp(cmd, "enc") == 0 || strcmp(cmd, "ENC4") == 0 || strcmp(cmd, "enc4") == 0) {
        Serial.print("[ENC]:M1:RAW="); Serial.print(encoder1.last_raw_14bit);
        Serial.print(":DEG="); Serial.print(encoder1.getSingleTurnDeg(), 2);
        Serial.print(":PARITY="); Serial.print(encoder1.last_parity_ok ? "OK" : "FAIL");
        Serial.print(":MAG="); Serial.print(encoder1.last_no_mag ? "NO_MAG" : "OK");
        Serial.print(":READS="); Serial.print(encoder1.total_reads);
        Serial.print(":RETRIES="); Serial.print(encoder1.parity_retries);
        Serial.print(":ERRS="); Serial.println(encoder1.parity_errors);

        Serial.print("[ENC]:M2:RAW="); Serial.print(encoder2.last_raw_14bit);
        Serial.print(":DEG="); Serial.print(encoder2.getSingleTurnDeg(), 2);
        Serial.print(":PARITY="); Serial.print(encoder2.last_parity_ok ? "OK" : "FAIL");
        Serial.print(":MAG="); Serial.print(encoder2.last_no_mag ? "NO_MAG" : "OK");
        Serial.print(":READS="); Serial.print(encoder2.total_reads);
        Serial.print(":RETRIES="); Serial.print(encoder2.parity_retries);
        Serial.print(":ERRS="); Serial.println(encoder2.parity_errors);
        return;
    }

    if (strcmp(cmd, "DFU") == 0 || strcmp(cmd, "dfu") == 0 || strcmp(cmd, "BOOT") == 0) {
        Serial.println(">>> 🔄 REBOOTING INTO STM32 SYSTEM DFU BOOTLOADER...");
        Serial.flush();
        delay(50);
        __HAL_RCC_PWR_CLK_ENABLE();
        HAL_PWR_EnableBkUpAccess();
        TAMP->BKP0R = 0xDEADBEEF;
        NVIC_SystemReset();
    }

    if (strcmp(cmd, "H") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0) {
        printHelp();
        return;
    }

    if (strcasecmp(cmd, "COAST1") == 0) {
        setMotorCommand(m1_ctrl, m1_target_rpm, m1_target_rpm, 0.0f, 0.0f, 0.0f);
        Serial.println(">>> M1: FREEWHEEL COAST (Vtrq = [0.00, 0.00])");
        return;
    }
    if (strcasecmp(cmd, "COAST2") == 0) {
        setMotorCommand(m2_ctrl, m2_target_rpm, m2_target_rpm, 0.0f, 0.0f, 0.0f);
        Serial.println(">>> M2: FREEWHEEL COAST (Vtrq = [0.00, 0.00])");
        return;
    }
    if (strcasecmp(cmd, "COAST") == 0) {
        setMotorCommand(m1_ctrl, m1_target_rpm, m1_target_rpm, 0.0f, 0.0f, 0.0f);
        setMotorCommand(m2_ctrl, m2_target_rpm, m2_target_rpm, 0.0f, 0.0f, 0.0f);
        Serial.println(">>> BOTH MOTORS: FREEWHEEL COAST (Vtrq = [0.00, 0.00])");
        return;
    }
    if (strncasecmp(cmd, "TRQ1", 4) == 0) {
        const char *p = skip_ws(cmd + 4);
        const char *next = nullptr;
        float vmin = fast_atof(p, &next);
        if (next != p) {
            p = skip_ws(next);
            float vmax = fast_atof(p, &next);
            if (next != p) {
                p = skip_ws(next);
                const char *ff_end = nullptr;
                float vff = fast_atof(p, &ff_end);
                if (ff_end == p) vff = 0.0f;
                setMotorCommand(m1_ctrl, m1_target_rpm, m1_target_rpm, vmin, vmax, vff);
                Serial.print(">>> M1 Vtrq ["); Serial.print(vmin, 2);
                Serial.print(", "); Serial.print(vmax, 2);
                Serial.print("] ff="); Serial.println(vff, 2);
            }
        }
        return;
    }
    if (strncasecmp(cmd, "TRQ2", 4) == 0) {
        const char *p = skip_ws(cmd + 4);
        const char *next = nullptr;
        float vmin = fast_atof(p, &next);
        if (next != p) {
            p = skip_ws(next);
            float vmax = fast_atof(p, &next);
            if (next != p) {
                p = skip_ws(next);
                const char *ff_end = nullptr;
                float vff = fast_atof(p, &ff_end);
                if (ff_end == p) vff = 0.0f;
                setMotorCommand(m2_ctrl, m2_target_rpm, m2_target_rpm, vmin, vmax, vff);
                Serial.print(">>> M2 Vtrq ["); Serial.print(vmin, 2);
                Serial.print(", "); Serial.print(vmax, 2);
                Serial.print("] ff="); Serial.println(vff, 2);
            }
        }
        return;
    }
    if (strncasecmp(cmd, "ICLAMP1", 7) == 0) {
        float val = _constrain(fast_atof(cmd + 7), 0.1f, 6.0f);
        m1_ctrl.i_clamp_v = val;
        Serial.print(">>> M1 I-Clamp: +/-"); Serial.print(val, 2); Serial.println(" V");
        return;
    }
    if (strncasecmp(cmd, "ICLAMP2", 7) == 0) {
        float val = _constrain(fast_atof(cmd + 7), 0.1f, 6.0f);
        m2_ctrl.i_clamp_v = val;
        Serial.print(">>> M2 I-Clamp: +/-"); Serial.print(val, 2); Serial.println(" V");
        return;
    }

    // Parameter tuning with float parsing
    if (strncmp(cmd, "T1", 2) == 0) {
        float val = fast_atof(cmd + 2);
        setMotorCommand(m1_ctrl, m1_target_rpm, val, m1_ctrl.cmd_vtrq_min, m1_ctrl.cmd_vtrq_max, m1_ctrl.cmd_vtrq_ff);
        Serial.print(">>> M1 Target Speed: "); Serial.print(m1_target_rpm, 1); Serial.println(" RPM");
        return;
    }
    if (strncmp(cmd, "T2", 2) == 0) {
        float val = fast_atof(cmd + 2);
        setMotorCommand(m2_ctrl, m2_target_rpm, val, m2_ctrl.cmd_vtrq_min, m2_ctrl.cmd_vtrq_max, m2_ctrl.cmd_vtrq_ff);
        Serial.print(">>> M2 Target Speed: "); Serial.print(m2_target_rpm, 1); Serial.println(" RPM");
        return;
    }
    if (strncmp(cmd, "T", 1) == 0) {
        float val = fast_atof(cmd + 1);
        setMotorCommand(m1_ctrl, m1_target_rpm, val, m1_ctrl.cmd_vtrq_min, m1_ctrl.cmd_vtrq_max, m1_ctrl.cmd_vtrq_ff);
        setMotorCommand(m2_ctrl, m2_target_rpm, val, m2_ctrl.cmd_vtrq_min, m2_ctrl.cmd_vtrq_max, m2_ctrl.cmd_vtrq_ff);
        Serial.print(">>> Coupled Target Speed: "); Serial.print(val, 1); Serial.println(" RPM");
        return;
    }

    if (strncmp(cmd, "P1", 2) == 0) {
        float val = fast_atof(cmd + 2);
        if (val >= 0.0f) { motor1.PID_velocity.P = val; Serial.print(">>> M1 P: "); Serial.println(val, 4); }
        return;
    }
    if (strncmp(cmd, "P2", 2) == 0) {
        float val = fast_atof(cmd + 2);
        if (val >= 0.0f) { motor2.PID_velocity.P = val; Serial.print(">>> M2 P: "); Serial.println(val, 4); }
        return;
    }

    if (strncmp(cmd, "I1", 2) == 0) {
        float val = fast_atof(cmd + 2);
        if (val >= 0.0f) { motor1.PID_velocity.I = val; Serial.print(">>> M1 I: "); Serial.println(val, 4); }
        return;
    }
    if (strncmp(cmd, "I2", 2) == 0) {
        float val = fast_atof(cmd + 2);
        if (val >= 0.0f) { motor2.PID_velocity.I = val; Serial.print(">>> M2 I: "); Serial.println(val, 4); }
        return;
    }

    if (strncmp(cmd, "D1", 2) == 0) {
        float val = fast_atof(cmd + 2);
        if (val >= 0.0f) { motor1.PID_velocity.D = val; Serial.print(">>> M1 D: "); Serial.println(val, 5); }
        return;
    }
    if (strncmp(cmd, "D2", 2) == 0) {
        float val = fast_atof(cmd + 2);
        if (val >= 0.0f) { motor2.PID_velocity.D = val; Serial.print(">>> M2 D: "); Serial.println(val, 5); }
        return;
    }

    if (strncasecmp(cmd, "LPF1", 4) == 0 || strncmp(cmd, "F1", 2) == 0) {
        const char *p = (strncasecmp(cmd, "LPF1", 4) == 0) ? (cmd + 4) : (cmd + 2);
        float val = fast_atof(p);
        if (val >= 50.0f) val *= 1e-6f;
        else if (val >= 0.05f) val *= 0.001f;
        if (val > 0.0f) {
            motor1.LPF_velocity.Tf = val;
            Serial.print(">>> M1 LPF Tf: "); Serial.print(val * 1000.0f, 2);
            Serial.print(" ms ("); Serial.print(val, 6); Serial.println(" s)");
        }
        return;
    }
    if (strncasecmp(cmd, "LPF2", 4) == 0 || strncmp(cmd, "F2", 2) == 0) {
        const char *p = (strncasecmp(cmd, "LPF2", 4) == 0) ? (cmd + 4) : (cmd + 2);
        float val = fast_atof(p);
        if (val >= 50.0f) val *= 1e-6f;
        else if (val >= 0.05f) val *= 0.001f;
        if (val > 0.0f) {
            motor2.LPF_velocity.Tf = val;
            Serial.print(">>> M2 LPF Tf: "); Serial.print(val * 1000.0f, 2);
            Serial.print(" ms ("); Serial.print(val, 6); Serial.println(" s)");
        }
        return;
    }

    if (strncasecmp(cmd, "DIR1", 4) == 0) {
        int d = fast_atoi(cmd + 4);
        motor1.sensor_direction = (d >= 0) ? Direction::CW : Direction::CCW;
        Serial.print(">>> M1 Sensor Direction: ");
        Serial.println(motor1.sensor_direction == Direction::CW ? "CW (+1)" : "CCW (-1)");
        return;
    }
    if (strncasecmp(cmd, "DIR2", 4) == 0) {
        int d = fast_atoi(cmd + 4);
        motor2.sensor_direction = (d >= 0) ? Direction::CW : Direction::CCW;
        Serial.print(">>> M2 Sensor Direction: ");
        Serial.println(motor2.sensor_direction == Direction::CW ? "CW (+1)" : "CCW (-1)");
        return;
    }

    if (strncmp(cmd, "V1", 2) == 0) {
        float val = fast_atof(cmd + 2);
        if (val > 0.0f) { motor1.voltage_limit = val; motor1.PID_velocity.limit = val; Serial.print(">>> M1 V-Limit: "); Serial.println(val, 2); }
        return;
    }
    if (strncmp(cmd, "V2", 2) == 0) {
        float val = fast_atof(cmd + 2);
        if (val > 0.0f) { motor2.voltage_limit = val; motor2.PID_velocity.limit = val; Serial.print(">>> M2 V-Limit: "); Serial.println(val, 2); }
        return;
    }

    if (strncasecmp(cmd, "RAMP1", 5) == 0) {
        float val = fast_atof(cmd + 5);
        if (val > 0.0f) { motor1.PID_velocity.output_ramp = val; Serial.print(">>> M1 Voltage Ramp: "); Serial.print(val, 1); Serial.println(" V/s"); }
        return;
    }
    if (strncasecmp(cmd, "RAMP2", 5) == 0) {
        float val = fast_atof(cmd + 5);
        if (val > 0.0f) { motor2.PID_velocity.output_ramp = val; Serial.print(">>> M2 Voltage Ramp: "); Serial.print(val, 1); Serial.println(" V/s"); }
        return;
    }

    if (strncasecmp(cmd, "MAXRPM1", 7) == 0) {
        float val = fast_atof(cmd + 7);
        if (val > 0.0f) { motor1.velocity_limit = val * 0.104719755f; Serial.print(">>> M1 Max Speed: "); Serial.print(val, 1); Serial.println(" RPM"); }
        return;
    }
    if (strncasecmp(cmd, "MAXRPM2", 7) == 0) {
        float val = fast_atof(cmd + 7);
        if (val > 0.0f) { motor2.velocity_limit = val * 0.104719755f; Serial.print(">>> M2 Max Speed: "); Serial.print(val, 1); Serial.println(" RPM"); }
        return;
    }

    if (strncasecmp(cmd, "KV1", 3) == 0) {
        float val = fast_atof(cmd + 3);
        if (val > 0.0f) { motor1.KV_rating = val; m1_ctrl.updateKe(val); Serial.print(">>> M1 KV: "); Serial.println(val, 1); }
        return;
    }
    if (strncasecmp(cmd, "KV2", 3) == 0) {
        float val = fast_atof(cmd + 3);
        if (val > 0.0f) { motor2.KV_rating = val; m2_ctrl.updateKe(val); Serial.print(">>> M2 KV: "); Serial.println(val, 1); }
        return;
    }

    if (strncasecmp(cmd, "LAG1", 4) == 0 || strncasecmp(cmd, "L1", 2) == 0) {
        const char *p = (strncasecmp(cmd, "LAG1", 4) == 0) ? (cmd + 4) : (cmd + 2);
        float val = fast_atof(p);
        if (val >= 0.0f) { m1_lag_comp_us = val; Serial.print(">>> M1 Sensor Lag Comp: "); Serial.print(val, 1); Serial.println(" us"); }
        return;
    }
    if (strncasecmp(cmd, "LAG2", 4) == 0 || strncasecmp(cmd, "L2", 2) == 0) {
        const char *p = (strncasecmp(cmd, "LAG2", 4) == 0) ? (cmd + 4) : (cmd + 2);
        float val = fast_atof(p);
        if (val >= 0.0f) { m2_lag_comp_us = val; Serial.print(">>> M2 Sensor Lag Comp: "); Serial.print(val, 1); Serial.println(" us"); }
        return;
    }
    if (strncasecmp(cmd, "LAG", 3) == 0) {
        float val = fast_atof(cmd + 3);
        if (val >= 0.0f) { m1_lag_comp_us = val; m2_lag_comp_us = val; Serial.print(">>> Both Motors Sensor Lag Comp: "); Serial.print(val, 1); Serial.println(" us"); }
        return;
    }

    if (strncasecmp(cmd, "DWT1", 4) == 0) {
        float val = fast_atof(cmd + 4);
        if (val > 0.0f) {
            encoder1.setDWTWindowMs(val);
            Serial.print(">>> M1 DWT Velocity Window: "); Serial.print(encoder1.window_ms, 2);
            Serial.print(" ms ("); Serial.print(encoder1.window_samples); Serial.println(" samples)");
        }
        return;
    }
    if (strncasecmp(cmd, "DWT2", 4) == 0) {
        float val = fast_atof(cmd + 4);
        if (val > 0.0f) {
            encoder2.setDWTWindowMs(val);
            Serial.print(">>> M2 DWT Velocity Window: "); Serial.print(encoder2.window_ms, 2);
            Serial.print(" ms ("); Serial.print(encoder2.window_samples); Serial.println(" samples)");
        }
        return;
    }
    if (strncasecmp(cmd, "DWT", 3) == 0) {
        float val = fast_atof(cmd + 3);
        if (val > 0.0f) {
            encoder1.setDWTWindowMs(val);
            encoder2.setDWTWindowMs(val);
            Serial.print(">>> Both Motors DWT Velocity Window: "); Serial.print(val, 2);
            Serial.print(" ms ("); Serial.print(encoder1.window_samples); Serial.println(" samples)");
        }
        return;
    }

    // Direct relative zero nudge via standalone +<delta> or -<delta>
    if (cmd[0] == '+' || (cmd[0] == '-' && (cmd[1] == '.' || (cmd[1] >= '0' && cmd[1] <= '9')))) {
        float delta = fast_atof(cmd);
        motor1.zero_electric_angle = normalizeAngleRad(m1_base_zero_angle + delta);
        m1_base_zero_angle = motor1.zero_electric_angle;
        m1_calibrated = true;
        Serial.print(">>> M1 Zero Electric Angle Nudged: ");
        Serial.print(motor1.zero_electric_angle, 4);
        Serial.print(" rad ("); Serial.print(motor1.zero_electric_angle * 57.2957795f, 1);
        Serial.print(" deg) [delta: "); Serial.print(delta, 4); Serial.println("]");
        return;
    }

    if (strncasecmp(cmd, "ZERO1", 5) == 0 || strncasecmp(cmd, "O1", 2) == 0) {
        const char *p = (strncasecmp(cmd, "ZERO1", 5) == 0) ? (cmd + 5) : (cmd + 2);
        p = skip_ws(p);
        bool is_rel = (*p == '+' || *p == '-');
        float val = fast_atof(p);
        if (is_rel) {
            motor1.zero_electric_angle = normalizeAngleRad(m1_base_zero_angle + val);
        } else {
            motor1.zero_electric_angle = normalizeAngleRad(val);
        }
        m1_base_zero_angle = motor1.zero_electric_angle;
        m1_calibrated = true;
        Serial.print(">>> M1 Zero Electric Angle ");
        Serial.print(is_rel ? "Nudged to: " : "Override: ");
        Serial.print(motor1.zero_electric_angle, 4);
        Serial.print(" rad ("); Serial.print(motor1.zero_electric_angle * 57.2957795f, 1);
        Serial.println(" deg)");
        return;
    }
    if (strncasecmp(cmd, "ZERO2", 5) == 0 || strncasecmp(cmd, "O2", 2) == 0) {
        const char *p = (strncasecmp(cmd, "ZERO2", 5) == 0) ? (cmd + 5) : (cmd + 2);
        p = skip_ws(p);
        bool is_rel = (*p == '+' || *p == '-');
        float val = fast_atof(p);
        if (is_rel) {
            motor2.zero_electric_angle = normalizeAngleRad(m2_base_zero_angle + val);
        } else {
            motor2.zero_electric_angle = normalizeAngleRad(val);
        }
        m2_base_zero_angle = motor2.zero_electric_angle;
        m2_calibrated = true;
        Serial.print(">>> M2 Zero Electric Angle ");
        Serial.print(is_rel ? "Nudged to: " : "Override: ");
        Serial.print(motor2.zero_electric_angle, 4);
        Serial.print(" rad ("); Serial.print(motor2.zero_electric_angle * 57.2957795f, 1);
        Serial.println(" deg)");
        return;
    }

    if (strncasecmp(cmd, "A1", 2) == 0) {
        float val = fast_atof(cmd + 2);
        if (val > 0.0f) { motor1.voltage_sensor_align = val; Serial.print(">>> M1 Align Voltage: "); Serial.print(val, 2); Serial.println(" V"); }
        return;
    }
    if (strncasecmp(cmd, "A2", 2) == 0) {
        float val = fast_atof(cmd + 2);
        if (val > 0.0f) { motor2.voltage_sensor_align = val; Serial.print(">>> M2 Align Voltage: "); Serial.print(val, 2); Serial.println(" V"); }
        return;
    }

    // Open-loop voltage commands: 'v1 <volts>', 'v2 <volts>', or 'v <volts>' (both)
    if (strncasecmp(cmd, "v1 ", 3) == 0 || strncasecmp(cmd, "v1", 2) == 0) {
        float val = fast_atof(cmd + 2);
        driver1.enable();
        motor1.enable();
        m1_enabled = true;
#if defined(INVERT_ENCODER_DIRECTION) && (INVERT_ENCODER_DIRECTION == 1)
        val = -val;
#endif
        motor1.voltage.q = val;
        motor1.voltage.d = 0.0f;
        motor1.setPhaseVoltage(fabsf(val), 0.0f, (val >= 0.0f) ? 0.0f : _PI);
        Serial.print(">>> M1 Open-Loop Voltage: "); Serial.print(val, 2); Serial.println(" V");
        return;
    }
    if (strncasecmp(cmd, "v2 ", 3) == 0 || strncasecmp(cmd, "v2", 2) == 0) {
        float val = fast_atof(cmd + 2);
        driver2.enable();
        motor2.enable();
        m2_enabled = true;
#if defined(INVERT_ENCODER_DIRECTION) && (INVERT_ENCODER_DIRECTION == 1)
        val = -val;
#endif
        motor2.voltage.q = val;
        motor2.voltage.d = 0.0f;
        motor2.setPhaseVoltage(fabsf(val), 0.0f, (val >= 0.0f) ? 0.0f : _PI);
        Serial.print(">>> M2 Open-Loop Voltage: "); Serial.print(val, 2); Serial.println(" V");
        return;
    }
    if ((cmd[0] == 'v' || cmd[0] == 'V') && (cmd[1] == ' ' || cmd[1] == '+' || cmd[1] == '-' || (cmd[1] >= '0' && cmd[1] <= '9'))) {
        float val = fast_atof(cmd + 1);
        driver1.enable();
        driver2.enable();
        motor1.enable();
        motor2.enable();
        m1_enabled = true;
        m2_enabled = true;
#if defined(INVERT_ENCODER_DIRECTION) && (INVERT_ENCODER_DIRECTION == 1)
        val = -val;
#endif
        motor1.voltage.q = val;
        motor1.voltage.d = 0.0f;
        motor2.voltage.q = val;
        motor2.voltage.d = 0.0f;
        motor1.setPhaseVoltage(fabsf(val), 0.0f, (val >= 0.0f) ? 0.0f : _PI);
        motor2.setPhaseVoltage(fabsf(val), 0.0f, (val >= 0.0f) ? 0.0f : _PI);
        Serial.print(">>> Both Motors Open-Loop Voltage: "); Serial.print(val, 2); Serial.println(" V");
        return;
    }

    if (strcasecmp(cmd, "s") == 0 || strcasecmp(cmd, "STATUS") == 0) {
        printHelp();
        return;
    }

    if (strncasecmp(cmd, "SET_MODE", 8) == 0) {
        int mode = fast_atoi(cmd + 8);
        encoder1.is_lut_active = (mode == 1);
        encoder2.is_lut_active = (mode == 1);
        Serial.print(">>> Calibration Mode: ");
        Serial.println((mode == 1) ? "128-Point LUT Active" : "Raw Angle");
        return;
    }
    if (strncasecmp(cmd, "M1", 2) == 0 && (cmd[2] == ' ' || cmd[2] == '\0')) {
        int mode = fast_atoi(cmd + 2);
        encoder1.is_lut_active = (mode == 1);
        Serial.print(">>> M1 LUT Mode: ");
        Serial.println((mode == 1) ? "128-Point LUT Active" : "Raw Angle");
        return;
    }
    if (strncasecmp(cmd, "M2", 2) == 0 && (cmd[2] == ' ' || cmd[2] == '\0')) {
        int mode = fast_atoi(cmd + 2);
        encoder2.is_lut_active = (mode == 1);
        Serial.print(">>> M2 LUT Mode: ");
        Serial.println((mode == 1) ? "128-Point LUT Active" : "Raw Angle");
        return;
    }

    if (strcasecmp(cmd, "u") == 0 || strcasecmp(cmd, "u1") == 0) {
        dumpLUT(1);
        return;
    }
    if (strcasecmp(cmd, "u2") == 0) {
        dumpLUT(2);
        return;
    }

    if (strcasecmp(cmd, "SAVE") == 0 || strcasecmp(cmd, "save") == 0) {
        bool ok = saveConfigToFlash();
        dumpConfigJSON();
        Serial.println(ok ? "[CONFIG_SAVED]:OK" : "[CONFIG_SAVED]:FAIL");
        return;
    }

    if (strcasecmp(cmd, "LOAD") == 0 || strcasecmp(cmd, "load") == 0) {
        bool ok = loadConfigFromFlash();
        dumpConfigJSON();
        Serial.println(ok ? "[CONFIG_LOADED]:OK" : "[CONFIG_LOADED]:FAIL");
        return;
    }

    if (strcasecmp(cmd, "EXPORT") == 0 || strcasecmp(cmd, "export") == 0) {
        dumpConfigJSON();
        return;
    }

    Serial.print("ERR: Unrecognized command: ");
    Serial.println(cmd);
}

// ----------------------------------------------------------------------------
// Autonomous Industry-Grade Stall Protection Watchdog (Thermal I^2*t + FSM)
// Uses decoupled winding effort V_trq (never total V_q = Ke*w + V_trq)
// ----------------------------------------------------------------------------
void updateStallGuard(int id, BLDCMotor &motor, FastBLDCDriver3PWM &driver,
                      MT6816Sensor &encoder, int en_pin,
                      MotorTorqueCtrl &ctrl,
                      volatile bool &enabled, volatile float &target_rpm,
                      IndustrialStallGuard &guard, float dt) {
    if (!enabled) {
        ctrl.stall_vtrq_cap = MOTOR_VOLTAGE_LIMIT;
        guard.reset();
        return;
    }

    if (guard.latched_fault) {
        enabled = false;
        motor.disable();
        driver.disable();
        digitalWrite(en_pin, LOW);
        resetMotorTorqueCtrl(ctrl, target_rpm, motor.shaft_velocity, true);
        return;
    }

    bool enc_bad = !encoder.is_connected || encoder.last_no_mag ||
                   (!encoder.last_parity_ok && encoder.consecutive_parity_errors >= 50);
    if (enc_bad) {
        if (guard.enc_fault_ticks < 255) {
            guard.enc_fault_ticks++;
        }
        if (guard.enc_fault_ticks >= 5) {
            guard.state = StallState::LATCHED_FAULT;
            guard.latched_fault = true;
            enabled = false;
            motor.disable();
            driver.disable();
            digitalWrite(en_pin, LOW);
            resetMotorTorqueCtrl(ctrl, target_rpm, motor.shaft_velocity, true);
            Serial.print("[ENC_FAULT]:M"); Serial.println(id);
            return;
        }
    } else {
        guard.enc_fault_ticks = 0;
    }

    float actual_rpm = motor.shaftVelocity() * 9.54929659f;
    float abs_actual = fabsf(actual_rpm);
    float abs_target = fabsf(target_rpm);
    float abs_effort = fabsf(ctrl.v_trq_slewed);
    float vel_error = fabsf(target_rpm - actual_rpm);

    // Effort-motion co-incidence and reversal masking
    bool is_reversing = (target_rpm * actual_rpm < 0.0f) && (abs_actual > STALL_VELOCITY_FLOOR_RPM);
    bool effort_active = (abs_effort >= STALL_MIN_EFFORT_VOLTS);
    bool motion_stalled = (abs_target >= STALL_VELOCITY_FLOOR_RPM) && (abs_actual < STALL_VELOCITY_FLOOR_RPM);
    bool severe_lag = (abs_effort >= STALL_SATURATION_VOLTS) && (vel_error > STALL_VELOCITY_LAG_RPM);

    bool stall_qualified = effort_active && !is_reversing && (motion_stalled || severe_lag);

    // Thermal I^2*t leaky energy accumulator
    if (stall_qualified) {
        float excess = (abs_effort * abs_effort) - (STALL_I2T_CONT_VOLTS * STALL_I2T_CONT_VOLTS);
        if (excess > 0.0f) {
            guard.energy_accumulator += excess * dt;
        }
    } else {
        guard.energy_accumulator = fmaxf(0.0f, guard.energy_accumulator - (STALL_I2T_LEAK_RATE * dt));
    }

    // Progressive Multi-Stage Mitigation FSM
    switch (guard.state) {
        case StallState::NORMAL:
            ctrl.stall_vtrq_cap = MOTOR_VOLTAGE_LIMIT;
            if (guard.energy_accumulator >= (STALL_I2T_TRIP_ENERGY * 0.60f)) {
                guard.state = StallState::WARNING;
            }
            break;

        case StallState::WARNING:
            ctrl.stall_vtrq_cap = MOTOR_VOLTAGE_LIMIT;
            if (guard.energy_accumulator < (STALL_I2T_TRIP_ENERGY * 0.30f)) {
                guard.state = StallState::NORMAL;
            } else if (guard.energy_accumulator >= STALL_I2T_TRIP_ENERGY) {
                guard.state = StallState::FOLDBACK;
                guard.state_timer = millis();
                ctrl.stall_vtrq_cap = STALL_FOLDBACK_VOLTS;
                Serial.print("[STALL_FOLDBACK]:M"); Serial.println(id);
            }
            break;

        case StallState::FOLDBACK:
            ctrl.stall_vtrq_cap = STALL_FOLDBACK_VOLTS;
            if (millis() - guard.state_timer >= 300) {
                if (guard.recovery_attempts < STALL_MAX_RECOVERY_TRIES) {
                    guard.recovery_attempts++;
                    guard.state = StallState::UNWEDGE_DITHER;
                    guard.state_timer = millis();
                    setMotorCommand(ctrl, target_rpm, -target_rpm * 0.5f,
                                    -STALL_FOLDBACK_VOLTS, STALL_FOLDBACK_VOLTS, 0.0f);
                } else {
                    guard.state = StallState::LATCHED_FAULT;
                    guard.latched_fault = true;
                    enabled = false;
                    driver.disable();
                    resetMotorTorqueCtrl(ctrl, target_rpm, motor.shaft_velocity, true);
                    Serial.print("[STALL_FAULT]:M"); Serial.println(id);
                }
            }
            break;

        case StallState::UNWEDGE_DITHER:
            ctrl.stall_vtrq_cap = STALL_FOLDBACK_VOLTS;
            if (abs_actual >= STALL_VELOCITY_FLOOR_RPM * 2.0f) {
                guard.reset();
                ctrl.stall_vtrq_cap = MOTOR_VOLTAGE_LIMIT;
                Serial.print("[STALL_CLEAR]:M"); Serial.println(id);
            } else if (millis() - guard.state_timer >= STALL_DITHER_PULSE_MS) {
                guard.state = StallState::FOLDBACK;
                guard.state_timer = millis();
            }
            break;

        case StallState::LATCHED_FAULT:
            enabled = false;
            driver.disable();
            resetMotorTorqueCtrl(ctrl, target_rpm, motor.shaft_velocity, true);
            break;
    }
}

void checkStallWatchdog() {
    if (!stall_watchdog_enabled || m1_calib_active || m2_calib_active) return;

    static uint32_t last_check_us = 0;
    uint32_t now_us = micros();
    if (last_check_us == 0) {
        last_check_us = now_us;
        return;
    }
    uint32_t diff_us = now_us - last_check_us;
    if (diff_us < 5000) return; // ~200 Hz evaluation
    float dt = (float)diff_us * 1e-6f;
    last_check_us = now_us;
    if (dt > 0.05f) dt = 0.05f;

    if (!m1_calib_active) {
        updateStallGuard(1, motor1, driver1, encoder1, M1_EN, m1_ctrl, m1_enabled, m1_target_rpm, stall_guard1, dt);
    }
    if (!m2_calib_active) {
        updateStallGuard(2, motor2, driver2, encoder2, M2_EN, m2_ctrl, m2_enabled, m2_target_rpm, stall_guard2, dt);
    }
}

// ----------------------------------------------------------------------------
// 26-Byte Telemetry Frame Generator (25 Hz)
// ----------------------------------------------------------------------------
void sendBinaryTelemetry() {
    if (!show_telemetry || m1_calib_active || m2_calib_active) return;
    TelemetryPacket pkt;
    pkt.header[0] = 0xAA;
    pkt.header[1] = 0x55;

    // Motor 1 Fields
    pkt.m1_raw_angle  = encoder1.last_raw_14bit;
    pkt.m1_actual_rpm = (int16_t)(motor1.shaftVelocity() * 9.54929659f * 10.0f);
    pkt.m1_target_rpm = (int16_t)(m1_target_rpm * 10.0f);
    pkt.m1_uq_mv      = (int16_t)(motor1.voltage.q * 1000.0f);

    uint8_t s1 = 0;
    if (m1_enabled)            s1 |= (1 << 0);
    if (encoder1.is_lut_active) s1 |= (1 << 1);
    if (!encoder1.last_no_mag) s1 |= (1 << 2);
    if (encoder1.last_parity_ok) s1 |= (1 << 3);
    if (m1_is_stalled)         s1 |= (1 << 4);
    if (stall_guard1.state == StallState::WARNING || stall_guard1.state == StallState::FOLDBACK || stall_guard1.state == StallState::UNWEDGE_DITHER || stall_guard1.latched_fault) {
        s1 |= (1 << 4);
    }
    if (m1_calib_active)       s1 |= (1 << 5);
    if (stall_guard1.latched_fault) s1 |= (1 << 6);
    pkt.m1_status = s1;

    // Motor 2 Fields
    pkt.m2_raw_angle  = encoder2.last_raw_14bit;
    pkt.m2_actual_rpm = (int16_t)(motor2.shaftVelocity() * 9.54929659f * 10.0f);
    pkt.m2_target_rpm = (int16_t)(m2_target_rpm * 10.0f);
    pkt.m2_uq_mv      = (int16_t)(motor2.voltage.q * 1000.0f);

    uint8_t s2 = 0;
    if (m2_enabled)            s2 |= (1 << 0);
    if (encoder2.is_lut_active) s2 |= (1 << 1);
    if (!encoder2.last_no_mag) s2 |= (1 << 2);
    if (encoder2.last_parity_ok) s2 |= (1 << 3);
    if (m2_is_stalled)         s2 |= (1 << 4);
    if (stall_guard2.state == StallState::WARNING || stall_guard2.state == StallState::FOLDBACK || stall_guard2.state == StallState::UNWEDGE_DITHER || stall_guard2.latched_fault) {
        s2 |= (1 << 4);
    }
    if (m2_calib_active)       s2 |= (1 << 5);
    if (stall_guard2.latched_fault) s2 |= (1 << 6);
    pkt.m2_status = s2;

    // Real-Time Control & Profiler Stats
    pkt.foc_khz_x10    = 200; // 20.0 kHz
    // Peak ISR latency converted from cycles to microseconds
    uint32_t lat_us    = isr_peak_cycles / (SystemCoreClock / 1000000);
    pkt.isr_lat_us     = (lat_us > 255) ? 255 : (uint8_t)lat_us;
    pkt.spikes_count   = isr_jitter_spikes;
    pkt.calib_progress = live_calib_progress;

    // Reset peak cycles every telemetry period to track rolling max latency
    isr_peak_cycles = isr_last_cycles;

    // Compute XOR checksum over bytes [2 .. 24] (23 bytes total)
    uint8_t chk = 0;
    const uint8_t *raw_ptr = (const uint8_t *)&pkt;
    for (int i = 2; i < 25; i++) {
        chk ^= raw_ptr[i];
    }
    pkt.checksum = chk;

    // Atomic binary write to serial buffer
    Serial.write(raw_ptr, sizeof(TelemetryPacket));
}

// ----------------------------------------------------------------------------
// ESP32 High-Speed 400 Hz Bidirectional Link Handlers
// ----------------------------------------------------------------------------

void sendESP32Feedback() {
    if (SerialESP.availableForWrite() < (int)sizeof(STM32FeedbackPacket)) return;

    STM32FeedbackPacket resp;
    resp.header[0] = STM32_FEEDBACK_SYNC_1;
    resp.header[1] = STM32_FEEDBACK_SYNC_2;

    uint8_t st = 0;
    if (m1_enabled) st |= STM32_STATUS_M1_ENABLED;
    if (m2_enabled) st |= STM32_STATUS_M2_ENABLED;
    if (m1_calib_active || m2_calib_active) st |= STM32_STATUS_CALIB_ACTIVE;
    if (stall_guard1.state != StallState::NORMAL || stall_guard2.state != StallState::NORMAL) st |= STM32_STATUS_STALL_WARNING;
    if (stall_guard1.latched_fault || stall_guard2.latched_fault) st |= STM32_STATUS_LATCHED_FAULT;
    if (encoder1.last_parity_ok && !encoder1.last_no_mag) st |= STM32_STATUS_ENC1_OK;
    if (encoder2.last_parity_ok && !encoder2.last_no_mag) st |= STM32_STATUS_ENC2_OK;
    resp.status_flags = st;

    resp.m1_actual_rpm = motor1.shaftVelocity() * 9.54929659f;
    resp.m2_actual_rpm = motor2.shaftVelocity() * 9.54929659f;
    resp.calib_progress = live_calib_progress;

    uint8_t chk = 0;
    const uint8_t *p = (const uint8_t *)&resp;
    for (size_t i = 2; i < sizeof(STM32FeedbackPacket) - 1; i++) {
        chk ^= p[i];
    }
    resp.checksum = chk;

    SerialESP.write((const uint8_t *)&resp, sizeof(resp));
}

void processESP32Link() {
    static uint8_t rx_buf[sizeof(STM32MotorCommandPacket)];
    static uint8_t rx_idx = 0;
    static bool sync_found = false;
    float latest_drive_m1 = 0.0f;
    float latest_drive_m2 = 0.0f;
    int16_t latest_m1_vmin_mv = -12000;
    int16_t latest_m1_vmax_mv =  12000;
    int16_t latest_m1_vff_mv  =  0;
    int16_t latest_m2_vmin_mv = -12000;
    int16_t latest_m2_vmax_mv =  12000;
    int16_t latest_m2_vff_mv  =  0;
    bool have_latest_drive = false;
    bool have_valid_packet = false;
    size_t bytes_processed = 0;

    uint32_t uisr = USART1->ISR;
    if (uisr & USART_ISR_ORE) { esp_usart1_ore++; USART1->ICR = USART_ICR_ORECF; }
    if (uisr & USART_ISR_FE)  { esp_usart1_fe++;  USART1->ICR = USART_ICR_FECF; }
    if (uisr & USART_ISR_NE)  { esp_usart1_ne++;  USART1->ICR = USART_ICR_NECF; }

    while (SerialESP.available() > 0 &&
           bytes_processed < ESP_COMMAND_BYTE_BUDGET) {
        uint8_t b = SerialESP.read();
        bytes_processed++;
        esp_rx_bytes++;

        if (!sync_found) {
            if (b == STM32_CMD_SYNC_1) {
                rx_buf[0] = STM32_CMD_SYNC_1;
                rx_idx = 1;
            } else if (rx_idx == 1 && b == STM32_CMD_SYNC_2) {
                rx_buf[1] = STM32_CMD_SYNC_2;
                rx_idx = 2;
                sync_found = true;
            } else {
                rx_idx = 0;
            }
        } else {
            rx_buf[rx_idx++] = b;
            if (rx_idx >= sizeof(STM32MotorCommandPacket)) {
                sync_found = false;
                rx_idx = 0;

                // Verify XOR checksum over bytes [2 .. 22]
                uint8_t chk = 0;
                for (size_t i = 2; i < sizeof(STM32MotorCommandPacket) - 1; i++) {
                    chk ^= rx_buf[i];
                }
                if (chk == rx_buf[sizeof(STM32MotorCommandPacket) - 1]) {
                    const STM32MotorCommandPacket *pkt = (const STM32MotorCommandPacket *)rx_buf;
                    last_esp_packet_ms = millis();
                    esp_link_active = true;
                    g_esp_link_active_isr = true;
                    g_esp_rx_seq_isr++;
                    have_valid_packet = true;
                    esp_pkts_ok++;

                    switch (pkt->cmd_mode) {
                        case STM32_CMD_DRIVE:
                            if (!g_estop_active && !s_calib_isolation_active) {
                                // Keep only the newest drive target from this
                                // service pass. This bounds PID-target churn
                                // when UART bytes arrive in a burst.
                                latest_drive_m1   = pkt->param1;
                                latest_drive_m2   = pkt->param2;
                                latest_m1_vmin_mv = pkt->m1_vtrq_min_mv;
                                latest_m1_vmax_mv = pkt->m1_vtrq_max_mv;
                                latest_m1_vff_mv  = pkt->m1_vtrq_ff_mv;
                                latest_m2_vmin_mv = pkt->m2_vtrq_min_mv;
                                latest_m2_vmax_mv = pkt->m2_vtrq_max_mv;
                                latest_m2_vff_mv  = pkt->m2_vtrq_ff_mv;
                                have_latest_drive = true;
                                if ((m1_enabled || m2_enabled) && pkt->param1 == 0.0f && pkt->param2 == 0.0f) {
                                    esp_drive_zero++;
                                }
                            }
                            break;

                        case STM32_CMD_ESTOP:
                            have_latest_drive = false;
                            triggerGlobalEmergencyStop();
                            break;

                        case STM32_CMD_ENABLE:
                            if (!s_calib_isolation_active) {
                                g_estop_active = false;
                                m1_ctrl.updateKe(motor1.KV_rating);
                                m2_ctrl.updateKe(motor2.KV_rating);
                                resetMotorTorqueCtrl(m1_ctrl, m1_target_rpm, motor1.shaft_velocity, true);
                                resetMotorTorqueCtrl(m2_ctrl, m2_target_rpm, motor2.shaft_velocity, true);
                                m1_enabled = true;
                                m2_enabled = true;
                                driver1.enable();
                                driver2.enable();
                                motor1.enable();
                                motor2.enable();
                            }
                            break;

                        case STM32_CMD_DISABLE:
                            if (!s_calib_isolation_active) {
                                have_latest_drive = false;
                                disableBothDrivers();
                            }
                            break;

                        case STM32_CMD_SET_PID:
                            if (!s_calib_isolation_active) {
                                if (pkt->param1 > 0.0f) {
                                    motor1.PID_velocity.P = pkt->param1;
                                    motor2.PID_velocity.P = pkt->param1;
                                }
                                if (pkt->param2 >= 0.0f) {
                                    motor1.PID_velocity.I = pkt->param2;
                                    motor2.PID_velocity.I = pkt->param2;
                                }
                            }
                            break;

                        case STM32_CMD_CALIB_M1:
                            if (!s_calib_isolation_active) {
                                runMasterCalibrationSequence(1, pkt->param1 > 0.0f ? pkt->param1 : CALIB_ALIGN_VOLTAGE);
                            }
                            break;

                        case STM32_CMD_CALIB_M2:
                            if (!s_calib_isolation_active) {
                                runMasterCalibrationSequence(2, pkt->param1 > 0.0f ? pkt->param1 : CALIB_ALIGN_VOLTAGE);
                            }
                            break;

                        case STM32_CMD_SAVE_FLASH:
                            if (!s_calib_isolation_active) {
                                saveConfigToFlash();
                            }
                            break;

                        case STM32_CMD_ENTER_DFU:
                            if (!s_calib_isolation_active) {
                                disableBothDrivers();
                                delay(20);
                                __HAL_RCC_PWR_CLK_ENABLE();
                                HAL_PWR_EnableBkUpAccess();
                                TAMP->BKP0R = 0xDEADBEEF;
                                NVIC_SystemReset();
                            }
                            break;

                        default:
                            break;
                    }
                } else {
                    esp_chk_err++;
                    // Checksum mismatch on false sync: rescan rx_buf[1..23] so real header isn't lost
                    for (uint8_t shift = 1; shift < sizeof(STM32MotorCommandPacket); shift++) {
                        if (rx_buf[shift] == STM32_CMD_SYNC_1) {
                            if (shift + 1 < sizeof(STM32MotorCommandPacket)) {
                                if (rx_buf[shift + 1] == STM32_CMD_SYNC_2) {
                                    uint8_t rem = sizeof(STM32MotorCommandPacket) - shift;
                                    memmove(rx_buf, &rx_buf[shift], rem);
                                    rx_idx = rem;
                                    sync_found = true;
                                    break;
                                }
                            } else {
                                rx_buf[0] = STM32_CMD_SYNC_1;
                                rx_idx = 1;
                                sync_found = false;
                                break;
                            }
                        }
                    }

                }
            }
        }
    }

    if (have_latest_drive && !g_estop_active && !s_calib_isolation_active) {
        setMotorCommand(m1_ctrl, m1_target_rpm, latest_drive_m1,
                        (float)latest_m1_vmin_mv * 0.001f,
                        (float)latest_m1_vmax_mv * 0.001f,
                        (float)latest_m1_vff_mv * 0.001f);
        setMotorCommand(m2_ctrl, m2_target_rpm, latest_drive_m2,
                        (float)latest_m2_vmin_mv * 0.001f,
                        (float)latest_m2_vmax_mv * 0.001f,
                        (float)latest_m2_vff_mv * 0.001f);

        // Never auto-enable disabled drivers on an idle (0, 0, 0) command.
        if (((fabsf(latest_drive_m1) > 0.01f) || (latest_m1_vmin_mv != 0) || (latest_m1_vmax_mv != 0)) && !m1_enabled) {
            m1_ctrl.updateKe(motor1.KV_rating);
            resetMotorTorqueCtrl(m1_ctrl, m1_target_rpm, motor1.shaft_velocity, false);
            m1_enabled = true;
            driver1.enable();
            motor1.enable();
        }
        if (((fabsf(latest_drive_m2) > 0.01f) || (latest_m2_vmin_mv != 0) || (latest_m2_vmax_mv != 0)) && !m2_enabled) {
            m2_ctrl.updateKe(motor2.KV_rating);
            resetMotorTorqueCtrl(m2_ctrl, m2_target_rpm, motor2.shaft_velocity, false);
            m2_enabled = true;
            driver2.enable();
            motor2.enable();
        }
    }

    // At most one reply per bounded service pass; do not create a TX burst
    // proportional to the number of queued drive commands.
    if (have_valid_packet) {
        sendESP32Feedback();
    }
}

// ----------------------------------------------------------------------------
// Arduino Setup: Autonomous Standalone Initialization (< 150 ms)
// ----------------------------------------------------------------------------
void setup() {
    // Check DFU Magic Flag in RTC/TAMP Backup Register 0
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    if (TAMP->BKP0R == 0xDEADBEEF) {
        TAMP->BKP0R = 0; // Clear flag
        __HAL_RCC_SYSCFG_CLK_ENABLE();
        __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
        const uint32_t boot_addr = 0x1FFF0000;
        __set_MSP(*(__IO uint32_t *)boot_addr);
        void (*boot_jump)(void) = (void (*)(void))(*(__IO uint32_t *)(boot_addr + 4));
        boot_jump();
        while (1) {}
    }

    // 0. Immediately clamp Driver Enable pins LOW to prevent current surge on startup
    pinMode(M1_EN, OUTPUT);
    digitalWrite(M1_EN, LOW);
    pinMode(M2_EN, OUTPUT);
    digitalWrite(M2_EN, LOW);

    // 1. Standalone DWT Unlock & Hardware CORDIC Accelerator Init
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    initHardwareCORDIC();

    // 2. Status LED setup (Disabled: LOW = OFF)
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    // 3. Serial Port Initialization (921600 Baud for Ultra High Speed Telemetry)
    Serial.begin(921600);
    SerialESP.begin(ESP_UART_BAUD);
    // Enable STM32G474 8-byte hardware RX/TX FIFO on USART1 (SerialESP) so the 20 kHz
    // Priority-0 FOC ISR never causes a 1-byte UART RX overrun (ORE) at 921,600 baud.
    __disable_irq();
    USART1->CR1 &= ~USART_CR1_UE;
    USART1->CR1 |= USART_CR1_FIFOEN;
    USART1->CR1 |= USART_CR1_UE;
    __enable_irq();

    // 4. Initialize MT6816 Encoders on Independent Hardware SPI Buses (SPI1 & SPI2)
    encoder1.init();
    encoder2.init();
    encoder1.resetErrors();
    encoder2.resetErrors();

    // 5. Configure Drivers (TIM2 & TIM3 @ 25 kHz Center-Aligned SpaceVectorPWM)
    driver1.voltage_power_supply = MOTOR_SUPPLY_VOLTAGE;
    driver1.voltage_limit = MOTOR_SUPPLY_VOLTAGE;
    driver1.pwm_frequency = 25000;
    driver1.init();

    driver2.voltage_power_supply = MOTOR_SUPPLY_VOLTAGE;
    driver2.voltage_limit = MOTOR_SUPPLY_VOLTAGE;
    driver2.pwm_frequency = 25000;
    driver2.init();

    // 6. Configure Motors (GM3506 Optimized Dynamics)
    motor1.linkDriver(&driver1);
    motor1.linkSensor(&encoder1);
    motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor1.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor1.velocity_limit = MOTOR_VELOCITY_LIMIT;
    motor1.KV_rating = MOTOR_KV_RATING;
    motor1.voltage_sensor_align = MOTOR_ALIGN_VOLTAGE;
    motor1.controller = MotionControlType::velocity;
    motor1.PID_velocity.P = DEFAULT_PID_P;
    motor1.PID_velocity.I = DEFAULT_PID_I;
    motor1.PID_velocity.D = DEFAULT_PID_D;
    motor1.PID_velocity.output_ramp = DEFAULT_PID_RAMP;
    motor1.PID_velocity.Ts = 0.0002f;
    motor1.PID_velocity.limit = motor1.voltage_limit;
    motor1.LPF_velocity.Tf = DEFAULT_LPF_TF;
    motor1.LPF_velocity.Ts = 0.0002f;
    motor1.init();
    motor1.disable();
    driver1.disable();
    digitalWrite(M1_EN, LOW);

    motor2.linkDriver(&driver2);
    motor2.linkSensor(&encoder2);
    motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor2.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor2.velocity_limit = MOTOR_VELOCITY_LIMIT;
    motor2.KV_rating = MOTOR_KV_RATING;
    motor2.voltage_sensor_align = MOTOR_ALIGN_VOLTAGE;
    motor2.controller = MotionControlType::velocity;
    motor2.PID_velocity.P = DEFAULT_PID_P;
    motor2.PID_velocity.I = DEFAULT_PID_I;
    motor2.PID_velocity.D = DEFAULT_PID_D;
    motor2.PID_velocity.output_ramp = DEFAULT_PID_RAMP;
    motor2.PID_velocity.Ts = 0.0002f;
    motor2.PID_velocity.limit = motor2.voltage_limit;
    motor2.LPF_velocity.Tf = DEFAULT_LPF_TF;
    motor2.LPF_velocity.Ts = 0.0002f;
    motor2.init();
    motor2.disable();
    driver2.disable();
    digitalWrite(M2_EN, LOW);

    // 7. Silent Pre-Loaded Alignment Check (Auto-load from Flash if saved, else fallback to constants)
    if (loadConfigFromFlash()) {
        Serial.println(">>> [STARTUP] Calibration & LUT profile loaded from Flash.");
    } else {
        if (M1_DEFAULT_ZERO_ELECTRIC_ANGLE != 0.0f) {
            motor1.zero_electric_angle = M1_DEFAULT_ZERO_ELECTRIC_ANGLE;
            m1_base_zero_angle = M1_DEFAULT_ZERO_ELECTRIC_ANGLE;
            motor1.sensor_direction = Direction::CW;
            m1_calibrated = true;
        }

        if (M2_DEFAULT_ZERO_ELECTRIC_ANGLE != 0.0f) {
            motor2.zero_electric_angle = M2_DEFAULT_ZERO_ELECTRIC_ANGLE;
            m2_base_zero_angle = M2_DEFAULT_ZERO_ELECTRIC_ANGLE;
            motor2.sensor_direction = Direction::CW;
            m2_calibrated = true;
        }
    }

    m1_ctrl.updateKe(motor1.KV_rating);
    m2_ctrl.updateKe(motor2.KV_rating);

    // Ensure both motors and gate drivers start disabled by default
    disableBothDrivers();

    // 8. Configure & Start TIM6 Hardware Timer Interrupt @ 20.0 kHz (Highest Priority 0)
    foc_timer = new HardwareTimer(TIM6);
    foc_timer->setOverflow(FOC_COMMUTATION_HZ, HERTZ_FORMAT);
    foc_timer->attachInterrupt(focTimerISR);
    foc_timer->setInterruptPriority(0, 0);
    foc_timer->resume();

    last_telemetry_time = millis();
    last_led_time = millis();

    printHelp();
    iwdgInit500ms();
}

// ----------------------------------------------------------------------------
// Background Task Loop (Serial Parsing, Telemetry, Safety)
// ----------------------------------------------------------------------------
void loop() {
    iwdgKick();
    uint32_t now = millis();

    // 0. High-Speed 400 Hz Bidirectional Binary Link to ESP32
    processESP32Link();

    // Refresh timestamp after processESP32Link() so last_esp_packet_ms cannot be ahead of now
    // (which would otherwise underflow unsigned subtraction to 0xFFFFFFFF and trip the watchdog).
    now = millis();

    // 0b. Safety Watchdog: If no ESP32 packet received for > 100 ms while motors armed, disarm motors
    if (esp_link_active && now >= last_esp_packet_ms) {
        uint32_t gap = now - last_esp_packet_ms;
        if (gap > esp_max_gap_ms) esp_max_gap_ms = gap;
    }
    if (esp_link_active && (m1_enabled || m2_enabled) &&
        (now >= last_esp_packet_ms) && (now - last_esp_packet_ms > 100)) {
        esp_wdog_trips++;
        g_estop_active = true;
        disableBothDrivers();
    }

    // 1. Process Incoming Serial Characters into Line Buffer
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        // Instant global emergency stop key (ESC = ASCII 27) without requiring newline.
        // Note: 'x'/'X' is handled as a standalone command in processCommand() so words like
        // 'export' or 'MAXRPM1' do not falsely trigger E-STOP mid-word!
        if (c == 27) {
            triggerGlobalEmergencyStop();
            rx_idx = 0;
            continue;
        }
        if (c == '\n' || c == '\r') {
            if (rx_idx > 0) {
                rx_line[rx_idx] = '\0';
                processCommand(rx_line);
                rx_idx = 0;
            }
        } else if (rx_idx < sizeof(rx_line) - 1) {
            rx_line[rx_idx++] = c;
        }
    }

    // 2. Autonomous Stall Protection
    checkStallWatchdog();

    // 3. Periodic Telemetry Stream (25 Hz = 40 ms) - Active when ESP32 link is not running
    if (!esp_link_active && show_telemetry && (now - last_telemetry_time >= (1000 / TELEMETRY_RATE_HZ))) {
        last_telemetry_time = now;
        sendBinaryTelemetry();
    }

    // 4. Status LED (Disabled on PA8: LOW = OFF)
    digitalWrite(STATUS_LED, LOW);
}

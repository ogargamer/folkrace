#include <Arduino.h>
#include <SimpleFOC.h>
#include "config_pins.h"

// ============================================================================
// STM32G474 — EXACT PRODUCTION DRIVER REAL-TIME PWM & ENABLE TEST BENCH
// ============================================================================
// Uses the EXACT driver initialization as production stm32_dual_foc:
//   driver1: BLDCDriver3PWM(M1_PWM_A, M1_PWM_B, M1_PWM_C, M1_EN) -> TIM2 (PA5, PA1, PA2) + PA3
//   driver2: BLDCDriver3PWM(M2_PWM_A, M2_PWM_B, M2_PWM_C, M2_EN) -> TIM3 (PA6, PA4, PB0) + PA7
//
// Voltage power supply: 12.0V, PWM frequency: 25000 Hz Center-Aligned SpaceVector PWM.
// No sensor or FOC loop dependency - safe for multimeter measurements with PSU OFF!
// ============================================================================

BLDCDriver3PWM driver1(M1_PWM_A, M1_PWM_B, M1_PWM_C, M1_EN);
BLDCDriver3PWM driver2(M2_PWM_A, M2_PWM_B, M2_PWM_C, M2_EN);

static bool d1_en = true;
static bool d2_en = true;
static float d1_v = 6.0f; // 50% duty
static float d2_v = 6.0f; // 50% duty

static void jump_to_dfu() {
    Serial.println("\n>>> [DFU] JUMPING TO STM32 SYSTEM DFU BOOTLOADER...");
    Serial.flush();
    delay(50);
    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }
    const uint32_t boot_addr = 0x1FFF0000;
    uint32_t jump_addr = *(__IO uint32_t *)(boot_addr + 4);
    void (*boot_jump)(void) = (void (*)(void))jump_addr;
    __set_MSP(*(__IO uint32_t *)boot_addr);
    boot_jump();
    while (1);
}

static void apply_driver_states() {
    if (d1_en) {
        driver1.enable();
        // phase A (PA5) = 3.0V (25%), phase B (PA1) = 6.0V (50%), phase C (PA2) = 9.0V (75%)
        driver1.setPwm(3.0f, 6.0f, 9.0f);
    } else {
        driver1.disable();
    }

    if (d2_en) {
        driver2.enable();
        // phase A (PA6) = 3.0V (25%), phase B (PA4) = 6.0V (50%), phase C (PB0) = 9.0V (75%)
        driver2.setPwm(3.0f, 6.0f, 9.0f);
    } else {
        driver2.disable();
    }
}

static void print_pins() {
    Serial.println("\n====================================================================");
    Serial.println("  STM32 EXACT PRODUCTION DRIVER REAL-TIME PWM & ENABLE TEST BENCH   ");
    Serial.println("====================================================================");
    Serial.println("MOTOR 1 INVERTER (TIM2 @ 25 kHz Center-Aligned PWM):");
    Serial.println("  * PA3 (M1_EN)    : Pulses HIGH (3.3V) / LOW (0.0V) every 1s");
    Serial.println("  * PA5 (M1_PWM_A) : TIM2_CH1 PWM (Duty  25% -> 25 kHz, ~0.825V DC)");
    Serial.println("  * PA1 (M1_PWM_B) : TIM2_CH2 PWM (Duty  50% -> 25 kHz, ~1.650V DC)");
    Serial.println("  * PA2 (M1_PWM_C) : TIM2_CH3 PWM (Duty  75% -> 25 kHz, ~2.475V DC)");
    Serial.println("MOTOR 2 INVERTER (TIM3 @ 25 kHz Center-Aligned PWM):");
    Serial.println("  * PA7 (M2_EN)    : Pulses HIGH (3.3V) / LOW (0.0V) every 1s");
    Serial.println("  * PA6 (M2_PWM_A) : TIM3_CH1 PWM (Duty  25% -> 25 kHz, ~0.825V DC)");
    Serial.println("  * PA4 (M2_PWM_B) : TIM3_CH2 PWM (Duty  50% -> 25 kHz, ~1.650V DC)");
    Serial.println("  * PB0 (M2_PWM_C) : TIM3_CH3 PWM (Duty  75% -> 25 kHz, ~2.475V DC)");
    Serial.println("STATUS LED (PA8): OFF (No blink)");
    Serial.println("--------------------------------------------------------------------");
    Serial.println("Interactive Commands:");
    Serial.println("  e1 / d1 : Enable / Disable Motor 1 driver (PA3)");
    Serial.println("  e2 / d2 : Enable / Disable Motor 2 driver (PA7)");
    Serial.println("  e  / d  : Enable / Disable BOTH drivers");
    Serial.println("  h  / ?  : Print this pinout guide");
    Serial.println("  DFU     : Jump to STM32 DFU bootloader");
    Serial.println("====================================================================\n");
}

void setup() {
    // 1. LED off (PA8 LOW)
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    // 2. Exact production driver setup
    driver1.voltage_power_supply = MOTOR_SUPPLY_VOLTAGE;
    driver1.voltage_limit = MOTOR_SUPPLY_VOLTAGE;
    driver1.pwm_frequency = 25000;
    driver1.init();

    driver2.voltage_power_supply = MOTOR_SUPPLY_VOLTAGE;
    driver2.voltage_limit = MOTOR_SUPPLY_VOLTAGE;
    driver2.pwm_frequency = 25000;
    driver2.init();

    // 3. Enable both and output test PWMs
    d1_en = true;
    d2_en = true;
    apply_driver_states();

    // 4. USB CDC Serial
    Serial.begin(921600);
}

void loop() {
    static uint32_t last_toggle = 0;
    static uint32_t last_report = 0;
    static char rx_buf[32];
    static uint8_t rx_idx = 0;
    uint32_t now = millis();

    // Pulse enable pins HIGH / LOW every 1 second
    if (now - last_toggle >= 1000) {
        last_toggle = now;
        d1_en = !d1_en;
        d2_en = !d2_en;
        apply_driver_states();
    }

    // Continuous refresh of PWM duty cycles when enabled
    if (d1_en) driver1.setPwm(3.0f, 6.0f, 9.0f);
    if (d2_en) driver2.setPwm(3.0f, 6.0f, 9.0f);

    // Periodic reminder every 3 seconds
    if (now - last_report >= 3000) {
        last_report = now;
        if (Serial) {
            Serial.printf("[DMM TEST] M1_EN(PA3)=%s | M2_EN(PA7)=%s | M2_PWM: PA6=0.83V PA4=1.65V PB0=2.48V\n",
                          d1_en ? "3.3V" : "0V", d2_en ? "3.3V" : "0V");
        }
    }

    // Process serial commands
    while (Serial && Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            if (rx_idx > 0) {
                rx_buf[rx_idx] = '\0';
                if (strcasecmp(rx_buf, "DFU") == 0) {
                    jump_to_dfu();
                } else if (strcasecmp(rx_buf, "e1") == 0) {
                    d1_en = true;
                    apply_driver_states();
                    Serial.println(">>> M1 Driver ENABLED (PA3 = HIGH)");
                } else if (strcasecmp(rx_buf, "d1") == 0) {
                    d1_en = false;
                    apply_driver_states();
                    Serial.println(">>> M1 Driver DISABLED (PA3 = LOW)");
                } else if (strcasecmp(rx_buf, "e2") == 0) {
                    d2_en = true;
                    apply_driver_states();
                    Serial.println(">>> M2 Driver ENABLED (PA7 = HIGH)");
                } else if (strcasecmp(rx_buf, "d2") == 0) {
                    d2_en = false;
                    apply_driver_states();
                    Serial.println(">>> M2 Driver DISABLED (PA7 = LOW)");
                } else if (strcasecmp(rx_buf, "e") == 0) {
                    d1_en = true;
                    d2_en = true;
                    apply_driver_states();
                    Serial.println(">>> BOTH Drivers ENABLED");
                } else if (strcasecmp(rx_buf, "d") == 0) {
                    d1_en = false;
                    d2_en = false;
                    apply_driver_states();
                    Serial.println(">>> BOTH Drivers DISABLED");
                } else if (strcasecmp(rx_buf, "h") == 0 || strcmp(rx_buf, "?") == 0) {
                    print_pins();
                }
                rx_idx = 0;
            }
        } else if (rx_idx < sizeof(rx_buf) - 1) {
            rx_buf[rx_idx++] = c;
        }
    }
}

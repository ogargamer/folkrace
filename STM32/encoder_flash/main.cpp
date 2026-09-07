#include <Arduino.h>
#include <Wire.h>

#define ENC_SDA PB9
#define ENC_SCL PB8
#define STATUS_LED PC13

const uint8_t MT6701_ADDR = 0x06;

void setup() {
    pinMode(ENC_SDA, INPUT_PULLUP);
    pinMode(ENC_SCL, INPUT_PULLUP);

    Wire.setSDA(ENC_SDA);
    Wire.setSCL(ENC_SCL);
    Wire.begin();
    Wire.setClock(50000); // 50kHz Low-Speed I2C for Maximum Reliability

    Serial.begin(921600);
    Serial.setTimeout(30);

    uint32_t start_time = millis();
    while (!Serial && (millis() - start_time < 3000)) {
        digitalWrite(STATUS_LED, (millis() % 200 < 100) ? LOW : HIGH);
    }
    digitalWrite(STATUS_LED, HIGH);

    Serial.println("\n==================================================");
    Serial.println("  MT6701 Dedicated I2C Flasher (PB9 SDA / PB8 SCL)");
    Serial.println("==================================================");
    Serial.println("Pin Wiring:");
    Serial.println("  Pin 1 (VDD)  -> 5V");
    Serial.println("  Pin 2 (MODE) -> 5V (VDD)");
    Serial.println("  Pin 4 (GND)  -> GND");
    Serial.println("  Pin 6 (SDA)  -> PB9");
    Serial.println("  Pin 7 (SCL)  -> PB8");
    Serial.println("==================================================\n");
}

void loop() {
    static uint32_t last_check = 0;
    if (millis() - last_check >= 1500) {
        last_check = millis();

        Serial.print("[INFO] Pinging MT6701 at 0x06 (SDA=PB9, SCL=PB8)... ");
        Wire.beginTransmission(MT6701_ADDR);
        byte err = Wire.endTransmission();

        if (err == 0) {
            Serial.println("SUCCESS! MT6701 Responding!");
            
            Wire.beginTransmission(MT6701_ADDR);
            Wire.write(0x00);
            Wire.endTransmission(false); // Repeated start

            uint8_t read_bytes = Wire.requestFrom(MT6701_ADDR, (uint8_t)16);
            if (read_bytes >= 16) {
                byte regs[16];
                for (int i = 0; i < 16; i++) {
                    regs[i] = Wire.read();
                }

                Serial.println("--- Register Dump ---");
                for (int i = 0; i < 16; i++) {
                    Serial.print(" Reg 0x");
                    if (i < 10) Serial.print("0");
                    Serial.print(i, HEX);
                    Serial.print(" : 0x");
                    if (regs[i] < 16) Serial.print("0");
                    Serial.print(regs[i], HEX);
                    Serial.print("  [BIN: ");
                    for (int b = 7; b >= 0; b--) {
                        Serial.print((regs[i] >> b) & 1);
                    }
                    Serial.println("]");
                }
                uint16_t raw_angle = ((regs[3] & 0xFF) << 6) | ((regs[4] & 0xFC) >> 2);
                float angle_deg = (raw_angle / 16384.0f) * 360.0f;
                Serial.print("Angle: "); Serial.print(angle_deg, 2); Serial.println(" deg");
                Serial.print("Reg 0x05 (ABZ_RES): 0x"); Serial.println(regs[5], HEX);
                Serial.print("Reg 0x06 (CONFIG) : 0x"); Serial.println(regs[6], HEX);
                Serial.println("---------------------\n");
            }
        } else {
            Serial.print("FAILED (Error Code "); Serial.print(err); Serial.println(")");
        }
    }

    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == '1') {
            Serial.println("\n>>> Writing 1024 PPR (Reg 0x05=0x00) & ABZ Mode (Reg 0x06=0x00)...");
            Wire.beginTransmission(MT6701_ADDR); Wire.write(0x05); Wire.write(0x00); Wire.endTransmission(); delay(20);
            Wire.beginTransmission(MT6701_ADDR); Wire.write(0x06); Wire.write(0x00); Wire.endTransmission(); delay(20);
            Serial.println(">>> Written to SRAM! Type 'f' to Flash OTP.");
        } else if (cmd == 'f' || cmd == 'F') {
            Serial.println("\n>>> Burning OTP Key 0x25 to Reg 0x0E at 5V...");
            Wire.beginTransmission(MT6701_ADDR); Wire.write(0x0E); Wire.write(0x25); byte ferr = Wire.endTransmission(); delay(200);
            if (ferr == 0) Serial.println(">>> FLASH SUCCESSFUL!");
            else { Serial.print(">>> FLASH FAILED (Error "); Serial.print(ferr); Serial.println(")"); }
        }
    }
}

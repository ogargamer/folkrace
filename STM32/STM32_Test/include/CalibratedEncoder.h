#ifndef CALIBRATED_ENCODER_H
#define CALIBRATED_ENCODER_H

#include <Arduino.h>
#include <SimpleFOC.h>

#define LUT_SIZE 128

class CalibratedEncoder : public Sensor {
public:
    Encoder* raw_encoder;
    float lut[LUT_SIZE];
    bool is_calibrated = false;

    CalibratedEncoder(Encoder* enc) : raw_encoder(enc) {
        memset(lut, 0, sizeof(lut));
    }

    void init() override {
        raw_encoder->init();
    }

    void update() override {
        raw_encoder->update();
    }

    float getAngle() override {
        return raw_encoder->getAngle() + getLutOffset();
    }

    float getSensorAngle() override {
        return raw_encoder->getSensorAngle() + getLutOffset();
    }

    float getVelocity() override {
        return raw_encoder->getVelocity();
    }

    float getMechanicalAngle() override {
        return raw_encoder->getMechanicalAngle();
    }

    float getLutOffset() {
        if (!is_calibrated) return 0.0f;
        float raw_mech = raw_encoder->getMechanicalAngle();
        float bin_float = (raw_mech / _2PI) * LUT_SIZE;
        int idx1 = (int)bin_float;
        if (idx1 >= LUT_SIZE) idx1 = 0;
        if (idx1 < 0) idx1 = 0;
        int idx2 = (idx1 + 1) % LUT_SIZE;
        float frac = bin_float - (float)idx1;
        return lut[idx1] * (1.0f - frac) + lut[idx2] * frac;
    }

    // Auto-calibration in open-loop mode
    void runCalibration(BLDCMotor& motor) {
        Serial.println("\n==================================================");
        Serial.println("  Starting Encoder Non-Linearity LUT Calibration  ");
        Serial.println("==================================================");
        Serial.println("Motor moving in Open-Loop at 10 RPM (~1 rad/s)...");

        memset(lut, 0, sizeof(lut));

        // Save state & switch to open-loop
        auto old_mode = motor.controller;
        float old_voltage = motor.voltage_limit;
        motor.controller = MotionControlType::velocity_openloop;
        motor.voltage_limit = 3.0f; // Safe open-loop voltage
        motor.enable();

        float open_loop_speed = 1.047f; // ~10 RPM
        
        // 1. Stabilize motor motion
        uint32_t t_start = millis();
        while (millis() - t_start < 2000) {
            motor.move(open_loop_speed);
            raw_encoder->update();
            delay(1);
        }

        // 2. Measure angle error over 1 full rotation (6.0 seconds)
        float start_angle = raw_encoder->getMechanicalAngle();
        uint32_t calib_start = millis();
        int samples[LUT_SIZE] = {0};
        float error_sum[LUT_SIZE] = {0};

        while (millis() - calib_start < 6000) {
            motor.move(open_loop_speed);
            raw_encoder->update();

            float elapsed_s = (millis() - calib_start) / 1000.0f;
            float expected_mech = fmod(start_angle + elapsed_s * open_loop_speed, _2PI);
            if (expected_mech < 0) expected_mech += _2PI;

            float actual_mech = raw_encoder->getMechanicalAngle();

            float err = expected_mech - actual_mech;
            // Wrap error to [-PI, PI]
            while (err > _PI) err -= _2PI;
            while (err < -_PI) err += _2PI;

            int bin = (int)((actual_mech / _2PI) * LUT_SIZE);
            if (bin >= 0 && bin < LUT_SIZE) {
                error_sum[bin] += err;
                samples[bin]++;
            }
            delay(1);
        }

        // 3. Compute smooth average offset per bin
        for (int i = 0; i < LUT_SIZE; i++) {
            if (samples[i] > 0) {
                lut[i] = error_sum[i] / samples[i];
            } else {
                lut[i] = 0.0f;
            }
        }

        // Stop & restore motor configuration
        motor.move(0);
        motor.voltage_limit = old_voltage;
        motor.controller = old_mode;
        motor.disable();

        is_calibrated = true;
        Serial.println("LUT Calibration COMPLETE! 128-point correction active.");
        Serial.println("==================================================\n");
    }
};

#endif // CALIBRATED_ENCODER_H

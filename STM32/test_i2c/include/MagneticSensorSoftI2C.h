#ifndef MAGNETIC_SENSOR_SOFT_I2C_H
#define MAGNETIC_SENSOR_SOFT_I2C_H

#include <Arduino.h>
#include <SoftWire.h>
#include <SimpleFOC.h>

class MagneticSensorSoftI2C : public Sensor {
  public:
    float cpr;
    
    MagneticSensorSoftI2C(SoftWire& sw, MagneticSensorI2CConfig_s config) 
      : _sw(sw), _conf(config), last_read_us(0), cached_raw(0), _responding(false) {
        cpr = 16384.0f; // 14-bit resolution for MT6701
    }

    void init() override {
        this->Sensor::init();
    }

    void setAddress(uint8_t addr) {
        _conf.chip_address = addr;
    }

    uint8_t getAddress() const {
        return _conf.chip_address;
    }

    bool isResponding() {
        _sw.beginTransmission(_conf.chip_address);
        _responding = (_sw.endTransmission() == 0);
        return _responding;
    }

    float getSensorAngle() override {
        return (getRawCount() / (float)cpr) * _2PI;
    }

    int getRawCount() {
        uint32_t now = micros();
        // Limit Software I2C reads to a maximum of 2kHz (every 500 microseconds)
        if (now - last_read_us >= 500 || last_read_us == 0) {
            last_read_us = now;
            
            byte readArray[2] = {0, 0};
            _sw.beginTransmission(_conf.chip_address);
            _sw.write(_conf.angle_register);
            uint8_t err = _sw.endTransmission();

            if (err == 0) {
                uint8_t count = _sw.requestFrom(_conf.chip_address, (uint8_t)2);
                for (byte i = 0; i < 2; i++) {
                    if (_sw.available()) {
                        readArray[i] = _sw.read();
                    }
                }
                if (count > 0) {
                    _responding = true;
                    uint16_t readValue = (readArray[0] & _conf.msb_mask) << _conf.msb_shift;
                    readValue |= (readArray[1] & _conf.lsb_mask) >> _conf.lsb_shift;
                    cached_raw = readValue;
                }
            } else {
                _responding = false;
            }
        }
        return cached_raw;
    }

  private:
    SoftWire& _sw;
    MagneticSensorI2CConfig_s _conf;
    uint32_t last_read_us;
    int cached_raw;
    bool _responding;
};

#endif // MAGNETIC_SENSOR_SOFT_I2C_H

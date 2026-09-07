#ifndef HARDWARE_TIMER_ENCODER_H
#define HARDWARE_TIMER_ENCODER_H

#include <Arduino.h>
#include <SimpleFOC.h>

class HardwareTimerEncoder : public Sensor {
public:
    TIM_TypeDef* _TIMx;
    HardwareTimer* _ht;
    uint32_t cpr;
    
    int32_t count_accumulator = 0;
    uint16_t last_raw_cnt = 0;
    uint32_t last_update_us = 0;
    float velocity = 0.0f;

    HardwareTimerEncoder(TIM_TypeDef* TIMx, uint32_t ppr) : _TIMx(TIMx), cpr(ppr * 4) {}

    void init() override {
        // Enable peripheral clock for TIM3
        if (_TIMx == TIM3) {
            __HAL_RCC_TIM3_CLK_ENABLE();
            __HAL_RCC_GPIOB_CLK_ENABLE();

            // Configure PB4 (TIM3_CH1) and PB5 (TIM3_CH2) in Alternate Function mode (AF2 for TIM3 on STM32G4)
            GPIO_InitTypeDef GPIO_InitStruct = {0};
            GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
            GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
            GPIO_InitStruct.Pull = GPIO_PULLUP;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
            GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        }

        // Configure TIM3 in Encoder Interface Mode (Counting on both TI1 and TI2)
        TIM_Encoder_InitTypeDef encoderConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};

        _ht = new HardwareTimer(_TIMx);
        _ht->setMode(1, TIMER_DISABLED);
        _ht->setMode(2, TIMER_DISABLED);

        TIM_HandleTypeDef* htim = _ht->getHandle();
        htim->Instance = _TIMx;
        htim->Init.Prescaler = 0;
        htim->Init.CounterMode = TIM_COUNTERMODE_UP;
        htim->Init.Period = 0xFFFF; // 16-bit auto-reload
        htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

        encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12; // Both edges of A & B
        encoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        encoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        encoderConfig.IC1Filter = 4; // Hardware glitch filter

        encoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        encoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        encoderConfig.IC2Filter = 4; // Hardware glitch filter

        HAL_TIM_Encoder_Init(htim, &encoderConfig);
        HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);

        last_raw_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
        last_update_us = micros();
    }

    void update() override {
        uint16_t current_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(_ht->getHandle());
        int16_t delta = (int16_t)(current_cnt - last_raw_cnt); // Handles 16-bit overflow/underflow automatically
        last_raw_cnt = current_cnt;
        count_accumulator += delta;

        uint32_t now_us = micros();
        float dt = (now_us - last_update_us) / 1000000.0f;
        if (dt >= 0.001f) { // 1 kHz velocity update
            float delta_rad = (delta / (float)cpr) * _2PI;
            velocity = delta_rad / dt;
            last_update_us = now_us;
        }
    }

    float getSensorAngle() override {
        return (count_accumulator / (float)cpr) * _2PI;
    }

    float getMechanicalAngle() override {
        uint16_t current_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(_ht->getHandle());
        uint32_t pos_in_rev = current_cnt % cpr;
        return (pos_in_rev / (float)cpr) * _2PI;
    }

    float getAngle() override {
        return getSensorAngle();
    }

    float getVelocity() override {
        return velocity;
    }
};

#endif // HARDWARE_TIMER_ENCODER_H

#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include <stdbool.h>

#include <mcl/mcl.h>

typedef enum {
    ADC_RES_12BIT = 0,
    ADC_RES_10BIT,
    ADC_RES_8BIT,
    ADC_RES_6BIT
} ADC_Resolution_t;

typedef enum {
    ADC_ALIGN_RIGHT = 0,
    ADC_ALIGN_LEFT
} ADC_Alignment_t;

typedef enum {
    ADC_SAMPLE_3 = 0,
    ADC_SAMPLE_15,
    ADC_SAMPLE_28,
    ADC_SAMPLE_56,
    ADC_SAMPLE_84,
    ADC_SAMPLE_112,
    ADC_SAMPLE_144,
    ADC_SAMPLE_480
} ADC_SampleTime_t;

typedef enum {
    ADC_CH0 = 0,
    ADC_CH1,
    ADC_CH2,
    ADC_CH3,
    ADC_CH4,
    ADC_CH5,
    ADC_CH6,
    ADC_CH7,
    ADC_CH8,
    ADC_CH9,
    ADC_CH10,
    ADC_CH11,
    ADC_CH12,
    ADC_CH13,
    ADC_CH14,
    ADC_CH15
} ADC_Channel_t;

typedef struct {
    ADC_TypeDef * _instance;
    ADC_Channel_t _channel;
    ADC_Alignment_t _alignment;
    ADC_Resolution_t _resolution;
    ADC_SampleTime_t _sampleTime;
} ADC_Config_t;

static inline void adc_init(const ADC_Config_t *cfg) {
    // Enable clock for ADC peripheral
    mcl::enableClockForAdc(cfg->_instance);
    // ADC off
    cfg->_instance->CR2 = 0;
    // Resolution
    cfg->_instance->CR1 &= ~(3UL << 24);
    cfg->_instance->CR1 |= ((uint32_t)cfg->_resolution << 24);
    // Alignment
    if(cfg->_alignment == ADC_ALIGN_LEFT) {
        cfg->_instance->CR2 |= ADC_CR2_ALIGN;
    } else {
        cfg->_instance->CR2 &= ~ADC_CR2_ALIGN;
    }
    // one conversion
    cfg->_instance->SQR1 = 0;
    // channel
    cfg->_instance->SQR3 = cfg->_channel;
    // sample time
    if(cfg->_channel <= 9) {
        cfg->_instance->SMPR2 &= ~(7UL << (cfg->_channel * 3));
        cfg->_instance->SMPR2 |= ((uint32_t)cfg->_sampleTime << (cfg->_channel * 3));
    } else {
        uint32_t shift = (cfg->_channel - 10) * 3;
        cfg->_instance->SMPR1 &= ~(7UL << shift);
        cfg->_instance->SMPR1 |= ((uint32_t)cfg->_sampleTime << shift);
    }
    // enable ADC
    cfg->_instance->CR2 |= ADC_CR2_ADON;
}

static inline void adc_start(const ADC_Config_t *cfg) {
    cfg->_instance->CR2 |= ADC_CR2_SWSTART;
}

static inline bool adc_is_complete(const ADC_Config_t *cfg) {
    return (cfg->_instance->SR & ADC_SR_EOC);
}

static inline uint16_t adc_read_data(const ADC_Config_t *cfg) {
    return (uint16_t)cfg->_instance->DR;
}

static inline uint16_t adc_read(const ADC_Config_t *cfg) {
    adc_start(cfg);
    while(!adc_is_complete(cfg));
    return adc_read_data(cfg);
}

#endif
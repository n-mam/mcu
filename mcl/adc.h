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

#if defined (STM32F446xx)
typedef enum {
    ADC_TRIGGER_TIM1_CC1   = 0,
    ADC_TRIGGER_TIM1_CC2   = 1,
    ADC_TRIGGER_TIM1_CC3   = 2,
    ADC_TRIGGER_TIM2_CC2   = 3,
    ADC_TRIGGER_TIM2_CC3   = 4,
    ADC_TRIGGER_TIM2_CC4   = 5,
    ADC_TRIGGER_TIM2_TRGO  = 6,
    ADC_TRIGGER_TIM3_TRGO  = 7,
    ADC_TRIGGER_TIM4_CC4   = 8,
    ADC_TRIGGER_TIM5_CC1   = 9,
    ADC_TRIGGER_TIM5_CC2   = 10,
    ADC_TRIGGER_TIM5_CC3   = 11,
    ADC_TRIGGER_TIM8_CC1   = 12,
    ADC_TRIGGER_TIM8_TRGO   = 13,
    ADC_TRIGGER_EXTI11     = 14,
    ADC_TRIGGER_SOFTWARE   = 15
} ADC_ExternalTrigger_t;
#elif defined (STM32F411xE)
typedef enum {
    ADC_TRIGGER_TIM1_CC1  = 0,
    ADC_TRIGGER_TIM1_CC2  = 1,
    ADC_TRIGGER_TIM1_CC3  = 2,
    ADC_TRIGGER_TIM2_CC2  = 3,
    ADC_TRIGGER_TIM2_CC3  = 4,
    ADC_TRIGGER_TIM2_CC4  = 5,
    ADC_TRIGGER_TIM2_TRGO = 6,
    ADC_TRIGGER_TIM3_CH1  = 7,
    ADC_TRIGGER_TIM3_TRGO = 8,
    ADC_TRIGGER_TIM4_CC4  = 9,
    ADC_TRIGGER_TIM5_CC1  = 10,
    ADC_TRIGGER_TIM5_CC2  = 11,
    ADC_TRIGGER_TIM5_CC3  = 12,
    ADC_TRIGGER_EXTI11    = 13,
    ADC_TRIGGER_SOFTWARE  = 15
} ADC_ExternalTrigger_t;
#endif

typedef void (*adc_interrupt_callback_t)(int);
adc_interrupt_callback_t adc_interrupt_callback;

typedef struct {
    ADC_TypeDef * _instance;
    ADC_Channel_t _channel;
    ADC_Alignment_t _alignment;
    ADC_Resolution_t _resolution;
    ADC_SampleTime_t _sampleTime;
    ADC_ExternalTrigger_t _externalTrigger;   // EXTSEL value
    adc_interrupt_callback_t _interrupt_callback;
} ADC_Config_t;

static inline void adc_global_init() {
    static bool initialized = false;
    if (initialized) return;
    initialized = true;
    // Clear ADCPRE
    ADC->CCR &= ~(3UL << 16);
    // ADC prescaler
    // Divide by 4 which implies:
    // 90MHz / 4 = 22.5MHz ADC clock
    ADC->CCR |=  (1UL << 16);
}

static inline void adc_init(const ADC_Config_t *cfg) {
    // ADC off
    cfg->_instance->CR2 = 0;
    // Resolution
    cfg->_instance->CR1 &= ~(3UL << 24);
    cfg->_instance->CR1 |= ((uint32_t)cfg->_resolution << 24);
    // ADC interrupt on overruns
    cfg->_instance->CR1 |= ADC_CR1_OVRIE;
    // ADC overrun interrupt callback
    adc_interrupt_callback = cfg->_interrupt_callback;
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
    // External trigger configuration
    cfg->_instance->CR2 &= ~(ADC_CR2_EXTSEL | ADC_CR2_EXTEN);
    // Select external trigger source
    if (cfg->_externalTrigger != ADC_TRIGGER_SOFTWARE) {
        cfg->_instance->CR2 |=
            ((uint32_t)cfg->_externalTrigger << ADC_CR2_EXTSEL_Pos);
        // Enable rising-edge trigger
        cfg->_instance->CR2 |= ADC_CR2_EXTEN_0;
    }
    // Disable continuous mode
    cfg->_instance->CR2 &= ~ADC_CR2_CONT;
    // Enable DMA transfer with this ADC
    cfg->_instance->CR2 |= ADC_CR2_DMA;
    // Keep DMA requests flowing after every conversion.
    cfg->_instance->CR2 |= ADC_CR2_DDS;
    // EOC after each conversion
    cfg->_instance->CR2 |= ADC_CR2_EOCS;
    // clear any pending ADC flags
    cfg->_instance->SR = 0;
    // Enable ADC IRQ in NVIC
    NVIC_EnableIRQ(ADC_IRQn);
    // start pending
}

static inline void adc_enable(const ADC_Config_t *cfg) {
    // Clear ADC status flags (especially OVR)
    cfg->_instance->SR = 0;
    // Enable ADC
    cfg->_instance->CR2 |= ADC_CR2_ADON;
    // Wait briefly
    mcl::delay_us(10);
}

static inline bool adc_is_complete(const ADC_Config_t *cfg) {
    return (cfg->_instance->SR & ADC_SR_EOC);
}

static inline void adc_start_conversion(const ADC_Config_t *cfg) {
    cfg->_instance->CR2 |= ADC_CR2_SWSTART;
}

static inline uint16_t adc_software_read(const ADC_Config_t *cfg) {
    // clear old conversion flag
    cfg->_instance->SR &= ~ADC_SR_EOC;
    adc_start_conversion(cfg);
    while(!adc_is_complete(cfg));
    return (uint16_t)cfg->_instance->DR;
}

extern "C" {
    void ADC_IRQHandler(void) {
        if (ADC1->SR & ADC_SR_OVR) {
            ADC1->SR &= ~ADC_SR_OVR;
            adc_interrupt_callback(0);
        }
    }
}

#endif
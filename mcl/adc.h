#ifndef ADC_H
#define ADC_H

#include <vector>
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

typedef void (*adc_regular_callback_t)(int);
typedef void (*adc_injected_callback_t)(uint16_t, uint16_t, void *);

typedef struct {
    void *ctx;
    ADC_TypeDef *instance;
    ADC_Channel_t channel;
    ADC_Alignment_t alignment;
    ADC_Resolution_t resolution;
    ADC_SampleTime_t sampleTime;
    ADC_ExternalTrigger_t externalTrigger;
    adc_regular_callback_t interrupt_callback_regular;
    adc_injected_callback_t interrupt_callback_injected;
} ADC_Config_t;

inline const ADC_Config_t *adc_cfg_table[3] = {nullptr};

static inline int adc_get_index(const ADC_Config_t *cfg) {
    if (cfg->instance == ADC1) {
        return 0;
    }
    #if defined(ADC2)
    if (cfg->instance == ADC2) {
        return 1;
    }
    #endif
    #if defined(ADC3)
    if (cfg->instance == ADC3) {
        return 2;
    }
    #endif
    return -1;
}

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

static inline void adc_gpio_init(GPIO_TypeDef *GPIOx, const std::vector<uint8_t>& pins) {
    mcl::enableClockForGpio(GPIOx);
    for (size_t i = 0; i < pins.size(); ++i) {
        uint32_t shift = static_cast<uint32_t>(pins[i]) * 2U;
        // Analog mode
        GPIOx->MODER &= ~(3UL << shift);
        GPIOx->MODER |=  (3UL << shift);
        // No pull-up/pull-down
        GPIOx->PUPDR &= ~(3UL << shift);
    }
}

static inline void adc_set_config(const ADC_Config_t *cfg) {
    int index = adc_get_index(cfg);
    if (index < 0) return;
    adc_cfg_table[index] = cfg;
}

static inline void adc_init(const ADC_Config_t *cfg) {
    adc_set_config(cfg);
    // ADC off
    cfg->instance->CR2 = 0;
    // Resolution
    cfg->instance->CR1 &= ~(3UL << 24);
    cfg->instance->CR1 |= ((uint32_t)cfg->resolution << 24);
    // ADC interrupt on overruns
    cfg->instance->CR1 |= ADC_CR1_OVRIE;
    // Alignment
    if(cfg->alignment == ADC_ALIGN_LEFT) {
        cfg->instance->CR2 |= ADC_CR2_ALIGN;
    } else {
        cfg->instance->CR2 &= ~ADC_CR2_ALIGN;
    }
    // one conversion
    cfg->instance->SQR1 = 0;
    // channel
    cfg->instance->SQR3 = cfg->channel;
    // sample time
    if(cfg->channel <= 9) {
        cfg->instance->SMPR2 &= ~(7UL << (cfg->channel * 3));
        cfg->instance->SMPR2 |= ((uint32_t)cfg->sampleTime << (cfg->channel * 3));
    } else {
        uint32_t shift = (cfg->channel - 10) * 3;
        cfg->instance->SMPR1 &= ~(7UL << shift);
        cfg->instance->SMPR1 |= ((uint32_t)cfg->sampleTime << shift);
    }
    // External trigger configuration
    cfg->instance->CR2 &= ~(ADC_CR2_EXTSEL | ADC_CR2_EXTEN);
    // Select external trigger source
    if (cfg->externalTrigger != ADC_TRIGGER_SOFTWARE) {
        cfg->instance->CR2 |=
            ((uint32_t)cfg->externalTrigger << ADC_CR2_EXTSEL_Pos);
        // Enable rising-edge trigger
        cfg->instance->CR2 |= ADC_CR2_EXTEN_0;
    }
    // Disable continuous mode
    cfg->instance->CR2 &= ~ADC_CR2_CONT;
    // Enable DMA transfer with this ADC
    cfg->instance->CR2 |= ADC_CR2_DMA;
    // Keep DMA requests flowing after every conversion.
    cfg->instance->CR2 |= ADC_CR2_DDS;
    // EOC after each conversion
    cfg->instance->CR2 |= ADC_CR2_EOCS;
    // clear any pending ADC flags
    cfg->instance->SR = 0;
    // Enable ADC IRQ in NVIC
    NVIC_EnableIRQ(ADC_IRQn);
    // start pending
}

static inline void adc_enable(const ADC_Config_t *cfg) {
    // Clear ADC status flags (especially OVR)
    cfg->instance->SR = 0;
    // Enable ADC
    cfg->instance->CR2 |= ADC_CR2_ADON;
    // Wait briefly
    mcl::delay_us(10);
}

static inline bool adc_is_complete(const ADC_Config_t *cfg) {
    return (cfg->instance->SR & ADC_SR_EOC);
}

static inline void adc_start_conversion(const ADC_Config_t *cfg) {
    cfg->instance->CR2 |= ADC_CR2_SWSTART;
}

static inline uint16_t adc_software_read(const ADC_Config_t *cfg) {
    // clear old conversion flag
    cfg->instance->SR &= ~ADC_SR_EOC;
    adc_start_conversion(cfg);
    while(!adc_is_complete(cfg));
    return (uint16_t)cfg->instance->DR;
}

static inline void adc_stop(ADC_TypeDef *adc) {
    // Disable injected end-of-conversion interrupt
    adc->CR1 &= ~ADC_CR1_JEOCIE;
    // Disable external trigger for injected conversions
    adc->CR2 &= ~ADC_CR2_JEXTEN;
    // Disable ADC interrupt
    NVIC_DisableIRQ(ADC_IRQn);
    // Clear status flags
    adc->SR = 0;
}

extern "C" void ADC_IRQHandler(void) {
    const ADC_Config_t *cfg = adc_cfg_table[0]; // ADC1 for now
    if (!cfg) return;
    ADC_TypeDef *adc = cfg->instance;
    // Injected conversion complete.
    // JDR1 = phase A
    // JDR2 = phase B
    if (adc->SR & ADC_SR_JEOC) {
        uint16_t jdr1 = (uint16_t)adc->JDR1;
        uint16_t jdr2 = (uint16_t)adc->JDR2;
        adc->SR &= ~ADC_SR_JEOC;
        if (cfg->interrupt_callback_injected) {
            cfg->interrupt_callback_injected(jdr1, jdr2, cfg->ctx);
        }
    }
    // Existing regular ADC overrun handling.
    if (adc->SR & ADC_SR_OVR) {
        adc->SR &= ~ADC_SR_OVR;
        if (cfg->interrupt_callback_regular) {
            cfg->interrupt_callback_regular(0);
        }
    }
}

#endif
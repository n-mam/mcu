#ifndef TIMER_H
#define TIMER_H

#include <math.h>
#include <stdint.h>

#include <mcl/mcl.h>

typedef struct {
    uint8_t pin;
    uint8_t pin_n;
    uint16_t step;
    float dutyCycle;
    GPIO_TypeDef *port;
    GPIO_TypeDef *port_n;
} Timer_Channel_t;

typedef struct {
    float *lut;
    uint32_t arr;
    uint32_t lutSize;
    uint32_t frequency;
    TIM_TypeDef *instance;
    Timer_Channel_t channel[5];
} Timer_Config_t;

static Timer_Config_t *timer2_cfg = NULL;

static inline bool timer_is_advanced(const Timer_Config_t *cfg) {
    return (cfg->instance == TIM1);
}

static inline uint32_t timer_clock(const Timer_Config_t *cfg) {
    return timer_is_advanced(cfg) ?
        mcl::apb2TimerClock() : mcl::apb1TimerClock();
}

static inline void timer_init(const Timer_Config_t *cfg) {
    mcl::enableClockForTimer(cfg->instance);
    cfg->instance->PSC = (timer_clock(cfg) / 1000000U) - 1U;
    cfg->instance->CR1 = 0;
    cfg->instance->CR1 |= TIM_CR1_ARPE;
    // if (cfg->instance == TIM2) {
    //     timer2_cfg = (Timer_Config_t *)cfg;
    // }
}

static inline void timer_init_gpio(GPIO_TypeDef *port, const uint8_t *pins,
        uint32_t count, uint32_t af) {
    // Enable clock for GPIO
    mcl::enableClockForGpio(port);
    for (int i = 0; i < count; i++) {
        uint8_t pin = pins[i];
        // Clear mode bits for the pin (00 reset state)
        port->MODER &= ~(0b11 << (2 * pin));
        // Set mode to alternate function (10 AF mode)
        port->MODER |= (0b10 << (2 * pin));
        // Set alternate function to timer (AF01)
        if (pin < 8) {
            port->AFR[0] &= ~(0xF << (4 * pin)); // Clear AFRL bits
            port->AFR[0] |= (af << (4 * pin)); // AF for timer
        } else {
            port->AFR[1] &= ~(0xF << (4 * (pin - 8))); // Clear AFRH bits
            port->AFR[1] |= (af << (4 * (pin - 8))); // AF for timer
        }
    }
}

static inline void timer_init_channel(Timer_Config_t *cfg, uint8_t ch, GPIO_TypeDef *port,
        uint8_t pin, GPIO_TypeDef *port_n, uint8_t pin_n) {
    cfg->channel[ch].port = port;
    cfg->channel[ch].pin = pin;
    cfg->channel[ch].port_n = port_n;
    cfg->channel[ch].pin_n = pin_n;
    switch (ch) {
        case 1:
            // CC1 in output mode
            cfg->instance->CCMR1 &= ~TIM_CCMR1_CC1S;
            // CCR preload enable
            cfg->instance->CCMR1 |= TIM_CCMR1_OC1PE;
            // pwm mode 1
            cfg->instance->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
            break;
        case 2:
            // CC2 in output mode
            cfg->instance->CCMR1 &= ~TIM_CCMR1_CC2S;
            // CCR preload enable
            cfg->instance->CCMR1 |= TIM_CCMR1_OC2PE;
            // pwm mode 1
            cfg->instance->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
            break;
        case 3:
            // CC3 in output mode
            cfg->instance->CCMR2 &= ~TIM_CCMR2_CC3S;
            // CCR preload enable
            cfg->instance->CCMR2 |= TIM_CCMR2_OC3PE;
            // pwm mode 1
            cfg->instance->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
            break;
        case 4:
            // CC4 in output mode
            cfg->instance->CCMR2 &= ~TIM_CCMR2_CC4S;
            // CCR preload enable
            cfg->instance->CCMR2 |= TIM_CCMR2_OC4PE;
            // pwm mode 1
            cfg->instance->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
            break;
    }
}

static inline void timer_set_dead_time(const Timer_Config_t *cfg, uint32_t dt_ns) {
    // dead time is clocked directly using using TIM1/8 input clock which is
    // APB2 (either scaled or as-is). This is same as CLK_PSC BEFORE
    // it gets divided by the timer's PSC factor, post which it becomes
    // CLK_CNT (input to the counter); Time period of DTS in nanoseconds
    // ~ 10.41ns - with a 96MHz scaled APB1 timer input clock
    if (!timer_is_advanced(cfg)) return;
    double T_dts_ns = (1.0 / timer_clock(cfg)) * 1e9;
    uint32_t dt_ticks = (uint32_t)(dt_ns / T_dts_ns);
    cfg->instance->BDTR =
        (cfg->instance->BDTR & ~TIM_BDTR_DTG) | (dt_ticks & 0x7F);
    // this is around 24 which is less than 63. since it falls in the
    // first (linear) dead-time range, where DTG = DT_ticks TODO
}

static inline void timer_start_channel(const Timer_Config_t *cfg, uint8_t ch, bool complementary) {
    switch (ch) {
        case 1:
            // CC1 output enable and enable complementary output
            cfg->instance->CCER |= TIM_CCER_CC1E;
            if (complementary && timer_is_advanced(cfg))
                cfg->instance->CCER |= TIM_CCER_CC1NE;
            break;
        case 2:
            // CC2 output enable and enable complementary output
            cfg->instance->CCER |= TIM_CCER_CC2E;
            if (complementary && timer_is_advanced(cfg))
                cfg->instance->CCER |= TIM_CCER_CC2NE;
            break;
        case 3:
            // CC3 output enable and enable complementary output
            cfg->instance->CCER |= TIM_CCER_CC3E;
            if (complementary && timer_is_advanced(cfg))
                cfg->instance->CCER |= TIM_CCER_CC3NE;
            break;
        case 4:
            // CC4 output enable no and complementary output on channel 4
            cfg->instance->CCER |= TIM_CCER_CC4E;
            break;
    }
}

static inline void timer_stop_channel(const Timer_Config_t *cfg, uint8_t ch) {
    switch (ch) {
        case 1: cfg->instance->CCER &= ~TIM_CCER_CC1E; break;
        case 2: cfg->instance->CCER &= ~TIM_CCER_CC2E; break;
        case 3: cfg->instance->CCER &= ~TIM_CCER_CC3E; break;
        case 4: cfg->instance->CCER &= ~TIM_CCER_CC4E; break;
    }
}

static inline void timer_set_frequency(Timer_Config_t *cfg, uint32_t frequency) {
    cfg->frequency = frequency;
    // Timer clock frequency => 1MHz
    uint32_t clk = timer_clock(cfg) / (cfg->instance->PSC + 1);
    // Calculate ARR count value to have the desired output PWM frequency
    cfg->arr = (clk / frequency) - 1;
    // Set auto reload register for the desired output signal frequency
    cfg->instance->ARR = cfg->arr;
    //init_sine_table();
}

static inline void timer_set_duty_cycle(Timer_Config_t *cfg, uint8_t ch, float duty) {
    cfg->channel[ch].dutyCycle = duty;
    // CCR value configures the PWM duty cycle
    uint32_t ccr = (uint32_t)(duty * (cfg->arr + 1));
    // CC value for desired PWM duty cycle
    switch (ch) {
        case 1: cfg->instance->CCR1 = ccr; break;
        case 2: cfg->instance->CCR2 = ccr; break;
        case 3: cfg->instance->CCR3 = ccr; break;
        case 4: cfg->instance->CCR4 = ccr; break;
    }
}

static inline void timer_enable_trgo(const Timer_Config_t *cfg) {
    cfg->instance->CR2 &= ~TIM_CR2_MMS;
     // Update Event
    cfg->instance->CR2 |= TIM_CR2_MMS_1;
}

static inline void timer_enable_interrupt(const Timer_Config_t *cfg) {
    // Enable update event interrupt
    cfg->instance->DIER |= TIM_DIER_UIE;
    // Enable TIM IRQ in NVIC
    if (cfg->instance == TIM2)
        NVIC_EnableIRQ(TIM2_IRQn);
    else if (cfg->instance == TIM3)
        NVIC_EnableIRQ(TIM3_IRQn);
    else if (cfg->instance == TIM4)
        NVIC_EnableIRQ(TIM4_IRQn);
    else if (cfg->instance == TIM5)
        NVIC_EnableIRQ(TIM5_IRQn);
}

static inline void timer_start(const Timer_Config_t *cfg) {
    // Force an update event
    cfg->instance->EGR = TIM_EGR_UG;
    // Reset counter
    cfg->instance->CNT = 0;
    if (timer_is_advanced(cfg)) {
        // Disable break input
        cfg->instance->BDTR &= ~TIM_BDTR_BKE;
        // Main output enable
        cfg->instance->BDTR |= TIM_BDTR_MOE;
    }
    // Enable the timer
    cfg->instance->CR1 |= TIM_CR1_CEN;
}

static inline void timer_stop(const Timer_Config_t *cfg) {
    if (timer_is_advanced(cfg))
        cfg->instance->BDTR &= ~TIM_BDTR_MOE;
    cfg->instance->CR1 &= ~TIM_CR1_CEN;
    cfg->instance->SR = 0;
}

// First we scale down APB1 timer to derive a 1 MHz timer input clock
// We then set ARR to configure the actual PWM frequency to say 50 Hz
// This implies that we have 50 pwm pulses in 1 sec, each of which is
// available for a single sinusoidal duty cycle increment step.
static inline void timer_init_sine_table(Timer_Config_t *cfg) {
    double step = M_PI / cfg->frequency;
    for (uint32_t i = 0; i <= cfg->frequency; i++)
        cfg->lut[i] = sin(i * step);
    cfg->lutSize = cfg->frequency + 1;
}

static inline void timer_sinusoidal_next_step(Timer_Config_t *cfg, uint8_t ch) {
    uint16_t *step = &cfg->channel[ch].step;
    timer_set_duty_cycle(cfg, ch, cfg->lut[*step]);
    (*step)++;
    if (*step >= cfg->lutSize)
        *step = 0;
}

extern "C" {
    void TIM2_IRQHandler(void) {
        if (TIM2->SR & TIM_SR_UIF) {
            // Clear interrupt flag
            TIM2->SR &= ~TIM_SR_UIF;
            if (timer2_cfg) {
                timer_sinusoidal_next_step(timer2_cfg, 1);
                timer_sinusoidal_next_step(timer2_cfg, 2);
                timer_sinusoidal_next_step(timer2_cfg, 3);
            }
        }
    }
}

inline void tim_test() {
    // RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    // TIM2->PSC = 8999;
    // TIM2->ARR = 9999;
    // TIM2->CR1 = TIM_CR1_ARPE;
    // TIM2->EGR = TIM_EGR_UG;
    // TIM2->CR1 |= TIM_CR1_CEN;
    // while(1) {
    //     LOG << " " << (int)TIM2->CNT;
    //     mcl::sleep_ms(100);
    // }
    Timer_Config_t tim{};
    tim.instance = TIM2;
    timer_init(&tim);
    timer_set_frequency(&tim, 777);
    timer_start(&tim);
    LOG << " PSC=" << TIM2->PSC
        << " ARR=" << (uint32_t)TIM2->ARR
        << " CR1=" << TIM2->CR1
        << " CNT=" << (uint32_t)TIM2->CNT
        << " SR=" << TIM2->SR
        << " APB1ENR=" << (uint32_t)RCC->APB1ENR;
    while(true) {
        LOG << " " << (uint32_t)TIM2->CNT;
        mcl::sleep_ms(100);
    }
}

#endif
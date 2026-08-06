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
} timer_channel_t;

// center aligned mode setting
enum cms { ea, ca1, ca2, ca3 };

typedef struct {
    cms mode;
    float *lut;
    uint32_t arr;
    uint32_t lutSize;
    uint32_t frequency;
    TIM_TypeDef *instance;
    timer_channel_t channel[5];
} timer_config_t;

typedef struct {
    timer_config_t *timer;
    float angle;                    // current electrical angle (rad)
    float modulation;               // 0..0.866
    float electrical_frequency;     // electrical field speed (Hz)
    float svpwm_update_frequency;
} svpwm_t;

static svpwm_t g_svpwm;

static timer_config_t *timer2_cfg = NULL;

static inline bool timer_is_advanced(const timer_config_t *cfg) {
    return (cfg->instance == TIM1);
}

static inline uint32_t timer_clock(const timer_config_t *cfg) {
    return timer_is_advanced(cfg) ?
        mcl::apb2TimerClock() : mcl::apb1TimerClock();
}

static inline void timer_init(const timer_config_t *cfg) {
    mcl::enableClockForTimer(cfg->instance);
    cfg->instance->PSC = (timer_clock(cfg) / 1000000U) - 1U;
    cfg->instance->CR1 = TIM_CR1_ARPE;
    if (timer_is_advanced(cfg) && (cfg->mode == cms::ca1)) {
        // center aligned mode 1
        cfg->instance->CR1 |= TIM_CR1_CMS_0;
        // for advanced timers, with RCR = 0 in center-aligned mode,
        // an update event (and thus the ISR) is generated at both the
        // counter overflow and underflow — i.e., twice per PWM period,
        // so the ISR will actually run at 40 kHz, not 20 kHz. That will
        // silently double the effective angle-advance rate
        cfg->instance->RCR = 1;
    }
    // if (cfg->instance == TIM2) {
    //     timer2_cfg = (timer_config_t *)cfg;
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

static inline void timer_init_channel(timer_config_t *cfg, uint8_t ch, GPIO_TypeDef *port,
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

static inline void timer_set_dead_time(const timer_config_t *cfg, uint32_t dt_ns) {
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

static inline void timer_start_channel(const timer_config_t *cfg, uint8_t ch, bool complementary) {
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

static inline void timer_stop_channel(const timer_config_t *cfg, uint8_t ch) {
    switch (ch) {
        case 1: cfg->instance->CCER &= ~TIM_CCER_CC1E; break;
        case 2: cfg->instance->CCER &= ~TIM_CCER_CC2E; break;
        case 3: cfg->instance->CCER &= ~TIM_CCER_CC3E; break;
        case 4: cfg->instance->CCER &= ~TIM_CCER_CC4E; break;
    }
}

static inline void timer_set_frequency(timer_config_t *cfg, uint32_t frequency) {
    cfg->frequency = frequency;
    // Timer clock frequency => 1MHz
    uint32_t clk = timer_clock(cfg) / (cfg->instance->PSC + 1);
    // Calculate ARR count value to have the desired output PWM frequency
    if (timer_is_advanced(cfg) && (cfg->mode != cms::ea)) {
        cfg->arr = (clk / (2 * frequency)) - 1; // center aligned
    } else {
        cfg->arr = (clk / frequency) - 1; // left aligned
    }
    // Set auto reload register for the desired output signal frequency
    cfg->instance->ARR = cfg->arr;
    //init_sine_table();
}

static inline void timer_set_duty_cycle(timer_config_t *cfg, uint8_t ch, float duty) {
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

static inline void timer_enable_trgo(const timer_config_t *cfg) {
    cfg->instance->CR2 &= ~TIM_CR2_MMS;
     // Update Event
    cfg->instance->CR2 |= TIM_CR2_MMS_1;
}

static inline void timer_enable_interrupt(const timer_config_t *cfg) {
    // Enable update event interrupt
    cfg->instance->DIER |= TIM_DIER_UIE;
    // Enable TIM IRQ in NVIC
    if (cfg->instance == TIM1)
        NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    else if (cfg->instance == TIM2)
        NVIC_EnableIRQ(TIM2_IRQn);
    else if (cfg->instance == TIM3)
        NVIC_EnableIRQ(TIM3_IRQn);
    else if (cfg->instance == TIM4)
        NVIC_EnableIRQ(TIM4_IRQn);
    else if (cfg->instance == TIM5)
        NVIC_EnableIRQ(TIM5_IRQn);
}

static inline void timer_start(const timer_config_t *cfg) {
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

static inline void timer_stop(const timer_config_t *cfg) {
    if (timer_is_advanced(cfg))
        cfg->instance->BDTR &= ~TIM_BDTR_MOE;
    cfg->instance->CR1 &= ~TIM_CR1_CEN;
    cfg->instance->SR = 0;
}

// First we scale down APB1 timer to derive a 1 MHz timer input clock
// We then set ARR to configure the actual PWM frequency to say 50 Hz
// This implies that we have 50 pwm pulses in 1 sec, each of which is
// available for a single sinusoidal duty cycle increment step.
static inline void timer_init_sine_table(timer_config_t *cfg) {
    double step = M_PI / cfg->frequency;
    for (uint32_t i = 0; i <= cfg->frequency; i++)
        cfg->lut[i] = sin(i * step);
    cfg->lutSize = cfg->frequency + 1;
}

static inline void timer_sinusoidal_next_step(timer_config_t *cfg, uint8_t ch) {
    uint16_t *step = &cfg->channel[ch].step;
    timer_set_duty_cycle(cfg, ch, cfg->lut[*step]);
    (*step)++;
    if (*step >= cfg->lutSize)
        *step = 0;
}

// standard symmetric SVPWM dwell-time equations
// T1 = √3 m sin(60°−α)
// T2 = √3 m sin(α)
// T0 = 1−T1−T2
static inline void svpwm_update(svpwm_t *svp) {
    constexpr float PI = 3.14159265359f;
    constexpr float PI3 = PI / 3.0f;
    constexpr float SQRT3 = 1.73205080757f;
    float theta = svp->angle;
    int sector = (int)(theta / PI3) % 6;
    float alpha = theta - sector * PI3;
    float m = svp->modulation;
    //-------------------------------------
    // normalized switching times
    //-------------------------------------
    float T1 = SQRT3 * m * sinf(PI3 - alpha);
    float T2 = SQRT3 * m * sinf(alpha);
    if(T1 + T2 > 1.0f) {
        float s = 1.0f / (T1 + T2);
        T1 *= s;
        T2 *= s;
    }
    float T0 = 1.0f - T1 - T2;
    float Ta = 0.0f, Tb = 0.0f, Tc = 0.0f;
    switch(sector) {
        case 0:
            Ta = T1 + T2 + T0*0.5f;
            Tb = T2 + T0*0.5f;
            Tc = T0*0.5f;
            break;
        case 1:
            Ta = T1 + T0*0.5f;
            Tb = T1 + T2 + T0*0.5f;
            Tc = T0*0.5f;
            break;
        case 2:
            Ta = T0*0.5f;
            Tb = T1 + T2 + T0*0.5f;
            Tc = T2 + T0*0.5f;
            break;
        case 3:
            Ta = T0*0.5f;
            Tb = T1 + T0*0.5f;
            Tc = T1 + T2 + T0*0.5f;
            break;
        case 4:
            Ta = T2 + T0*0.5f;
            Tb = T0*0.5f;
            Tc = T1 + T2 + T0*0.5f;
            break;
        case 5:
            Ta = T1 + T2 + T0*0.5f;
            Tb = T0*0.5f;
            Tc = T1 + T0*0.5f;
            break;
    }
    uint32_t arr = svp->timer->arr + 1;
    svp->timer->instance->CCR1 = (uint32_t)(Ta * arr);
    svp->timer->instance->CCR2 = (uint32_t)(Tb * arr);
    svp->timer->instance->CCR3 = (uint32_t)(Tc * arr);
    float angle_step = (2.0f * PI * svp->electrical_frequency) / svp->svpwm_update_frequency;
    svp->angle += angle_step;
    if (svp->angle >= 2.0f * PI) {
        svp->angle -= 2.0f * PI;
    }
}

extern "C"
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

extern "C"
    void TIM1_UP_TIM10_IRQHandler(void) {
        if(TIM1->SR & TIM_SR_UIF) {
            TIM1->SR &= ~TIM_SR_UIF;
            svpwm_update(&g_svpwm);
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
    timer_config_t tim{};
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
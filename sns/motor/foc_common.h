#ifndef FOC_COMMON_H
#define FOC_COMMON_H

constexpr float ADC_VREF   = 3.3f;
constexpr float CSA_GAIN   = 50.0f;      // V/V, INA240A2
constexpr float R_SHUNT    = 0.010f;     // 10 mill ohms
constexpr float ADC_MAXCNT = 4095.0f;

constexpr float PI = 3.14159265359f;
constexpr float PI3 = PI / 3.0f;
constexpr float TWO_PI = 6.28318530718f;
constexpr float SQRT3  = 1.73205080757f;

struct current_sense_t {
    volatile uint16_t raw_phase_a = 0;
    volatile uint16_t raw_phase_b = 0;
    volatile bool ready = false;
    volatile uint32_t isr_hits = 0;
    float bias_a = 2.5f;     // runtime calibrated bias
    float bias_b = 2.5f;
};

struct ramp_profile_trap_t {
    // Electrical frequency ramp [Hz]
    float ef_start;
    float ef_end;
    // Trapezoidal PWM duty ramp [0..1]
    float trap_duty_start;
    float trap_duty_end;
    // Ramp duration [seconds]
    float duration_s;
};

struct phase_currents_t {
    float ia, ib, ic;      // phase currents
    float alpha, beta;     // stationary frame
    float d, q;            // rotating frame
};

// PI controller
struct pi_controller_t {
    float kp = 0.0f;
    float ki = 0.0f;
    float integrator = 0.0f;
    float out_min = -1.0f;
    float out_max =  1.0f;
};

struct current_loop_t {
    pi_controller_t id_pi;
    pi_controller_t iq_pi;
    float id_ref = 0.0f;             // 0 for this motor -- no reluctance torque to exploit
    float iq_ref = 0.0f;             // torque command -- set externally, soft-started
    float vbus = 9.49f;                // confirm against your actual bench supply
    float overcurrent_limit = 0.8f;   // amps -- motor is rated 0.4A/max 1A per datasheet, leave margin
};

// functions
inline float ramp_value(float start, float end,
    float duration_s, float elapsed_s) {
        if (duration_s <= 0.0f)
            return end;
        float frac = elapsed_s / duration_s;
        if (frac <= 0.0f)
            frac = 0.0f;
        else if (frac >= 1.0f)
            frac = 1.0f;
        return start + frac * (end - start);
}

// linear svpwm limit:
// modulation = |Vref| / Vbus
constexpr float SVPWM_MAX_MODULATION = 0.57735026919f; // 1/sqrt(3)

struct svpwm_t {
    timer_config_t *timer = nullptr;
    // Command/state, useful for diagnostics.
    float alpha = 0.0f;
    float beta = 0.0f;
    float angle = 0.0f;
    float modulation = 0.0f;
};

// Apply a stationary-frame voltage vector:
//     alpha,beta = Vref vector normalized to Vbus
// The input magnitude is:
//     modulation = sqrt(alpha^2 + beta^2)
// with the linear SVPWM limit:
//     modulation <= 1/sqrt(3)
// This function:
// - calculates sector
// - calculates T1/T2/T0
// - calculates Ta/Tb/Tc
// - writes CCR1/CCR2/CCR3
// It does NOT:
// - advance angle
// - read encoder
// - run PI
// - know open/closed loop
inline void svpwm_update(svpwm_t &svpwm, float alpha, float beta) {
    // Store command for diagnostics.
    svpwm.alpha = alpha;
    svpwm.beta  = beta;
    // Convert alpha/beta vector to magnitude and angle.
    float modulation = sqrtf(alpha * alpha + beta * beta);
    if (modulation > SVPWM_MAX_MODULATION) {
        modulation = SVPWM_MAX_MODULATION;
    }
    svpwm.modulation = modulation;
    float theta = atan2f(beta, alpha);
    if (theta < 0.0f) {
        theta += TWO_PI;
    }
    svpwm.angle = theta;
    // Sector:
    // 0:   0 .. 60°
    // 1:  60 .. 120°
    // 2: 120 .. 180°
    // 3: 180 .. 240°
    // 4: 240 .. 300°
    // 5: 300 .. 360°
    int sector = (int)(theta / PI3);
    if (sector >= 6) {
        sector = 5;
    }
    float alpha_sector = theta - (float)sector * PI3;
    // Normalized dwell times.
    // T1 + T2 <= 1 in the linear modulation range.
    float T1 = SQRT3 * modulation * sinf(PI3 - alpha_sector);
    float T2 = SQRT3 * modulation * sinf(alpha_sector);
    float T0 = 1.0f - T1 - T2;
    // Defensive clamp against floating-point roundoff.
    if (T0 < 0.0f) {
        T0 = 0.0f;
    }
    float Ta = 0.0f;
    float Tb = 0.0f;
    float Tc = 0.0f;
    const float T0_half = 0.5f * T0;
    switch (sector) {
        case 0:
            Ta = T1 + T2 + T0_half;
            Tb = T2 + T0_half;
            Tc = T0_half;
            break;
        case 1:
            Ta = T1 + T0_half;
            Tb = T1 + T2 + T0_half;
            Tc = T0_half;
            break;
        case 2:
            Ta = T0_half;
            Tb = T1 + T2 + T0_half;
            Tc = T2 + T0_half;
            break;
        case 3:
            Ta = T0_half;
            Tb = T1 + T0_half;
            Tc = T1 + T2 + T0_half;
            break;
        case 4:
            Ta = T2 + T0_half;
            Tb = T0_half;
            Tc = T1 + T2 + T0_half;
            break;
        case 5:
            Ta = T1 + T2 + T0_half;
            Tb = T0_half;
            Tc = T1 + T0_half;
            break;
    }
    // Defensive bounds.
    Ta = fminf(fmaxf(Ta, 0.0f), 1.0f);
    Tb = fminf(fmaxf(Tb, 0.0f), 1.0f);
    Tc = fminf(fmaxf(Tc, 0.0f), 1.0f);
    uint32_t arr = svpwm.timer->arr + 1U;
    svpwm.timer->instance->CCR1 = (uint32_t)(Ta * (float)arr);
    svpwm.timer->instance->CCR2 = (uint32_t)(Tb * (float)arr);
    svpwm.timer->instance->CCR3 = (uint32_t)(Tc * (float)arr);
}

// foc_closed

inline void init_current_sampling() {
    // Enable GPIOA clock and
    // configure PA0 and PA1 to analog
    adc_gpio_init(GPIOA, {0, 1});
    // Enable clock for ADC peripheral
    // to access ADC common registers
    mcl::enableClockForAdc(ADC1);
    adc_global_init();
    // TIM1 CH4
    // Internal-only PWM/compare channel.
    // No GPIO is assigned to CH4.
    // Center-aligned TIM1:
    // With PWM mode 1 and rising-edge ADC trigger, the CH4 rising
    // edge occurs on the down-count when CNT crosses CCR4.
    // ARR-1 gives us a trigger essentially at the center of the
    // PWM period, while avoiding the CCR4 == ARR trigger issue.
    TIM1->CCMR2 &= ~(TIM_CCMR2_CC4S | TIM_CCMR2_OC4M);
    // PWM mode 1
    TIM1->CCMR2 |= (6UL << TIM_CCMR2_OC4M_Pos);
    // CCR4 preload
    TIM1->CCMR2 |= TIM_CCMR2_OC4PE;
    // Trigger essentially at the center of the
    // center-aligned PWM cycle.
    TIM1->CCR4 = TIM1->ARR - 1U;
    // Enable CH4 internally.
    // No GPIO configuration is necessary.
    TIM1->CCER |= TIM_CCER_CC4E;
    // ADC1
    // PA0 -> ADC1_IN0 -> JDR1
    // PA1 -> ADC1_IN1 -> JDR2
    // Two injected conversions per TIM1 CH4 trigger.
    constexpr uint32_t PHASE_A = ADC_CH0;
    constexpr uint32_t PHASE_B = ADC_CH1;
    constexpr uint32_t SAMPLE_TIME = ADC_SAMPLE_84;
    // ADC sample time
    ADC1->SMPR2 &= ~(
        (7UL << (PHASE_A * 3U)) |
        (7UL << (PHASE_B * 3U))
    );
    ADC1->SMPR2 |=
        (SAMPLE_TIME << (PHASE_A * 3U)) |
        (SAMPLE_TIME << (PHASE_B * 3U));
    // 12-bit, right aligned
    ADC1->CR1 &= ~(3UL << 24);
    ADC1->CR2 &= ~ADC_CR2_ALIGN;
    ADC1->CR1 |= ADC_CR1_SCAN;
    // Injected sequence
    // JSQ1 = phase A
    // JSQ2 = phase B
    // JL = 1 => sequence contains 2 conversions.
    ADC1->JSQR = 0;
    ADC1->JSQR |= (PHASE_A << ADC_JSQR_JSQ3_Pos);
    ADC1->JSQR |= (PHASE_B << ADC_JSQR_JSQ4_Pos);
    ADC1->JSQR |= ADC_JSQR_JL_0;
    // TIM1_CH4 -> injected trigger
    // STM32F446:
    // JEXTSEL = 0000 -> TIM1_CH4
    // JEXTEN  = 01   -> rising edge
    ADC1->CR2 &= ~(ADC_CR2_JEXTSEL | ADC_CR2_JEXTEN);
    // JEXTSEL = 0000 => TIM1_CH4
    // Rising-edge trigger
    ADC1->CR2 |= ADC_CR2_JEXTEN_0;
    // Interrupt after the injected sequence completes.
    // This occurs after JDR1 and JDR2 have both been filled.
    ADC1->CR1 |= ADC_CR1_JEOCIE;
    // Clear stale status.
    ADC1->SR = 0;
    // Enable ADC1.
    // Do NOT use SWSTART/JSWSTART.
    // TIM1 CH4 starts every injected sequence.
    ADC1->CR2 |= ADC_CR2_ADON;
    // ADC interrupt.
    NVIC_EnableIRQ(ADC_IRQn);
}

inline float adc_to_phase_amps(uint16_t raw, float v_bias) {
    float v_adc = ((float)raw / ADC_MAXCNT) * ADC_VREF;
    return (v_adc - v_bias) / (CSA_GAIN * R_SHUNT);
}

inline void clarke_transform(phase_currents_t &pc) {
    constexpr float ONE_THIRD = 1.0f / 3.0f;
    // amplitude-invariant Clarke transformation
    // uses all 3 measured/reconstructed phases
    pc.alpha = (2.0f * pc.ia - pc.ib - pc.ic) * ONE_THIRD;
    pc.beta  = (pc.ib - pc.ic) * (1.0f / SQRT3);
}

inline void park_transform(phase_currents_t &pc, float theta) {
    float c = cosf(theta);
    float s = sinf(theta);
    pc.d =  pc.alpha * c + pc.beta * s;
    pc.q = -pc.alpha * s + pc.beta * c;
}

inline phase_currents_t compute_dq_currents(uint16_t raw_a, uint16_t raw_b,
        float bias_a, float bias_b, float theta) {
    phase_currents_t pc{};
    pc.ia = adc_to_phase_amps(raw_a, bias_a);
    pc.ib = adc_to_phase_amps(raw_b, bias_b);
    pc.ic = -(pc.ia + pc.ib);
    clarke_transform(pc);
    park_transform(pc, theta);
    return pc;
}

inline float pi_update(pi_controller_t &pi, float error, float dt) {
    pi.integrator += pi.ki * error * dt;
    if (pi.integrator > pi.out_max) pi.integrator = pi.out_max;
    if (pi.integrator < pi.out_min) pi.integrator = pi.out_min;
    float out = pi.kp * error + pi.integrator;
    if (out > pi.out_max) out = pi.out_max;
    if (out < pi.out_min) out = pi.out_min;
    return out;
}

inline void pi_reset(pi_controller_t &pi) {
    pi.integrator = 0.0f;
}

inline void current_loop_reset(current_loop_t &loop) {
    pi_reset(loop.id_pi);
    pi_reset(loop.iq_pi);
}

#endif
#ifndef FOC_H
#define FOC_H

constexpr float ADC_VREF   = 3.3f;
constexpr float CSA_GAIN   = 50.0f;      // V/V, INA240A2
constexpr float R_SHUNT    = 0.010f;     // 10 mill ohms
constexpr float ADC_MAXCNT = 4095.0f;

// DFRobot 2804, 12N/14P
constexpr uint32_t POLE_PAIRS = 7;
constexpr float TWO_PI = 6.28318530718f;
constexpr uint32_t ENCODER_COUNTS = 4096;
volatile bool encoder_calibration_hold = true;

struct ramp_profile_t {
    // Electrical frequency ramp [Hz]
    float ef_start;
    float ef_end;
    // linear svpwm modulation ramp [0 ... 0.57735026] 1/sqrt(3)
    // This is Vref / Vbus.
    float m_start;
    float m_end;
    // Trapezoidal PWM duty ramp [0..1]
    float trap_duty_start;
    float trap_duty_end;
    // Ramp duration [seconds]
    float duration_s;
};

static inline float ramp_value(float start, float end,
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

struct current_sense_t {
    volatile uint16_t raw_phase_a = 0;
    volatile uint16_t raw_phase_b = 0;
    volatile bool ready = false;
    volatile uint32_t isr_hits = 0;
    float bias_a = 2.5f;     // runtime calibrated bias
    float bias_b = 2.5f;
};

struct svpwm_t {
    float angle;                    // current electrical angle (rad)
    float modulation;               // 0..0.866
    float electrical_frequency;     // electrical field speed (Hz)
    float svpwm_update_frequency;
    timer_config_t *timer;
};

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

struct phase_currents_t {
    float ia, ib, ic;      // phase currents
    float alpha, beta;     // stationary frame
    float d, q;            // rotating frame
};

inline float adc_to_phase_amps(uint16_t raw, float v_bias) {
    float v_adc = ((float)raw / ADC_MAXCNT) * ADC_VREF;
    return (v_adc - v_bias) / (CSA_GAIN * R_SHUNT);
}

inline void clarke_transform(phase_currents_t &pc) {
    constexpr float ONE_THIRD = 1.0f / 3.0f;
    constexpr float SQRT3 = 1.73205080757f;
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

inline uint16_t as5600_read_raw_angle(serial::i2c &bus) {
    //todo:: 4 byte read for now because of 2-byte read bug
    uint8_t buf[4] = {0};
    bus.read(0x36, 0x0C, buf, 4);
    return ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
}

inline float encoder_to_electrical_angle(uint16_t raw, float zero_offset) {
    int32_t delta = (int32_t)raw - (int32_t)zero_offset;
    delta = ((delta % (int32_t)ENCODER_COUNTS) + (int32_t)ENCODER_COUNTS) % (int32_t)ENCODER_COUNTS;
    float mech_frac = (float)delta / (float)ENCODER_COUNTS;
    return fmodf(mech_frac * TWO_PI * POLE_PAIRS, TWO_PI);
}

// standard symmetric SVPWM dwell-time equations
// T1 = √3 m sin(60°−α)
// T2 = √3 m sin(α)
// T0 = 1 − T1 − T2
inline void svpwm_update(svpwm_t *svp) {
    constexpr float PI = 3.14159265359f;
    constexpr float PI3 = PI / 3.0f;
    constexpr float SQRT3 = 1.73205080757f;
    float theta = svp->angle;
    int sector = (int)(theta / PI3) % 6;
    float alpha = theta - sector * PI3;
    float m = svp->modulation;
    // normalized switching times
    float T1 = SQRT3 * m * sinf(PI3 - alpha);
    float T2 = SQRT3 * m * sinf(alpha);
    // this is redundant if we gurantee that
    // the modulation would be inside the linear
    // range which is 1/sqrt(3) of Vbus
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

inline svpwm_t g_svpwm;
inline current_sense_t g_current;

void tim1_callback(timer_event_t event) {
    if (event != TIMER_EVENT_UPDATE) return;
    if (!encoder_calibration_hold) {
        svpwm_update(&g_svpwm);
    }
}

void adc_callback_injected(uint16_t phase_a, uint16_t phase_b) {
    g_current.raw_phase_a = phase_a;
    g_current.raw_phase_b = phase_b;
    g_current.ready = true;
}

inline void test_bldc_svpwm() {
    timer_config_t tm{};
    tm.instance = TIM1;
    tm.interrupt_callback = tim1_callback;
    tm.mode = cms::ca1;
    timer_init(&tm);
    timer_set_frequency(&tm, 20000);

    static const uint8_t hs[] = {8, 9, 10};
    static const uint8_t ls[] = {13, 14, 15};
    timer_init_gpio(GPIOA, hs, 3, 1);
    timer_init_gpio(GPIOB, ls, 3, 1);

    //timer_set_dead_time(&tm, 250);

    timer_init_channel(&tm, 1, GPIOA, 8, GPIOB, 13);
    timer_init_channel(&tm, 2, GPIOA, 9, GPIOB, 14);
    timer_init_channel(&tm, 3, GPIOA, 10, GPIOB, 15);

    timer_start_channel(&tm, 1, true);
    timer_start_channel(&tm, 2, true);
    timer_start_channel(&tm, 3, true);

    // CH4 trigger + ADC1 injected sequence
    init_current_sampling();
    ADC_Config_t cfg{};
    cfg._instance = ADC1;
    cfg._interrupt_callback_injected = adc_callback_injected;
    cfg._interrupt_callback_regular = nullptr;
    adc_set_config(&cfg);

    g_svpwm.timer = &tm;
    g_svpwm.angle = 0.0f;
    // TIM1 generates one update interrupt per PWM
    // period then: update_frequency = PWM frequency
    // TIM1 is configured as:
    //     20 kHz PWM
    //     center aligned
    //     RCR = 1
    // Therefore svpwm_update() is intended to execute at 20 kHz.
    g_svpwm.svpwm_update_frequency = 20'000.0f;

    ramp_profile_t ramp{};
    // svpm modulation
    // start to max liear svpm modulation
    ramp.m_start = 0.10f;
    ramp.m_end   = 0.57735026f;
    // Electrical frequency: (all off 5V Vbus)
    // 5 Hz -> 35 Hz electrical (hard disk motor)
    // 2 Hz -> 3 Hz electrcal (2804 3-Phase gimbal dfrobot)
    ramp.ef_start = 1.0f;
    ramp.ef_end   = 2.0f; //ok:7, notok:40
    // Trapezoidal fields are unused here.
    ramp.trap_duty_start = 0.0f;
    ramp.trap_duty_end   = 0.0f;
    // Both modulation and frequency use this same duration.
    ramp.duration_s = 8.0f;
    // Initial SVPWM state.
    g_svpwm.modulation = ramp.m_start;
    g_svpwm.electrical_frequency = ramp.ef_start;

    encoder_calibration_hold = true;
    timer_enable_interrupt(&tm);
    timer_start(&tm);
    // encoder offset calibration
    serial::i2c bus(3, 10, 400'000, I2C2, GPIOB);
    svpwm_t svpwm;
    svpwm.timer = &tm;
    svpwm.angle = 0.0f;
    svpwm.modulation = 0.15f;
    svpwm.electrical_frequency = 0.0f;
    svpwm.svpwm_update_frequency = 20'000.0f;
    svpwm_update(&svpwm);
    // wait for rotor to snap to electrical
    // zero take average of 50 samples
    mcl::sleep_ms(500);
    float encoder_zero_offset = 0;
    for (int i = 0; i < 50; i++) {
        LOG << " done..";
        float raw = (float)as5600_read_raw_angle(bus);
        LOG << " encoder_zero_offset: " << raw;
        encoder_zero_offset += raw;
        mcl::sleep_ms(2);
    }
    encoder_zero_offset /= 50.0f;
    encoder_calibration_hold = false;
    LOG << " encoder_zero_offset: " << encoder_zero_offset;

    const uint32_t ramp_start_ms = mcl::time_ms();
    uint32_t last_ramp_update_ms = ramp_start_ms;

    float g_theta_measured = 0.0f;
    static uint32_t last_encoder_ms = 0;

    while (!getInstance<config>()->shouldExit()) {
        // TIM1 update interrupt and SysTick can wake the processor.
        __WFI();
        uint32_t now_ms = mcl::time_ms();
        // The ramp itself is very slow compared with the 20 kHz
        // PWM interrupt, so a 1 ms foreground update is sufficient.
        if ((uint32_t)(now_ms - last_ramp_update_ms) >= 1U) {
            float elapsed_s = (float)(now_ms - ramp_start_ms) * 0.001f;
            // Both values use EXACTLY the same elapsed_s.
            // Therefore the voltage and frequency ramps are
            // synchronized.
            g_svpwm.modulation =
                ramp_value(
                    ramp.m_start,
                    ramp.m_end,
                    ramp.duration_s,
                    elapsed_s);
            g_svpwm.electrical_frequency =
                ramp_value(
                    ramp.ef_start,
                    ramp.ef_end,
                    ramp.duration_s,
                    elapsed_s);
            last_ramp_update_ms = now_ms;
        }

        if ((uint32_t)(now_ms - last_encoder_ms) >= 2U) {
            uint16_t raw = as5600_read_raw_angle(bus);
            g_theta_measured = encoder_to_electrical_angle(raw, encoder_zero_offset);
            last_encoder_ms = now_ms;
        }

        // current sense logging
        static uint32_t last_log_ms = 0;
        if (g_current.ready) {
            g_current.ready = false;
            // ~100 Hz log rate
            if ((uint32_t)(now_ms - last_log_ms) >= 10U) {
                // 2.5V Bi-directional mid point confirmed by 7semi support
                phase_currents_t pc = compute_dq_currents(
                    g_current.raw_phase_a, g_current.raw_phase_b,
                        g_current.bias_a, g_current.bias_b, g_theta_measured/*g_svpwm.angle*/);
                LOG << " Ia: " << pc.ia << ", Ib: " << pc.ib
                        << ", Ic: " << pc.ic << ", Id: " << pc.d << ", Iq: "
                            << pc.q << ", theta: " << g_theta_measured;
                last_log_ms = now_ms;
            }
        }
    }
    timer_stop(&tm);
}

#endif
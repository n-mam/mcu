#ifndef HAL_H
#define HAL_H

constexpr float ADC_VREF = 3.3f;
constexpr float ADC_MAXCNT = 4095.0f;
constexpr float VBUS = 9.49f;
constexpr float INV_VBUS = 1 / VBUS;

struct hardware_t {
    timer_config_t pwm_timer{};
    timer_config_t encoder_timer{};
    serial::i2c *encoder_i2c = nullptr;
};

struct csa_calibration_t {
    // CSA zero current bias
    float bias_a = 0.0f;
    float bias_b = 0.0f;
    uint64_t sum_a = 0;
    uint64_t sum_b = 0;
    uint32_t samples = 0;
    bool calibrating = false;
};

inline hardware_t hw{};
inline csa_calibration_t calibration;

typedef void (*pfn_tim_encoder_callback_t)(timer_event_t);
typedef void (*pfn_adc_injected_callback_t)(uint16_t, uint16_t);

inline void initialize_phase_pwm_timer() {
    auto& tm = hw.pwm_timer;
    tm.instance = TIM1;
    tm.interrupt_callback = nullptr;
    tm.mode = cms::ca1;
    timer_init(&tm);
    timer_set_frequency(&tm, 20'000);
    static const uint8_t hs[] = {8, 9, 10};
    static const uint8_t ls[] = {13, 14, 15};
    timer_init_gpio(GPIOA, hs, 3, 1);
    timer_init_gpio(GPIOB, ls, 3, 1);
    // TMC6300 has hardware dead time insertion (BBM)
    // timer_set_dead_time(&tm, 250);
    timer_init_channel(&tm, 1, GPIOA, 8, GPIOB, 13);
    timer_init_channel(&tm, 2, GPIOA, 9, GPIOB, 14);
    timer_init_channel(&tm, 3, GPIOA, 10, GPIOB, 15);
    // start timer channels
    timer_start_channel(&tm, 1, true);
    timer_start_channel(&tm, 2, true);
    timer_start_channel(&tm, 3, true);
}

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

inline void initialize_adc_hardware(pfn_adc_injected_callback_t callback) {
    // CH4 trigger + ADC1 injected sequence
    init_current_sampling();
    static ADC_Config_t cfg{};
    cfg._instance = ADC1;
    cfg._interrupt_callback_regular = nullptr;
    cfg._interrupt_callback_injected = callback;
    adc_set_config(&cfg);
}

inline void calibrate_current_sensor() {
    auto& tm = hw.pwm_timer;
    // Zero current ADC bias calibration
    constexpr uint32_t CURRENT_BIAS_SAMPLES = 1000;
    // Keep all three PWM phases at 50% duty while
    // the ADC continues to be triggered by TIM1 CH4.
    const uint32_t zero_duty = (tm.arr + 1U) / 2U;
    tm.instance->CCR1 = zero_duty;
    tm.instance->CCR2 = zero_duty;
    tm.instance->CCR3 = zero_duty;
    // current_bias_calibration_start
    calibration.samples = 0;
    calibration.sum_a = 0;
    calibration.sum_b = 0;
    calibration.calibrating = true;
    // Start TIM1 once. This also starts
    // the synchronized ADC injected sampling.
    timer_start(&tm);
    while (!(calibration.samples >= CURRENT_BIAS_SAMPLES)) { __WFI(); }
    calibration.calibrating = false;
    // current_bias_calibration_finish
    const float avg_a = (float)calibration.sum_a / (float)calibration.samples;
    const float avg_b = (float)calibration.sum_b / (float)calibration.samples;
    calibration.bias_a = (avg_a / ADC_MAXCNT) * ADC_VREF;
    calibration.bias_b = (avg_b / ADC_MAXCNT) * ADC_VREF;
    LOG << " current bias A: " << calibration.bias_a << "v";
    LOG << " current bias B: " << calibration.bias_b << "v";
}

inline void update_current_calibration_sample(uint16_t raw_a, uint16_t raw_b) {
    // zero-current calibration
    if (calibration.calibrating) {
        calibration.sum_a += raw_a;
        calibration.sum_b += raw_b;
        ++calibration.samples;
    }
}

inline void initialize_encoder_timer(pfn_tim_encoder_callback_t callback) {
    // Configure encoder reads via TIM2
    timer_config_t& tim2 = hw.encoder_timer;
    tim2.instance = TIM2;
    tim2.mode = cms::ea;
    tim2.interrupt_callback = callback;
    // Configure TIM2 and make its counter clock 1 MHz
    timer_init(&tim2);
    // Configure update frequency = 1kHz
    timer_set_frequency(&tim2, 1000);
    // Register/enable TIM2 update interrupt
    timer_enable_interrupt(&tim2);
    // Start TIM2
    timer_start(&tim2);
}

#endif
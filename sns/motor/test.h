#include <memory>

inline void test_m200() {
    auto m200 = mcl::initialize_m200(PWM_PIN_MOTOR, PWM_FREQ_MOTOR);
    while (!getInstance<config>()->shouldExit()) {
        LOG << "looping...1";
        m200->set_duty_cycle(getInstance<config>()->getKeyValue(config::key::motor) / 20.0);
        mcl::sleep_ms(1000);
    }
}

inline void test_blhS() {
    auto blhS = mcl::initialize_blheliS(PWM_PIN_MOTOR, PWM_FREQ_MOTOR, false);
    while (true) {
        LOG << "blhS looping... 1.1";
        blhS->set_duty_cycle(1.1 / 20);
        mcl::sleep_ms(3000);
        LOG << "blhS looping ... 1.2";
        blhS->set_duty_cycle(1.2 / 20);
        mcl::sleep_ms(3000);
        LOG << "blhS looping ... 1.3";
        blhS->set_duty_cycle(1.3 / 20);
        mcl::sleep_ms(3000);
    }
}

inline void test_servo_m200() {
    auto m200 = mcl::initialize_m200(PWM_PIN_MOTOR, PWM_FREQ_MOTOR);
    auto rudder = mcl::initialize_rudder(PWM_PIN_RUD_SERVO, PWM_FREQ_SERVO);
    while (true) {
        m200->set_duty_cycle(1.6 / 20.0);
        rudder->set_duty_cycle(2.0 / 20.0);
        mcl::sleep_ms(500);
        m200->set_duty_cycle(1.6 / 20.0);
        rudder->set_duty_cycle(1.5 / 20.0);
        mcl::sleep_ms(500);
        m200->set_duty_cycle(1.6 / 20.0);
        rudder->set_duty_cycle(1.0 / 20.0);
        mcl::sleep_ms(500);
    }
    m200->set_duty_cycle(1.5 / 20.0);
}

inline void test_servo_drv8833_motor() {
    mcl::pwm _pwm_servo(0, PWM_FREQ_SERVO);
    mcl::drv8833 motor(14, 15, PWM_FREQ_MOTOR_DRV8833);
    printf("starting servo and motor...\n");
    motor.start();
    _pwm_servo.start();
    mcl::sleep_ms(5000);
    while (true) {
        // MG990 0-180 degrees
        motor.set_speed(95);
        _pwm_servo.set_duty_cycle(2.0 / 20.0);
        mcl::sleep_ms(1000);
        motor.set_speed(90);
        _pwm_servo.set_duty_cycle(1.5 / 20.0);
        mcl::sleep_ms(1000);
        motor.set_speed(85);
        _pwm_servo.set_duty_cycle(1.0 / 20.0);
        mcl::sleep_ms(1000);
        motor.set_speed(80);
        _pwm_servo.set_duty_cycle(0.5 / 20.0);
        mcl::sleep_ms(1000);
        _pwm_servo.set_duty_cycle(2.5 / 20.0);
        mcl::sleep_ms(1000);
    }
}

inline void test_sg90_servo() {
    // for servos the pwm frequency is ideally in a range of 40-200Hz. 50Hz frequency
    // implies a cycle(pulse) every 20ms. The servo angle is determined by the pulse
    // width in a 50 Hz PWM signal. Most servos move to 0 when they receive a pulse
    // 1500 µs long. Generally it is safe to send a servo a pulse in the range 1000 µs
    // to 2000 µs. Generally a 10 µs change in pulse width results in a 1 degree change
    // in angle. At some point you will reach the limit of rotation. That limit varies
    // between different makes and models of servos. If you try to force a servo beyond
    // its limits it will get very hot (possibly to destruction) and may strip its gears.
    // The small 9g servos generally have an extended angle range, 180 degrees or more.
    // Typically they accept pulse widths in the range 500 µs to 2500 µs. Determine a
    // servos limits carefully by experiment.
    mcl::pwm _pwm(0, PWM_FREQ_SERVO);
    mcl::sleep_ms(5000);
    printf("starting servo...\n");
    _pwm.start();
    while (!getInstance<config>()->shouldExit()) {
        LOG << "test servo looping...5";
        // _pwm.set_duty_cycle(2.6 / 20.0);
        // mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(2.4 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(2.2 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(2.0 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(1.8 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(1.6 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(1.5 / 20.0); // center
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(1.4 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(1.2 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(1.0 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(0.8 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(0.6 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(0.4 / 20.0);
        mcl::sleep_ms(1000);
    }
}

inline void test_motor_drv8833() {
    mcl::drv8833 motor(14, 15, 20000);
    motor.start();
    // brushed motor might have a differnt
    // starting duty cycle in reverse direction
    mcl::sleep_ms(1000);
    while (1) {
        motor.set_speed(95);
    }
    // mcl::sleep_ms(1500);
    // motor.set_speed(90);
    // mcl::sleep_ms(1500);
    // motor.set_speed(85);
    // mcl::sleep_ms(1500);
    // motor.set_speed(80);
    // mcl::sleep_ms(1500);
    // motor.set_speed(75);
    // mcl::sleep_ms(1500);
    // motor.set_speed(70);
    // mcl::sleep_ms(1500);
    // motor.set_speed(65);
    // mcl::sleep_ms(1500);
    // motor.set_speed(60);
    // mcl::sleep_ms(1500);
    // motor.set_speed(55);
    // mcl::sleep_ms(1500);
    // motor.set_speed(50);
    // mcl::sleep_ms(1500);
    // motor.set_speed(45);
    // mcl::sleep_ms(3000);
    // motor.set_direction(false);
    // mcl::sleep_ms(1500);
    // motor.set_speed(50);
}

inline void test_28BYJ_48_stepper() {
    #if defined PICO
    const int in[4] = {2, 3, 4, 5};
    std::array<std::array<uint32_t, 4>, 8>
        sequence = {{
            {1, 0, 0, 0},
            {1, 1, 0, 0},
            {0, 1, 0, 0},
            {0, 1, 1, 0},
            {0, 0, 1, 0},
            {0, 0, 1, 1},
            {0, 0, 0, 1},
            {1, 0, 0, 1}
        }};
    for (auto& e : in) {
        gpio_init(e);
        gpio_set_dir(e, GPIO_OUT);
    }
    while (true) {
        for (const auto& step : sequence) {
            for (auto i = 0; i < 4; i++) {
                gpio_put(in[i], step[i]);
            }
            sleep_ms(10);
        }
        // for (const auto& step : stepSequence | std::views::reverse) {
        //     for (auto i = 0; i < 4; i++) {
        //         gpio_put(in[i], step[i]);
        //     }
        //     sleep_ms(10);
        // }
    }
    #endif
}

#if defined (STM32)
inline void test_bldc_sinusoidal_wave() {
    // on nucleo-64 boards PA2 PA3
    // are used by stlink USART
    // use PB timer AF instead
    timer_config_t t2{};
    t2.instance = TIM2;
    timer_init(&t2);
    timer_set_frequency(&t2, 50);
    static const uint8_t pins[] = {8, 9, 10};
    timer_init_gpio(GPIOB, pins, 3, 1); // AF1
    // TIM2 CH1 -> PB8
    timer_init_channel(&t2, 1, GPIOB, 8, nullptr, 0);
    timer_set_duty_cycle(&t2, 1, 0.0f);
    timer_start_channel(&t2, 1, false);
    // TIM2 CH2 -> PB9
    timer_init_channel(&t2, 2, GPIOB, 9, nullptr, 0);
    timer_set_duty_cycle(&t2, 2, 0.0f);
    timer_start_channel(&t2, 2, false);
    // TIM2 CH3 -> PB10
    timer_init_channel(&t2, 3, GPIOB, 10, nullptr, 0);
    timer_set_duty_cycle(&t2, 3, 0.0f);
    timer_start_channel(&t2, 3, false);
    // Start the timer
    timer_start(&t2);
    while (true){mcl::sleep_ms(100);}
}

// phases A, B and C same as
// phases U, V and W phases
typedef struct {
    uint8_t HA;
    uint8_t LA;
    uint8_t HB;
    uint8_t LB;
    uint8_t HC;
    uint8_t LC;
} step;

step comm_table[6] = {
    {1, 0, 0, 1, 0, 0},  // step 1: A+ B-
    {1, 0, 0, 0, 0, 1},  // step 2: A+ C-
    {0, 0, 1, 0, 0, 1},  // step 3: B+ C-
    {0, 1, 1, 0, 0, 0},  // step 4: B+ A-
    {0, 1, 0, 0, 1, 0},  // step 5: C+ A-
    {0, 0, 0, 1, 1, 0},  // step 6: C+ B-
};

inline void apply_step_pwm(step s, float duty, timer_config_t &timer, bool complementary = false) {
    if (!complementary) {
        GPIOA->BSRR =
            (s.LA ? GPIO_BSRR_BS1 : GPIO_BSRR_BR1) |
            (s.LB ? GPIO_BSRR_BS3 : GPIO_BSRR_BR3) |
            (s.LC ? GPIO_BSRR_BS5 : GPIO_BSRR_BR5);
    }
    timer_set_duty_cycle(&timer, 1, s.HA ? duty : 0.0f);
    timer_set_duty_cycle(&timer, 2, s.HB ? duty : 0.0f);
    timer_set_duty_cycle(&timer, 3, s.HC ? duty : 0.0f);
}

inline void apply_step_ll(step s) {
    GPIOA->BSRR =
        (s.HA ? GPIO_BSRR_BS0 : GPIO_BSRR_BR0) |
        (s.LA ? GPIO_BSRR_BS1 : GPIO_BSRR_BR1) |
        (s.HB ? GPIO_BSRR_BS2 : GPIO_BSRR_BR2) |
        (s.LB ? GPIO_BSRR_BS3 : GPIO_BSRR_BR3) |
        (s.HC ? GPIO_BSRR_BS4 : GPIO_BSRR_BR4) |
        (s.LC ? GPIO_BSRR_BS5 : GPIO_BSRR_BR5);
}

// LL on both HS and LS, manual dead time insertion
inline void test_bldc_trapezoidal_ll() {
    // Enable clock for the GPIOA
    mcl::enableClockForGpio(GPIOA);
    // Clear mode bits for PA0 to PA5 first
    GPIOA->MODER &= ~(
        (3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (2 * 2)) |
        (3 << (3 * 2)) | (3 << (4 * 2)) | (3 << (5 * 2)));
    // Set PA0 to PA5 as general purpose output (UL, UH, VL, VH, WL, WH)
    GPIOA->MODER |= (
        (1 << (0 * 2)) | (1 << (1 * 2)) | (1 << (2 * 2)) |
        (1 << (3 * 2)) | (1 << (4 * 2)) | (1 << (5 * 2)));
    // Optional: Set as push-pull, low-speed (default)
    uint8_t step = 0;
    while (true) {
        // All off
        apply_step_ll({0, 0, 0, 0, 0, 0});
        // Dead time
        mcl::delay_us(2);
        // Next commutation step
        apply_step_ll(comm_table[step]);
        // Hold for the motor to react
        mcl::sleep_ms(5);
        step = (step + 1) % 6;
    }
}

// PWM on the HS, LL on the LS, manual dead time insertion
inline void test_bldc_trapezoidal_pwm() {
    // Enable clock for LS pins
    mcl::enableClockForGpio(GPIOA);
    // Clear mode bits for PA1, PA3, PA5 as GPIO outputs (LU, LV, LW)
    GPIOA->MODER &= ~((3 << (1 * 2)) | (3 << (3 * 2)) | (3 << (5 * 2)));
    // Set PA1, PA3, PA5 as GP outputs (LU, LV, LW)
    GPIOA->MODER |=  ((1 << (1 * 2)) | (1 << (3 * 2)) | (1 << (5 * 2)));
    // Configure TIM4 CH1-CH3 on PB6, PB7, PB8 as High-Side PWM outputs (HU, HV, HW)
    timer_config_t tm{};
    tm.instance = TIM4;
    timer_init(&tm);
    // 20KHz BLDC frequency
    timer_set_frequency(&tm, 20'000);

    static const uint8_t pins[] = {6, 7, 8};
    timer_init_gpio(GPIOB, pins, 3, 2); // AF2
    // TIM4 CH1 PB6
    timer_init_channel(&tm, 1, GPIOB, 6, nullptr, 0);
    timer_set_duty_cycle(&tm, 1, 0.0f);
    timer_start_channel(&tm, 1, false);
    // TIM4 CH2 PB7
    timer_init_channel(&tm, 2, GPIOB, 7, nullptr, 0);
    timer_set_duty_cycle(&tm, 2, 0.0f);
    timer_start_channel(&tm, 2, false);
    // TIM4 CH3 PB8
    timer_init_channel(&tm, 3, GPIOB, 8, nullptr, 0);
    timer_set_duty_cycle(&tm, 3, 0.0f);
    timer_start_channel(&tm, 3, false);
    // Start the timer
    timer_start(&tm);
    int step = 0;
    float duty = 0.90f;
    while (!getInstance<config>()->shouldExit()) {
        // All off
        apply_step_pwm({0,0,0,0,0,0}, 0.0f, tm);
        // Dead time
        mcl::delay_us(2);
        // Next commutation step
        duty = 0.90; //getInstance<config>()->getKeyValue(config::key::motor);
        apply_step_pwm(comm_table[step], duty, tm);
        // Hold for the motor to react
        mcl::sleep_ms(5);
        step = (step + 1) % 6;
    }
    apply_step_pwm({0,0,0,0,0,0}, 0.0f, tm);
    timer_stop(&tm);
}

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

// Complementary PWM on LS and HS with harware dead time insertion
inline void test_bldc_trapezoidal_pwm_comp() {
    timer_config_t tm{};
    tm.instance = TIM1;
    timer_init(&tm);
    // 20KHz BLDC frequency
    timer_set_frequency(&tm, 20'000);
    static const uint8_t hs_pins[] = {8, 9, 10};
    static const uint8_t ls_pins[] = {13, 14, 15};
    // 8, 9, 10 --> CH1, CH2, CH3
    timer_init_gpio(GPIOA, hs_pins, 3, 1); // AF1
    // 13, 14, 15 --> CH1N, CH2N, CH3N
    timer_init_gpio(GPIOB, ls_pins, 3, 1); // AF1
    // Set dead time of 250ns
    // TMC6300 has BBM (Break before Make)
    // internal hardware dead time so we really
    // dont need external dead time setup via mcu
    //timer_set_dead_time(&tm, 250);
    // initialize all channels
    timer_init_channel(&tm, 1, GPIOA, 8, GPIOB, 13);
    timer_set_duty_cycle(&tm, 1, 0.0f);
    timer_start_channel(&tm, 1, true);

    timer_init_channel(&tm, 2, GPIOA, 9, GPIOB, 14);
    timer_set_duty_cycle(&tm, 2, 0.0f);
    timer_start_channel(&tm, 2, true);

    timer_init_channel(&tm, 3, GPIOA, 10, GPIOB, 15);
    timer_set_duty_cycle(&tm, 3, 0.0f);
    timer_start_channel(&tm, 3, true);
    // Start the timer
    timer_start(&tm);

    ramp_profile_t ramp{};
    // Electrical frequency:
    // 5 Hz -> 30 Hz electrical
    ramp.ef_start = 5.0f;
    ramp.ef_end   = 30.0f;
    // Unused in trapezoidal mode.
    ramp.m_start = 0.0f;
    ramp.m_end   = 0.0f;
    // PWM duty:
    // 15% -> 75%
    ramp.trap_duty_start = 0.15f;
    ramp.trap_duty_end   = 0.75f;
    // Ramp duration.
    ramp.duration_s = 8.0f;
    const uint32_t ramp_start_ms = mcl::time_ms();
    uint32_t last_commutation_ms = ramp_start_ms;
    int step = 0;
    // ramp duty and electrical frequency simultaneously:
    //     duty:
    //         trap_duty_start -> trap_duty_end
    //     electrical frequency:
    //         ef_start -> ef_end
    // The six-step commutation period is derived from the
    // instantaneous electrical frequency:
    //     Tstep = 1 / (6 * electrical_frequency)
    while (!getInstance<config>()->shouldExit()) {
        // Sleep until an interrupt wakes us.
        // SysTick is active, so the foreground loop does not busy-wait.
        __WFI();
        uint32_t now_ms = mcl::time_ms();
        float elapsed_s =
            (float)(now_ms - ramp_start_ms) * 0.001f;
        // Ramp duty and electrical frequency from the SAME elapsed time.
        float duty =
            ramp_value(
                ramp.trap_duty_start,
                ramp.trap_duty_end,
                ramp.duration_s,
                elapsed_s);
        float electrical_frequency =
            ramp_value(
                ramp.ef_start,
                ramp.ef_end,
                ramp.duration_s,
                elapsed_s);
        // Six-step commutation period.
        // One electrical revolution contains six commutation steps:
        //     Tstep = 1 / (6 * electrical_frequency)
        // Convert to milliseconds.
        if (electrical_frequency > 0.0f) {
            float step_period_ms =
                1000.0f / (6.0f * electrical_frequency);
            uint32_t step_period_ms_u = (uint32_t)step_period_ms;
            // Avoid a zero-period loop at very high frequency.
            if (step_period_ms_u < 1U)
                step_period_ms_u = 1U;
            if ((uint32_t)(now_ms - last_commutation_ms) >= step_period_ms_u) {
                // Keep the schedule relative to the previous
                // commutation event rather than "now".
                last_commutation_ms += step_period_ms_u;
                apply_step_pwm(
                    comm_table[step],
                    duty,
                    tm,
                    true);
                step = (step + 1) % 6;
            }
        }
    }
    apply_step_pwm({0,0,0,0,0,0}, 0.0f, tm, true);
    timer_stop(&tm);
}

inline void init_current_sampling() {
    // Enable clock for GPIOA (sampling pin)
    mcl::enableClockForGpio(GPIOA);
    // PA0 -> Analog mode
    GPIOA->MODER &= ~(3UL << (0 * 2));
    GPIOA->MODER |=  (3UL << (0 * 2));
    // No pull-up/pull-down
    GPIOA->PUPDR &= ~(3UL << (0 * 2));
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
    // ============================================================
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
    // ============================================================
    // ADC1
    // PA0 -> ADC1_IN0 -> JDR1
    // PA1 -> ADC1_IN1 -> JDR2
    // Two injected conversions per TIM1 CH4 trigger.
    // ============================================================
    constexpr uint32_t PHASE_A = ADC_CH0;
    constexpr uint32_t PHASE_B = ADC_CH1;
    constexpr uint32_t SAMPLE_TIME = ADC_SAMPLE_15;
    // ------------------------------------------------------------
    // ADC sample time
    // ------------------------------------------------------------
    ADC1->SMPR2 &= ~(
        (7UL << (PHASE_A * 3U)) |
        (7UL << (PHASE_B * 3U))
    );
    ADC1->SMPR2 |=
        (SAMPLE_TIME << (PHASE_A * 3U)) |
        (SAMPLE_TIME << (PHASE_B * 3U));
    // ------------------------------------------------------------
    // 12-bit, right aligned
    // ------------------------------------------------------------
    ADC1->CR1 &= ~(3UL << 24);
    ADC1->CR2 &= ~ADC_CR2_ALIGN;
    ADC1->CR1 |= ADC_CR1_SCAN;
    // ------------------------------------------------------------
    // Injected sequence
    // JSQ1 = phase A
    // JSQ2 = phase B
    // JL = 1 => sequence contains 2 conversions.
    // ------------------------------------------------------------
    ADC1->JSQR = 0;
    ADC1->JSQR |= (PHASE_A << ADC_JSQR_JSQ3_Pos);
    ADC1->JSQR |= (PHASE_B << ADC_JSQR_JSQ4_Pos);
    ADC1->JSQR |= ADC_JSQR_JL_0;
    // ------------------------------------------------------------
    // TIM1_CH4 -> injected trigger
    // STM32F446:
    // JEXTSEL = 0000 -> TIM1_CH4
    // JEXTEN  = 01   -> rising edge
    // ------------------------------------------------------------
    ADC1->CR2 &= ~(ADC_CR2_JEXTSEL | ADC_CR2_JEXTEN);
    // JEXTSEL = 0000 => TIM1_CH4
    // Rising-edge trigger
    ADC1->CR2 |= ADC_CR2_JEXTEN_0;
    // ------------------------------------------------------------
    // Interrupt after the injected sequence completes.
    // This occurs after JDR1 and JDR2 have both been filled.
    // ------------------------------------------------------------
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

constexpr float ADC_VREF   = 3.3f;
constexpr float ADC_MAXCNT = 4095.0f;
constexpr float CSA_GAIN   = 50.0f;      // V/V, INA240A2
constexpr float R_SHUNT    = 0.010f;     // 10 mill ohms
constexpr float V_BIAS     = 2.5f;       // bi-directional mid point confirmed by 7semi support

inline float phase_a_current_amps() {
    float v_adc = ((float)g_phase_a_adc / ADC_MAXCNT) * ADC_VREF;
    return (v_adc - V_BIAS) / (CSA_GAIN * R_SHUNT);
}

inline void test_bldc_svpwm() {
    timer_config_t tm{};
    tm.instance = TIM1;
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
    ramp.ef_start = 2.0f;
    ramp.ef_end   = 3.0f; //ok:7, notok:40
    // Trapezoidal fields are unused here.
    ramp.trap_duty_start = 0.0f;
    ramp.trap_duty_end   = 0.0f;
    // Both modulation and frequency use this same duration.
    ramp.duration_s = 8.0f;
    // Initial SVPWM state.
    g_svpwm.modulation = ramp.m_start;
    g_svpwm.electrical_frequency = ramp.ef_start;

    timer_enable_interrupt(&tm);

    const uint32_t ramp_start_ms = mcl::time_ms();
    timer_start(&tm);
    uint32_t last_ramp_update_ms = ramp_start_ms;

    while (!getInstance<config>()->shouldExit()) {
        // TIM1 update interrupt and SysTick can wake the processor.
        // TIM1 ISR:
        //     svpwm_update(&g_svpwm);
        // SysTick:
        //     advances mcl::time_ms()
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
        // current sense
        static uint32_t last_log_ms = 0;
        if (g_current_sample_ready) {
            g_current_sample_ready = false;
            if ((uint32_t)(now_ms - last_log_ms) >= 20U) {   // ~50 Hz log rate
                float ia = phase_a_current_amps();
                LOG << " Ia: " << ia << ", raw(a): " << g_phase_a_adc
                    << ", raw(b): " << g_phase_b_adc;
                last_log_ms = now_ms;
            }
        }
    }
    timer_stop(&tm);
}

#elif defined (PICO)
void test_bldc_trapezoidal_ll() {}
void test_bldc_sinusoidal_wave() {}
void test_bldc_trapezoidal_pwm() {}
void test_bldc_trapezoidal_pwm_comp() {}
void test_svpwm() {}
void test_bldc_foc() {}
#endif
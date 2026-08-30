#ifndef FOC_H
#define FOC_H

#include <foc/svpwm.h>
#include <foc/encoder.h>
#include <foc/current.h>
#include <foc/speed.h>

inline svpwm_t g_svm;
enum foc_state { stopped, running, fault };
foc_state state{ stopped };

inline void foc_voltage_apply(current_control_t& cc, float electrical_angle) {
    inverse_park_transform(cc.vd, cc.vq, electrical_angle);
    // pi outputs are in volts; svpwm
    // expects Vref normalized to Vbus.
    pt.v_alpha *= cc.inv_vbus;
    pt.v_beta  *= cc.inv_vbus;
    svpwm_update(g_svm, pt.v_alpha, pt.v_beta);
}

inline void adc_injected_callback(uint16_t raw_a, uint16_t raw_b) {
    // zero-current calibration
    if (cs.calibrating) {
        cs.calibration_sum_a += raw_a;
        cs.calibration_sum_b += raw_b;
        ++cs.calibration_samples;
        return;
    }
    // Disable FOC during encoder/zero current calibration
    if (state != foc_state::running) return;
    // Current control
    // ADC counts to phase currents
    current_sense_update(cs, raw_a, raw_b);

    const encoder_state_t* enc = encoder_read;
    const auto predicted_electrical_angle =
            encoder_predict_electrical_angle(*enc);
    static uint16_t speed_loop_divider = 0;
    if (++speed_loop_divider >= 200) {
        speed_loop_divider = 0;
        constexpr float SPEED_LOOP_DT = 1.0f / 100.0f;
        speed_control_update(
            sc, enc->mechanical_velocity, SPEED_LOOP_DT);
    }
    constexpr float CURRENT_LOOP_DT = 1.0f / 20'000.0f;
    current_control_update(
        cc, cs, predicted_electrical_angle,
            enc->electrical_velocity, CURRENT_LOOP_DT);
    // voltage to pwm
    foc_voltage_apply(cc, predicted_electrical_angle);
}

serial::i2c *i2cbus = nullptr;
void tim2_encoder_callback(timer_event_t event) {
    if (event == TIMER_EVENT_UPDATE) {
        // executes at 1kHz
        encoder_read_and_update_angles(*i2cbus);
    }
}

inline void test_foc() {

    timer_config_t tm{};
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
    //timer_set_dead_time(&tm, 250);
    timer_init_channel(&tm, 1, GPIOA, 8, GPIOB, 13);
    timer_init_channel(&tm, 2, GPIOA, 9, GPIOB, 14);
    timer_init_channel(&tm, 3, GPIOA, 10, GPIOB, 15);
    // start timer channels
    timer_start_channel(&tm, 1, true);
    timer_start_channel(&tm, 2, true);
    timer_start_channel(&tm, 3, true);

    // CH4 trigger + ADC1 injected sequence
    init_current_sampling();
    ADC_Config_t cfg{};
    cfg._instance = ADC1;
    cfg._interrupt_callback_regular = nullptr;
    cfg._interrupt_callback_injected = adc_injected_callback;
    adc_set_config(&cfg);

    g_svm.timer = &tm;
    state = foc_state::stopped;

    // Zero current ADC bias calibration
    constexpr uint32_t CURRENT_BIAS_SAMPLES = 1000;
    // No average motor voltage.
    // Keep all three PWM phases at 50% duty while the ADC
    // continues to be triggered by TIM1 CH4.
    const uint32_t zero_duty = (tm.arr + 1U) / 2U;
    tm.instance->CCR1 = zero_duty;
    tm.instance->CCR2 = zero_duty;
    tm.instance->CCR3 = zero_duty;
    current_bias_calibration_start();
    // Start TIM1 once. This also starts
    // the synchronized ADC injected sampling.
    timer_start(&tm);
    while (!current_bias_calibration_complete(CURRENT_BIAS_SAMPLES)) { __WFI(); }
    current_bias_calibration_finish();

    // Encoder electrical-zero calibration
    serial::i2c bus(3, 10, 400'000, I2C2, GPIOB);
    svpwm_t calibration_svm{};
    calibration_svm.timer = &tm;
    auto rc = calibrate_encoder(bus, calibration_svm, cc.vbus);
    if (!rc) return;

    // TIM2 encoder read setup
    i2cbus = &bus;
    timer_config_t tim2_cfg = {};
    tim2_cfg.instance = TIM2;
    tim2_cfg.mode = cms::ea;
    tim2_cfg.interrupt_callback = tim2_encoder_callback;
    // Configure TIM2 and make its counter clock 1 MHz
    timer_init(&tim2_cfg);
    // Configure update frequency = 1kHz
    timer_set_frequency(&tim2_cfg, 1000);
    // Register/enable TIM2 update interrupt
    timer_enable_interrupt(&tim2_cfg);
    // Start TIM2
    timer_start(&tim2_cfg);

    // With vbus = 9.49 V:
    // v_limit = 9.49 / sqrt(3) ≈ 5.48 V
    // so each PI can request up to ±5.48 V.
    const float v_limit = SVPWM_MAX_MODULATION * cc.vbus;
    cc.d_pi = { .kp = Kp, .ki = Ki, .integrator = 0.0f, .out_min = -v_limit, .out_max = v_limit };
    cc.q_pi = { .kp = Kp, .ki = Ki, .integrator = 0.0f, .out_min = -v_limit, .out_max = v_limit };
    sc.pi   = { .kp = 0.015f, .ki = 0.001f, .integrator = 0.0f, .out_min = -0.20f, .out_max = 0.20f };

    // manual hold delay
    mcl::delay_ms(2000);
    current_loop_reset(cc);
    speed_loop_reset(sc);

    state = foc_state::running;

    uint64_t total_cycles = 0;
    uint32_t last_cycles = DWT->CYCCNT;

    uint64_t last_log_cycles = 0;
    uint64_t log_period_cycles = (uint64_t)SystemCoreClock / 2U;

    // cc.d_ref = 0.0f;
    // cc.q_ref = 0.3f;
    constexpr float SPEED_START = 10.0f;
    float SPEED_END = 2.0f;
    constexpr float SPEED_RAMP_DURATION_S = 15.0f;
    sc.speed_ref = SPEED_START;
    float inv_SystemCoreClock = 1 / (float)SystemCoreClock;
    uint32_t last_ramp_ms = mcl::time_ms();

    while (true) {
        uint32_t now_cycles = DWT->CYCCNT;
        total_cycles += (uint32_t)(now_cycles - last_cycles);
        last_cycles = now_cycles;
        float elapsed_s = (float)total_cycles * inv_SystemCoreClock;
        uint32_t now_ms = mcl::time_ms();
        if ((uint32_t)(now_ms - last_ramp_ms) >= 1U) {
            sc.speed_ref = ramp_linear(
                SPEED_START,
                SPEED_END,
                SPEED_RAMP_DURATION_S,
                elapsed_s);
            last_ramp_ms = now_ms;
        }

        // Trace capture: arm once settled at the low-speed hold,
        // dump once the buffer fills.
        static bool trace_armed = false;
        if (!trace_armed && elapsed_s >= 32.0f) {
            trace_start();
            trace_armed = true;
        }
        if (g_trace_full) {
            trace_dump();
            g_trace_full = false;
        }

        if (total_cycles - last_log_cycles >= log_period_cycles) {
            last_log_cycles = total_cycles;
            log_foc_state(elapsed_s);
        }
    }

    timer_stop(&tm);
}

inline void log_foc_state(float elapsed_s) {
    const encoder_state_t* encoder = encoder_read;
    char log_buffer[512];
    int len = snprintf(log_buffer, sizeof(log_buffer),
        "theta:%f elapsed:%f s_ref:%f s_mes:%f "
        "[ia:%f ib:%f ic:%f] q_ref:%fA [d:%f q:%f] vd:%f vq:%f mod:%f d_i:%f q_i:%f s_i:%f",
            encoder->electrical_angle, elapsed_s, sc.speed_ref, sc.speed_measured,
                cs.ia, cs.ib, cs.ic, cc.q_ref, pt.d, pt.q, cc.vd, cc.vq,
                    g_svm.modulation, cc.d_pi.integrator, cc.q_pi.integrator,
                        sc.pi.integrator);
    if (len > 0) {
        if (len >= sizeof(log_buffer)) len = sizeof(log_buffer) - 1;
        LOG << std::string(log_buffer, len);
    }
}

#endif
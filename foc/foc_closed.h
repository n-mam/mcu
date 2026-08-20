#ifndef FOC_CLOSED_H
#define FOC_CLOSED_H

#include <foc/encoder.h>
#include <foc/current.h>
#include <foc/foc_common.h>

inline svpwm_t g_svm;

inline volatile bool current_loop_enabled = false;
inline volatile bool encoder_calibration_hold = true;

inline void foc_voltage_apply(current_control_t& cc, float electrical_angle) {
    inverse_park_transform(cc.vd, cc.vq, electrical_angle);
    // pi outputs are in volts; svpwm
    // expects Vref normalized to Vbus.
    pt.v_alpha /= cc.vbus;
    pt.v_beta  /= cc.vbus;
    svpwm_update(g_svm, pt.v_alpha, pt.v_beta);
}

inline void adc_callback_injected(uint16_t raw_a, uint16_t raw_b) {
    // zero-current calibration
    if (cs.calibrating) {
        cs.calibration_sum_a += raw_a;
        cs.calibration_sum_b += raw_b;
        ++cs.calibration_samples;
        return;
    }
    // Don't run FOC during encoder calibration
    if (encoder_calibration_hold) return;
    // ADC counts to phase currents
    current_sense_update(cs, raw_a, raw_b);
    // Current control
    if (!current_loop_enabled) return;
    constexpr float CURRENT_LOOP_DT = 1.0f / 20'000.0f;
    foc_current_update(cc, cs, encoder.electrical_angle, CURRENT_LOOP_DT);
    // voltage to PWM
    foc_voltage_apply(cc, encoder.electrical_angle);
}

inline void test_foc_closed_loop() {
    timer_config_t tm{};
    tm.instance = TIM1;
    tm.interrupt_callback = nullptr;
    tm.mode = cms::ca1;
    timer_init(&tm);
    timer_set_frequency(&tm, 20000);
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
    cfg._interrupt_callback_injected = adc_callback_injected;
    adc_set_config(&cfg);

    g_svm.timer = &tm;
    current_loop_enabled = false;
    encoder_calibration_hold = true;
    // ZERO-CURRENT ADC BIAS CALIBRATION
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
    LOG << " current bias A: " << cs.bias_a << "v";
    LOG << " current bias B: " << cs.bias_b << "v";

    // Encoder electrical-zero calibration
    serial::i2c bus(3, 10, 400'000, I2C2, GPIOB);
    svpwm_t calibration_svm{};
    calibration_svm.timer = &tm;
    calibrate_encoder(bus, calibration_svm);
    encoder_calibration_hold = false;

    // With vbus = 9.49 V:
    // v_limit = 9.49 / sqrt(3) ≈ 5.48 V
    // so each PI can request up to ±5.48 V.
    const float v_limit = SVPWM_MAX_MODULATION * cc.vbus;
    cc.d_pi = { .kp = 5.0f, .ki = 0.0f, .integrator = 0.0f, .out_min = -v_limit, .out_max = v_limit };
    cc.q_pi = { .kp = 5.0f, .ki = 0.2f, .integrator = 0.0f, .out_min = -v_limit, .out_max = v_limit };

    uint64_t total_cycles = 0;
    uint32_t last_log_us = 0;
    uint32_t last_cycles = DWT->CYCCNT;
    constexpr uint32_t log_period_us = 80'000U;

    //mcl::delay_ms(5000);
    current_loop_reset(cc);
    current_loop_enabled = true;

    while (true) {
        uint32_t now_cycles = DWT->CYCCNT;
        total_cycles += (uint32_t)(now_cycles - last_cycles);
        last_cycles = now_cycles;
        float elapsed_s = (float)total_cycles / (float)SystemCoreClock;
        uint32_t now_us = (uint32_t)(elapsed_s * 1e6f);
        cc.q_ref = 0.10f;
        encoder_read_and_update_angles(bus);
        // log once every 10 seconds
        //if ((uint32_t)(now_us - last_log_us) >= log_period_us) {
            last_log_us = now_us;
            log_foc_state(elapsed_s);
        //}
    }

    timer_stop(&tm);
}

inline void log_foc_state(float elapsed_s) {
    char log_buffer[512];
    int len = snprintf(log_buffer, sizeof(log_buffer),
        "theta:%f elapsed:%f ph_a:%d ph_b:%d [ia:%f ib:%f ic:%f] iq_ref:%fA [a:%f b:%f] [d:%f q:%f] vd:%f vq:%f mod:%f d_i:%f q_i:%f",
            encoder.electrical_angle, elapsed_s, cs.raw_a, cs.raw_b,
                cs.ia, cs.ib, cs.ic, cc.q_ref, pt.alpha, pt.beta, pt.d, pt.q, cc.vd, cc.vq,
                    g_svm.modulation, cc.d_pi.integrator, cc.q_pi.integrator);
    if (len > 0) {
        if (len >= sizeof(log_buffer)) len = sizeof(log_buffer) - 1;
        LOG << std::string(log_buffer, len);
    }
}

#endif
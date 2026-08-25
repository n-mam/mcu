#ifndef FOC_H
#define FOC_H

#include <foc/encoder.h>
#include <foc/current.h>
#include <foc/svpwm.h>
#include <foc/speed.h>

inline svpwm_t g_svm;

inline volatile bool current_loop_hold = true;
inline volatile bool encoder_calibration_hold = true;

inline void foc_voltage_apply(current_control_t& cc, float electrical_angle) {
    inverse_park_transform(cc.vd, cc.vq, electrical_angle);
    // pi outputs are in volts; svpwm
    // expects Vref normalized to Vbus.
    pt.v_alpha /= cc.vbus;
    pt.v_beta  /= cc.vbus;
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
    // Don't run FOC during encoder calibration
    if (encoder_calibration_hold || current_loop_hold) return;
    // Current control
    // ADC counts to phase currents
    current_sense_update(cs, raw_a, raw_b);

    // Time elapsed since the last encoder sample.
    float encoder_age =
        (float)(DWT->CYCCNT - encoder.last_update_cycles)
            / (float)SystemCoreClock;

    // // Predict the electrical angle forward from the last encoder sample.
    float electrical_angle = encoder.electrical_angle; +
        encoder.electrical_velocity * encoder_age;

    // Wrap to [0, TWO_PI)
    electrical_angle = fmodf(electrical_angle, TWO_PI);

    if (electrical_angle < 0.0f) {
        electrical_angle += TWO_PI;
    }

    static uint8_t speed_loop_divider = 0;
    if (++speed_loop_divider >= 200) {
        speed_loop_divider = 0;
        constexpr float SPEED_LOOP_DT = 1.0f / 100.0f;
        speed_control_update(
            sc, encoder.mechanical_velocity, SPEED_LOOP_DT);
    }

    constexpr float CURRENT_LOOP_DT = 1.0f / 20'000.0f;
    current_control_update(cc, cs, electrical_angle, encoder.electrical_velocity, CURRENT_LOOP_DT);
    // voltage to PWM
    foc_voltage_apply(cc, electrical_angle);
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
    current_loop_hold = true;
    encoder_calibration_hold = true;

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
    calibrate_encoder(bus, calibration_svm);

    // Read the encoder at the final calibrated rotor position.
    encoder_read_and_update_angles(bus);
    // Start the open-loop electrical field from the
    // same electrical angle as the encoder.
    g_open_loop.angle = encoder.electrical_angle;
    // Discard any velocity-estimator state accumulated during calibration.
    reset_velocity_estimator();

    // With vbus = 9.49 V:
    // v_limit = 9.49 / sqrt(3) ≈ 5.48 V
    // so each PI can request up to ±5.48 V.
    const float v_limit = SVPWM_MAX_MODULATION * cc.vbus;
    cc.d_pi = { .kp = kp, .ki = ki, .integrator = 0.0f, .out_min = -v_limit, .out_max = v_limit };
    cc.q_pi = { .kp = kp, .ki = ki, .integrator = 0.0f, .out_min = -v_limit, .out_max = v_limit };
    sc.pi = { .kp = 0.01f, .ki = 0.002f, .integrator = 0.0f, .out_min = -0.3, .out_max = 0.3 };

    uint64_t total_cycles = 0;
    uint32_t last_cycles = DWT->CYCCNT;

    uint32_t last_log_us = 0;
    constexpr uint32_t log_period_us = 500'000U;

    // manual hold delay
    mcl::delay_ms(2500);
    current_loop_reset(cc);
    speed_loop_reset(sc);

    current_loop_hold = false;
    encoder_calibration_hold = false;
    uint32_t last_encoder_read_us = 0;
    while (true) {
        uint32_t now_cycles = DWT->CYCCNT;
        total_cycles += (uint32_t)(now_cycles - last_cycles);
        last_cycles = now_cycles;
        float elapsed_s = (float)total_cycles / (float)SystemCoreClock;
        uint32_t now_us = (uint32_t)(elapsed_s * 1e6f);
        //cc.q_ref = 0.025f;
        static float speed_command = 0;
        speed_command += 0.002f;
        if (speed_command > 10.0f)
            speed_command = 10.0f;
        sc.speed_ref = speed_command;
        if ((uint32_t)(now_us - last_encoder_read_us) >= ENCODER_READ_PERIOD_US) {
            last_encoder_read_us = now_us;
            encoder_read_and_update_angles(bus);
        }
        // log once every log_period_us
        if ((uint32_t)(now_us - last_log_us) >= log_period_us) {
            last_log_us = now_us;
            log_foc_state(elapsed_s);
            LOG << "raw:" << encoder.raw
                    << " vel:" << encoder.mechanical_velocity << " qref:" << cc.q_ref
                        << " sref:" << sc.speed_ref << " smes:" << sc.speed_measured;
        }
    }

    timer_stop(&tm);
}

inline void log_foc_state(float elapsed_s) {
    char log_buffer[512];
    int len = snprintf(log_buffer, sizeof(log_buffer),
        "theta:%f elapsed:%f s_ref:%f s_mes:%f "
        "[ia:%f ib:%f ic:%f] q_ref:%fA [d:%f q:%f] vd:%f vq:%f mod:%f d_i:%f q_i:%f s_i:%f",
            encoder.electrical_angle, elapsed_s, sc.speed_ref, sc.speed_measured,
                cs.ia, cs.ib, cs.ic, cc.q_ref, pt.d, pt.q, cc.vd, cc.vq,
                    g_svm.modulation, cc.d_pi.integrator, cc.q_pi.integrator,
                        sc.pi.integrator);
    if (len > 0) {
        if (len >= sizeof(log_buffer)) len = sizeof(log_buffer) - 1;
        LOG << std::string(log_buffer, len);
    }
}

#endif
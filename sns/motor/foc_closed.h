#ifndef FOC_CLOSED_H
#define FOC_CLOSED_H

#include <motor/foc_common.h>
#include <motor/encoder.h>

struct closed_loop_foc_t {
    float iq_start;
    float iq_end;
    float iq_ramp_duration_s;
};

inline svpwm_t g_svm;
inline current_loop_t g_current_loop;
inline closed_loop_foc_t g_closed_loop;
inline volatile float g_theta_measured = 0.0f;
inline volatile bool current_loop_enabled = false;
inline volatile bool encoder_calibration_hold = true;

inline volatile float g_iq_measured = 0.0f;
inline volatile float g_id_measured = 0.0f;
inline volatile uint16_t g_raw_a = 0;
inline volatile uint16_t g_raw_b = 0;

inline volatile float g_current_bias_a = 2.5f;
inline volatile float g_current_bias_b = 2.5f;
inline volatile bool g_current_bias_calibrating = false;
inline volatile uint32_t g_current_bias_samples = 0;
inline volatile uint64_t g_current_bias_sum_a = 0;
inline volatile uint64_t g_current_bias_sum_b = 0;
inline volatile float g_vd, g_vq;

inline volatile float g_theta = 0.0f;
inline volatile float g_i_alpha = 0.0f;
inline volatile float g_i_beta = 0.0f;
inline volatile float g_i_mag = 0.0f;
inline volatile float g_i_angle = 0.0f;
inline volatile float g_id_error = 0.0f;
inline volatile float g_iq_error = 0.0f;

inline void closed_loop_update(uint16_t raw_a, uint16_t raw_b, float theta, float dt) {

    if (!current_loop_enabled) return;
    phase_currents_t pc = compute_dq_currents(
            raw_a, raw_b, g_current_bias_a, g_current_bias_b, theta);

    g_raw_a = raw_a;
    g_raw_b = raw_b;
    g_theta = theta;
    g_i_alpha = pc.alpha;
    g_i_beta  = pc.beta;
    g_i_mag   = sqrtf(pc.alpha * pc.alpha + pc.beta * pc.beta);
    g_i_angle = atan2f(pc.beta, pc.alpha);
    if (g_i_angle < 0.0f) g_i_angle += TWO_PI;
    g_iq_measured = pc.q;
    g_id_measured = pc.d;

    float id_error = g_current_loop.id_ref - pc.d;
    float iq_error = g_current_loop.iq_ref - pc.q;
    float vd = pi_update(g_current_loop.id_pi, id_error, dt);
    float vq = pi_update(g_current_loop.iq_pi, iq_error, dt);

    g_vd = vd; g_vq = vq;
    g_id_error = id_error;
    g_iq_error = iq_error;

    // Inverse Park:
    // Vd/Vq -> Valpha/Vbeta
    float c = cosf(theta);
    float s = sinf(theta);
    float valpha = vd * c - vq * s;
    float vbeta = vd * s + vq * c;
    // PI outputs are volts.
    // SVPWM expects normalized Vref/Vbus.
    valpha /= g_current_loop.vbus;
    vbeta  /= g_current_loop.vbus;
    svpwm_update(g_svm, valpha, vbeta);
}

inline void adc_callback_injected(uint16_t phase_a, uint16_t phase_b) {
    // Zero-current bias calibration happens on the
    // synchronized injected ADC samples, before FOC is enabled.
    if (g_current_bias_calibrating) {
        g_current_bias_sum_a += phase_a;
        g_current_bias_sum_b += phase_b;
        ++g_current_bias_samples;
        return;
    }
    if (encoder_calibration_hold) return;
    // closed-loop FOC runs here.
    constexpr float CLOSED_LOOP_DT = 1.0f / 20'000.0f;
    closed_loop_update(phase_a, phase_b, g_theta_measured, CLOSED_LOOP_DT);
}

inline void current_bias_calibration_start() {
    g_current_bias_samples = 0;
    g_current_bias_sum_a = 0;
    g_current_bias_sum_b = 0;
    g_current_bias_calibrating = true;
}

inline bool current_bias_calibration_complete(uint32_t target_samples) {
    return g_current_bias_samples >= target_samples;
}

inline void current_bias_calibration_finish() {
    const uint32_t n = g_current_bias_samples;
    if (n == 0) {
        g_current_bias_calibrating = false;
        return;
    }
    const float avg_a =
        (float)g_current_bias_sum_a / (float)n;
    const float avg_b =
        (float)g_current_bias_sum_b / (float)n;
    g_current_bias_a =
        (avg_a / ADC_MAXCNT) * ADC_VREF;
    g_current_bias_b =
        (avg_b / ADC_MAXCNT) * ADC_VREF;
    g_current_bias_calibrating = false;
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
    // Start TIM1 once. This also starts the synchronized
    // ADC injected sampling.
    timer_start(&tm);
    while (!current_bias_calibration_complete(CURRENT_BIAS_SAMPLES)) {
        __WFI();
    }
    current_bias_calibration_finish();
    LOG << " current bias A: " << g_current_bias_a
            << " V, B: " << g_current_bias_b << " V";

    // Encoder electrical-zero calibration
    serial::i2c bus(3, 10, 400'000, I2C2, GPIOB);
    svpwm_t calibration_svm{};
    calibration_svm.timer = &tm;
    encoder_calibration_t encoder = calibrate_encoder(bus, calibration_svm);
    encoder_calibration_hold = false;
    //while(true) __WFI();

    constexpr float SVPWM_HEADROOM = SVPWM_MAX_MODULATION * 9.49f * 0.9f; // ~4.93V, true system ceiling with margin
    constexpr float PI_OUT_LIMIT = SVPWM_HEADROOM / 1.41421356f;          // ~3.49V per axis, worst case stays inside the real circle
    g_current_loop.id_pi = { .kp = 0.02f, .ki = 5.0f, .integrator = 0.0f, .out_min = -PI_OUT_LIMIT, .out_max = PI_OUT_LIMIT };
    g_current_loop.iq_pi = { .kp = 0.02f, .ki = 5.0f, .integrator = 0.0f, .out_min = -PI_OUT_LIMIT, .out_max = PI_OUT_LIMIT };

    current_loop_reset(g_current_loop);
    current_loop_enabled = true;
    // soft-start the torque command
    g_closed_loop.iq_start = 0.0f;
    g_closed_loop.iq_end = 0.01f;
    g_closed_loop.iq_ramp_duration_s = 8.0f;
    const uint32_t iq_ramp_start_ms = mcl::time_ms();
    uint32_t last_encoder_ms = 0;

    while (!getInstance<config>()->shouldExit()) {
        //__WFI();
        uint32_t now_ms = mcl::time_ms();
        float elapsed_s = (float)(now_ms - iq_ramp_start_ms) * 0.001f;
        g_current_loop.iq_ref =
            ramp_value(
                g_closed_loop.iq_start,
                g_closed_loop.iq_end,
                g_closed_loop.iq_ramp_duration_s,
                elapsed_s);
        if ((uint32_t)(now_ms - last_encoder_ms) >= 1U) {
            uint16_t raw = as5600_read_raw_angle(bus);
            g_theta_measured = encoder_to_electrical_angle(raw, encoder.zero_raw, encoder.sign);
            last_encoder_ms = now_ms;
            LOG
                << "theta=" << g_theta_measured
                << " ctrl=" << g_theta
                << " ia=" << g_i_alpha
                << " ib=" << g_i_beta
                << " Imag=" << g_i_mag
                << " Iang=" << g_i_angle
                << " id=" << g_id_measured
                << " iq=" << g_iq_measured
                << " iderr=" << g_id_error
                << " iqerr=" << g_iq_error
                << " vd=" << g_vd
                << " vq=" << g_vq
                << " mod=" << g_svm.modulation
                << " idi=" << g_current_loop.id_pi.integrator
                << " iqi=" << g_current_loop.iq_pi.integrator;
        }
    }

    timer_stop(&tm);
}

inline void test_as5600() {
    serial::i2c bus(3, 10, 400'000, I2C2, GPIOB);
    while(true) {
        uint16_t raw = as5600_read_raw_angle(bus);
        LOG << " raw: " << raw;
        mcl::sleep_ms(1);
    }
}

#endif
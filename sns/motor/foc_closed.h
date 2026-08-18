#ifndef FOC_CLOSED_H
#define FOC_CLOSED_H

#include <motor/foc_common.h>
#include <motor/encoder.h>

struct ramp_foc_t {
    float iq_start;
    float iq_end;
    float iq_ramp_duration_s;
};

inline svpwm_t g_svm;
inline ramp_foc_t g_ramp;
inline current_loop_t g_current_loop;
inline volatile bool current_loop_enabled = false;
inline volatile bool encoder_calibration_hold = true;

inline volatile uint16_t g_raw_a = 0;
inline volatile uint16_t g_raw_b = 0;

inline volatile float g_current_bias_a = 2.5f;
inline volatile float g_current_bias_b = 2.5f;
inline volatile bool g_current_bias_calibrating = false;
inline volatile uint32_t g_current_bias_samples = 0;
inline volatile uint64_t g_current_bias_sum_a = 0;
inline volatile uint64_t g_current_bias_sum_b = 0;

//trace
inline volatile float g_a = 0.0f;
inline volatile float g_b = 0.0f;
inline volatile float g_d = 0.0f;
inline volatile float g_q = 0.0f;
inline volatile float g_vd = 0.0f;
inline volatile float g_vq = 0.0f;
inline volatile float g_theta = 0.0f;
inline volatile float g_d_err = 0.0f;
inline volatile float g_q_err = 0.0f;
inline volatile float g_ab_mag = 0.0f;
inline volatile float g_ab_angle = 0.0f;

inline void closed_loop_update(
        uint16_t raw_a, uint16_t raw_b, float theta, float dt) {
    if (!current_loop_enabled) return;
    phase_currents_t pc = compute_dq_currents(
            raw_a, raw_b, g_current_bias_a, g_current_bias_b, theta);
    g_raw_a = raw_a;
    g_raw_b = raw_b;
    g_theta = theta;
    g_a = pc.alpha;
    g_b = pc.beta;
    g_d = pc.d;
    g_q = pc.q;
    g_ab_mag = sqrtf(pc.alpha * pc.alpha + pc.beta * pc.beta);
    g_ab_angle = atan2f(pc.beta, pc.alpha);
    if (g_ab_angle < 0.0f) g_ab_angle += TWO_PI;

    float d_error = g_current_loop.id_ref - pc.d;
    float q_error = g_current_loop.iq_ref - pc.q;

    float vd = pi_update(g_current_loop.id_pi, d_error, dt);
    float vq = pi_update(g_current_loop.iq_pi, q_error, dt);

    // Limit the requested voltage vector to the linear SVPWM boundary.
    const float v_limit = SVPWM_MAX_MODULATION * g_current_loop.vbus;
    const float v_mag = sqrtf(vd * vd + vq * vq);
    if (v_mag > v_limit) {
        const float scale = v_limit / v_mag;
        vd *= scale; vq *= scale;
    }

    g_vd = vd;
    g_vq = vq;
    g_d_err = d_error;
    g_q_err = q_error;

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
    // static uint32_t isr_div = 0;
    // if (++isr_div < 20) return;   // 20kHz / 20 = 1kHz
    // isr_div = 0;
    // closed-loop FOC runs here.
    constexpr float CLOSED_LOOP_DT = 1.0f / 20'000.0f;
    closed_loop_update(phase_a, phase_b, g_theta, CLOSED_LOOP_DT);
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
    // Start TIM1 once. This also starts
    // the synchronized ADC injected sampling.
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

    // With vbus = 9.49 V:
    // v_limit = 9.49 / sqrt(3) ≈ 5.48 V
    // so each PI can request up to ±5.48 V.
    const float v_limit = SVPWM_MAX_MODULATION * g_current_loop.vbus;
    g_current_loop.id_pi = {
        .kp = 1.0f, .ki = 1.0f, .integrator = 0.0f, .out_min = -v_limit, .out_max = v_limit };
    g_current_loop.iq_pi = {
        .kp = 1.0f, .ki = 1.0f, .integrator = 0.0f, .out_min = -v_limit, .out_max = v_limit };

    current_loop_reset(g_current_loop);
    current_loop_enabled = true;

    // soft-start the torque command
    g_ramp.iq_start = 0.0f;
    g_ramp.iq_end = 0.10f;
    g_ramp.iq_ramp_duration_s = 8.0f;

    uint32_t last_log_us = mcl::time_us();
    const uint32_t iq_ramp_start_us = mcl::time_us();
    constexpr uint32_t log_period_us = 8'000'000U;  // 8 seconds
    while (true) {
        uint32_t now_us = mcl::time_us();
        float elapsed_s = (float)(now_us - iq_ramp_start_us) * 1e-6f;
        g_current_loop.iq_ref =
            ramp_value(
                g_ramp.iq_start,
                g_ramp.iq_end,
                g_ramp.iq_ramp_duration_s,
                elapsed_s);
            uint16_t raw = as5600_read_raw_angle(bus);
            g_theta = encoder_to_electrical_angle(raw, encoder.zero_raw, encoder.sign);
            // log only once every 8 seconds
            if (0) {//(uint32_t)(now_us - last_log_us) >= log_period_us) {
                last_log_us = now_us;
                LOG << "T:" << g_theta << " ph_a: " << g_raw_a << " ph_b: " << g_raw_b
                        << " a: " << g_a << " b: " << g_b << " d: " << g_d << " q: " << g_q
                            << " d_err:" << g_d_err << " q_err:" << g_q_err << " vd: " << g_vd << " vq: " << g_vq
                                << " ab_mag: " << g_ab_mag << " ab_ang: " << g_ab_angle << " mod: " << g_svm.modulation
                                    << " idi: " << g_current_loop.id_pi.integrator << " iqi: " << g_current_loop.iq_pi.integrator;
            }
    }

    timer_stop(&tm);
}

#endif
#ifndef CURRENT_H
#define CURRENT_H

constexpr float ADC_VREF   = 3.3f;
constexpr float CSA_GAIN   = 50.0f;      // V/V, INA240A2
constexpr float R_SHUNT    = 0.010f;     // 10 mill ohms
constexpr float ADC_MAXCNT = 4095.0f;

constexpr float MOTOR_KV = 220.0f;               // RPM per volt, from datasheet
constexpr float MOTOR_RESISTANCE = 2.3f;
constexpr float MOTOR_INDUCTANCE_H = 0.00086f;   // L, henries, from datasheet
const float CURRENT_BANDWIDTH = 2000.0f;
const float WC = 2.0f * PI * CURRENT_BANDWIDTH;
inline float Kp = MOTOR_INDUCTANCE_H * WC;
inline float Ki = MOTOR_RESISTANCE * WC;

// Ke (V per mechanical rad/s) derived from KV, then divided by pole pairs
// to get lambda (V per ELECTRICAL rad/s) -- electrical_velocity is
// electrical rad/s, so this is the constant that pairs with it directly.
// Treat this as a starting estimate; verify empirically if possible
// (spin open-loop at known speed, measure induced phase voltage,
// lambda = V_peak / electrical_velocity).
constexpr float MOTOR_KE = 1.0f / (MOTOR_KV * (TWO_PI / 60.0f));
constexpr float MOTOR_LAMBDA = MOTOR_KE / (float)POLE_PAIRS * 0.55;

// PI controller
struct pi_controller_t {
    float kp = 0.0f;
    float ki = 0.0f;
    float integrator = 0.0f;
    float out_min = -1.0f;
    float out_max =  1.0f;
};

struct current_control_t {
    pi_controller_t d_pi;
    pi_controller_t q_pi;
    float d_ref = 0.0f;
    float q_ref = 0.0f;
    float vd = 0.0f;
    float vq = 0.0f;
    float vbus = 9.49f;
    float current_limit = 0.8f;
};

struct phase_transforms_t {
    // stationary frame
    float alpha = 0.0f;
    float beta = 0.0f;
    // rotating frame
    float d = 0.0f;
    float q = 0.0f;
    // Inverse Park outputs
    float v_alpha = 0.0f;
    float v_beta  = 0.0f;
};

struct current_sense_t {
    // Measured; in amps
    float ia = 0.0f;
    float ib = 0.0f;
    // Derived; from ia, ib
    float ic = 0.0f;
    // Raw ADC values
    uint16_t raw_a = 0;
    uint16_t raw_b = 0;
    // CSA zero current bias, clibrated
    float bias_a = 0.0f;
    float bias_b = 0.0f;
    uint64_t calibration_sum_a = 0;
    uint64_t calibration_sum_b = 0;
    uint32_t calibration_samples = 0;
    bool calibrating = false;
};

inline current_sense_t cs;
inline current_control_t cc;
inline phase_transforms_t pt;

inline void current_bias_calibration_start() {
    cs.calibration_samples = 0;
    cs.calibration_sum_a = 0;
    cs.calibration_sum_b = 0;
    cs.calibrating = true;
}

inline bool current_bias_calibration_complete(uint32_t target_samples) {
    return cs.calibration_samples >= target_samples;
}

inline void current_bias_calibration_finish() {
    const uint32_t n = cs.calibration_samples;
    if (n == 0) {
        cs.calibrating = false;
        return;
    }
    const float avg_a = (float)cs.calibration_sum_a / (float)n;
    const float avg_b = (float)cs.calibration_sum_b / (float)n;
    cs.bias_a = (avg_a / ADC_MAXCNT) * ADC_VREF;
    cs.bias_b = (avg_b / ADC_MAXCNT) * ADC_VREF;
    LOG << " current bias A: " << cs.bias_a << "v";
    LOG << " current bias B: " << cs.bias_b << "v";
    cs.calibrating = false;
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

inline float adc_to_phase_amps(uint16_t raw, float v_bias) {
    float v_adc = ((float)raw / ADC_MAXCNT) * ADC_VREF;
    return (v_adc - v_bias) / (CSA_GAIN * R_SHUNT);
}

inline void clarke_transform(float ia, float ib, float ic) {
    constexpr float ONE_THIRD = 1.0f / 3.0f;
    // amplitude-invariant Clarke transformation
    // uses all 3 measured/reconstructed phases
    pt.alpha = (2.0f * ia - ib - ic) * ONE_THIRD;
    pt.beta  = (ib - ic) * (1.0f / SQRT3);
}

inline void park_transform(float alpha, float beta, float electrical_angle) {
    float c = cosf(electrical_angle);
    float s = sinf(electrical_angle);
    pt.d =  alpha * c + beta * s;
    pt.q = -alpha * s + beta * c;
}

inline void inverse_park_transform(float vd, float vq, float electrical_angle) {
    const float c = cosf(electrical_angle);
    const float s = sinf(electrical_angle);
    pt.v_alpha = vd * c - vq * s;
    pt.v_beta  = vd * s + vq * c;
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

inline void current_loop_reset(current_control_t &cc) {
    pi_reset(cc.d_pi);
    pi_reset(cc.q_pi);
}

// current_sense_update()
//     ADC counts to amps
// current_control_update()
//     amps to dq to PI to voltage
// svpwm_apply()
//     voltage vector to PWM

inline void current_sense_update(
        current_sense_t& sense, uint16_t raw_a, uint16_t raw_b) {
    sense.raw_a = raw_a;
    sense.raw_b = raw_b;
    sense.ia = adc_to_phase_amps(raw_a, sense.bias_a);
    sense.ib = adc_to_phase_amps(raw_b, sense.bias_b);
    sense.ic = -(sense.ia + sense.ib);
}

inline void current_control_update(current_control_t& control, current_sense_t& sense,
        float electrical_angle, float electrical_velocity, float dt) {
    // phase abc to alpha/beta
    clarke_transform(sense.ia, sense.ib, sense.ic);
    // alpha/beta to d/q
    park_transform(pt.alpha, pt.beta, electrical_angle);
    // Current errors.
    const float d_error = control.d_ref - pt.d;
    const float q_error = control.q_ref - pt.q;
    // Current PI controllers.
    float vd_pi = pi_update(control.d_pi, d_error, dt);
    float vq_pi = pi_update(control.q_pi, q_error, dt);

    // Feedforward: decoupling + back-EMF compensation.
    // These are recomputed fresh each cycle -- no integrator state,
    // so anti-windup scaling below only needs to touch the PI portion.
    float vd_ff = -electrical_velocity * MOTOR_INDUCTANCE_H * pt.q;
    float vq_ff =  electrical_velocity * MOTOR_INDUCTANCE_H * pt.d
                    + electrical_velocity * MOTOR_LAMBDA;

    float vd = vd_pi + vd_ff;
    float vq = vq_pi + vq_ff;

    // Limit voltage vector to linear SVPWM range.
    const float v_limit = SVPWM_MAX_MODULATION * control.vbus;
    const float v_mag = sqrtf(vd * vd + vq * vq);
    if (v_mag > v_limit) {
        const float scale = v_limit / v_mag;
        vd *= scale;
        vq *= scale;
        // Preserve the existing anti-windup behaviour.
        control.d_pi.integrator *= scale;
        control.q_pi.integrator *= scale;
    }
    // Store the actual voltage command.
    control.vd = vd;
    control.vq = vq;
}
#endif
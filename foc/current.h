#ifndef CURRENT_H
#define CURRENT_H

#include <foc/control.h>
#include <foc/transforms.h>

constexpr float CSA_GAIN   = 50.0f; // V/V, INA240A2
constexpr float R_SHUNT    = 0.010f; // 10 mill ohms

constexpr float MOTOR_KV = 220.0f; // RPM per volt, from datasheet
constexpr float MOTOR_RESISTANCE = 2.3f;
constexpr float MOTOR_INDUCTANCE_H = 0.00086f; // L, henries, from datasheet
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

struct current_control_t {
    pi_controller_t d_pi;
    pi_controller_t q_pi;
    float d_ref = 0.0f;
    float q_ref = 0.0f;
    float id = 0.0f;
    float iq = 0.0f;
    float vd = 0.0f;
    float vq = 0.0f;
    float modulation = 0.0f;
    float current_limit = 0.8f;
    // current sense
    // Measured; in amps
    float ia = 0.0f;
    float ib = 0.0f;
    // Derived from ia, ib using
    // balanced 3-phase system
    float ic = 0.0f;
    // Raw ADC values
    uint16_t raw_a = 0;
    uint16_t raw_b = 0;

    inline float adc_to_phase_amps(uint16_t raw, float v_bias) {
        float v_adc = ((float)raw / ADC_MAXCNT) * ADC_VREF;
        return (v_adc - v_bias) / (CSA_GAIN * R_SHUNT);
    }

    inline void adc_to_phase_currents(uint16_t raw_a, uint16_t raw_b,
            float bias_a, float bias_b) {
        this->raw_a = raw_a;
        this->raw_b = raw_b;
        ia = adc_to_phase_amps(raw_a, bias_a);
        ib = adc_to_phase_amps(raw_b, bias_b);
        ic = -(ia + ib);
    }

    inline void reset() {
        d_pi.reset();
        q_pi.reset();
    }

    inline auto current_control(float electrical_angle, float electrical_velocity, float dt) {

        // phase abc to alpha/beta
        auto ab = clarke_transform(ia, ib, ic);

        // alpha/beta to d/q
        auto dq = park_transform(ab, electrical_angle);

        // These are still currents at this point
        id = dq.d;
        iq = dq.q;

        // Current errors.
        const float d_error = d_ref - dq.d;
        const float q_error = q_ref - dq.q;

        // Current PI controllers (output is volts now)
        float vd_pi = d_pi.update(d_error, dt);
        float vq_pi = q_pi.update(q_error, dt);

        // Feedforward: decoupling + back-EMF compensation.
        // These are recomputed fresh each cycle -- no integrator state,
        // so anti-windup scaling below only needs to touch the PI portion.
        float vd_ff = -electrical_velocity * MOTOR_INDUCTANCE_H * dq.q;
        float vq_ff =  electrical_velocity * MOTOR_INDUCTANCE_H * dq.d
                            + electrical_velocity * MOTOR_LAMBDA;
        vd = vd_pi + vd_ff;
        vq = vq_pi + vq_ff;

        // Limit voltage vector to linear SVPWM range.
        const float v_limit = SVPWM_MAX_MODULATION * VBUS;
        const float v_mag = sqrtf(vd * vd + vq * vq);
        if (v_mag > v_limit) {
            const float scale = v_limit / v_mag;
            vd *= scale;
            vq *= scale;
        }

        return dq_t{vd, vq};
    }

};

#endif
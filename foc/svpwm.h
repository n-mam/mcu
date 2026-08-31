#ifndef SVPWM_H
#define SVPWM_H

#include <tuple>

constexpr float PI = 3.14159265359f;
constexpr float PI_OVER_3 = PI / 3.0f;
constexpr float TWO_PI = 2.0f * PI;
constexpr float SQRT3  = 1.73205080757f;

// linear svpwm limit: modulation = |Vref| / Vbus
constexpr float SVPWM_MAX_MODULATION = 0.57735026919f; // 1/sqrt(3)

struct svpwm_t {
    float angle = 0.0f;
    float modulation = 0.0f;
    timer_config_t *timer = nullptr;
};

// Apply a stationary-frame voltage vector:
//     alpha, beta = Vref vector normalized to Vbus
// The input magnitude is:
//     modulation = sqrt(alpha^2 + beta^2)
// with the linear SVPWM limit:
//     modulation <= 1/sqrt(3)
// This function:
// - calculates sector
// - calculates T1/T2/T0
// - calculates Ta/Tb/Tc
// - returns Ta/Tb/Tc
inline auto svpwm_update(float alpha, float beta) {
    // Convert alpha/beta vector to magnitude and angle.
    float modulation = sqrtf(alpha * alpha + beta * beta);
    if (modulation > SVPWM_MAX_MODULATION) {
        modulation = SVPWM_MAX_MODULATION;
    }
    //svpwm.modulation = modulation;
    float theta = atan2f(beta, alpha);
    if (theta < 0.0f) {
        theta += TWO_PI;
    }
    //svpwm.angle = theta;
    int sector = (int)(theta / PI_OVER_3);
    if (sector >= 6) {
        sector = 5;
    }
    float alpha_sector = theta - (float)sector * PI_OVER_3;
    // Normalized dwell times.
    // T1 + T2 <= 1 in the linear modulation range.
    float T1 = SQRT3 * modulation * sinf(PI_OVER_3 - alpha_sector);
    float T2 = SQRT3 * modulation * sinf(alpha_sector);
    float T0 = 1.0f - T1 - T2;
    // Defensive clamp against floating-point roundoff.
    if (T0 < 0.0f) {
        T0 = 0.0f;
    }
    float Ta = 0.0f;
    float Tb = 0.0f;
    float Tc = 0.0f;
    const float T0_half = 0.5f * T0;
    switch (sector) {
        case 0:
            Ta = T1 + T2 + T0_half;
            Tb = T2 + T0_half;
            Tc = T0_half;
            break;
        case 1:
            Ta = T1 + T0_half;
            Tb = T1 + T2 + T0_half;
            Tc = T0_half;
            break;
        case 2:
            Ta = T0_half;
            Tb = T1 + T2 + T0_half;
            Tc = T2 + T0_half;
            break;
        case 3:
            Ta = T0_half;
            Tb = T1 + T0_half;
            Tc = T1 + T2 + T0_half;
            break;
        case 4:
            Ta = T2 + T0_half;
            Tb = T0_half;
            Tc = T1 + T2 + T0_half;
            break;
        case 5:
            Ta = T1 + T2 + T0_half;
            Tb = T0_half;
            Tc = T1 + T0_half;
            break;
    }
    // Defensive bounds.
    Ta = fminf(fmaxf(Ta, 0.0f), 1.0f);
    Tb = fminf(fmaxf(Tb, 0.0f), 1.0f);
    Tc = fminf(fmaxf(Tc, 0.0f), 1.0f);
    return std::make_tuple(Ta, Tb, Tc);
}

// voltage to pwm
void voltage_to_timer_pwm(timer_config_t *timer, float alpha, float beta) {
    auto [Ta, Tb, Tc] = svpwm_update(alpha, beta);
    uint32_t arr = timer->arr + 1U;
    timer->instance->CCR1 = (uint32_t)(Ta * (float)arr);
    timer->instance->CCR2 = (uint32_t)(Tb * (float)arr);
    timer->instance->CCR3 = (uint32_t)(Tc * (float)arr);
}

inline float ramp_linear(float start, float end,
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

inline void log_foc_state(float elapsed_s);

#endif
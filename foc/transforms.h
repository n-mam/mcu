#ifndef TRANSFORMS_H
#define TRANSFORMS_H

struct ab_t {
    float alpha = 0.0f;
    float beta = 0.0f;
};

struct dq_t {
    float d = 0.0f;
    float q = 0.0f;
};

inline auto clarke_transform(float ia, float ib, float ic) {
    constexpr float ONE_THIRD = 1.0f / 3.0f;
    // amplitude-invariant Clarke transformation
    // uses all 3 measured/reconstructed phases
    ab_t result;
    result.alpha = (2.0f * ia - ib - ic) * ONE_THIRD;
    result.beta  = (ib - ic) * (1.0f / SQRT3);
    return result;
}

inline auto park_transform(const ab_t ab, float electrical_angle) {
    float c = cosf(electrical_angle);
    float s = sinf(electrical_angle);
    dq_t result;
    result.d =  ab.alpha * c + ab.beta * s;
    result.q = -ab.alpha * s + ab.beta * c;
    return result;
}

inline auto inverse_park_transform(const dq_t dq, float electrical_angle) {
    const float c = cosf(electrical_angle);
    const float s = sinf(electrical_angle);
    ab_t result;
    result.alpha = dq.d * c - dq.q * s;
    result.beta  = dq.d * s + dq.q * c;
    return result;
}

#endif
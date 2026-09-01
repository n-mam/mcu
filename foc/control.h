#ifndef CONTROL_H
#define CONTROL_H

// PI controller
struct pi_controller_t {

    float kp = 0.0f;
    float ki = 0.0f;
    float integrator = 0.0f;
    float out_min = -1.0f;
    float out_max =  1.0f;

    inline void reset() {
        integrator = 0.0f;
    }

    inline float update(float error, float dt) {

        const float proportional = kp * error;

        const float candidate_integrator =
            integrator + ki * error * dt;

        const float candidate =
            proportional + candidate_integrator;

        const bool saturating_high =
            candidate > out_max && error > 0.0f;

        const bool saturating_low =
            candidate < out_min && error < 0.0f;

        if (!saturating_high && !saturating_low) {
            integrator = candidate_integrator;
        }

        const float output =
            proportional + integrator;

        return fminf(fmaxf(output, out_min), out_max);
    }

};

// inline float pi_update(pi_controller_t &pi, float error, float dt) {
//     pi.integrator += pi.ki * error * dt;
//     if (pi.integrator > pi.out_max) pi.integrator = pi.out_max;
//     if (pi.integrator < pi.out_min) pi.integrator = pi.out_min;
//     float out = pi.kp * error + pi.integrator;
//     if (out > pi.out_max) out = pi.out_max;
//     if (out < pi.out_min) out = pi.out_min;
//     return out;
// }

#endif
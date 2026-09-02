#ifndef SPEED_H
#define SPEED_H

#include <foc/control.h>

struct speed_control_t {

    pi_controller_t pi;
    float speed_ref = 0.0f;
    float speed_measured = 0.0f;
    float iq_ref_limit = 0.30f;

    inline void reset() {
        pi.reset();
    }

    inline float speed_control(float mechanical_velocity, float dt) {
        speed_measured = mechanical_velocity;
        const float error = speed_ref - speed_measured;

        pi.out_min = -iq_ref_limit;
        pi.out_max =  iq_ref_limit;

        float iq_ref = pi.update(error, dt);

        if (iq_ref > iq_ref_limit)  iq_ref = iq_ref_limit;
        if (iq_ref < -iq_ref_limit) iq_ref = -iq_ref_limit;

        return iq_ref;
    }
};

#endif
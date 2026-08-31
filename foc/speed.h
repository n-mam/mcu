#ifndef SPEED_H
#define SPEED_H

struct speed_control_t {
    pi_controller_t pi;
    float speed_ref = 0.0f;
    float speed_measured = 0.0f;
    float iq_ref_limit = 0.30f;
};

inline void speed_loop_reset(speed_control_t& s) {
    pi_reset(s.pi);
}

inline void speed_control_update(speed_control_t& s,
        float mechanical_velocity, float dt) {
    s.speed_measured = mechanical_velocity;
    const float error = s.speed_ref - s.speed_measured;

    s.pi.out_min = -s.iq_ref_limit;
    s.pi.out_max =  s.iq_ref_limit;

    float iq_ref = pi_update(s.pi, error, dt);

    if (iq_ref > s.iq_ref_limit)  iq_ref = s.iq_ref_limit;
    if (iq_ref < -s.iq_ref_limit) iq_ref = -s.iq_ref_limit;

    cc.q_ref = iq_ref;
}

inline speed_control_t sc;

#endif
#ifndef SPEED_H
#define SPEED_H

struct speed_control_t {
    pi_controller_t pi;
    float speed_ref = 0.0f;      // mechanical rad/s
    float speed_measured = 0.0f; // mechanical rad/s
    float iq_ref_limit = 0.15f;   // should match/derive from current_limit
};

inline speed_control_t sc;

inline void speed_loop_reset(speed_control_t& s) {
    pi_reset(s.pi);
}

inline void speed_control_update(speed_control_t& s,
        float mechanical_velocity, float dt) {
    s.speed_measured = mechanical_velocity;
    const float error = s.speed_ref - s.speed_measured;
    float iq_ref = pi_update(s.pi, error, dt);
    // pi_update already clamps to out_min/out_max, which we set
    // to +/- iq_ref_limit below -- this line is just documentation
    // of intent, not an extra clamp.
    cc.q_ref = iq_ref;
}

#endif
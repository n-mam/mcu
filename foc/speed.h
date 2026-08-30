#ifndef SPEED_H
#define SPEED_H

struct speed_control_t {
    pi_controller_t pi;
    float speed_ref = 0.0f;      // mechanical rad/s
    float speed_measured = 0.0f; // mechanical rad/s
    float iq_ref_limit = 0.20f;   // should match/derive from current_limit
    float friction_ff = 0.05f;    // amps -- minimum q current to overcome
                                   // static friction/cogging breakaway,
                                   // tune empirically (see below)
};

inline float sign_f(float x) {
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

inline void speed_loop_reset(speed_control_t& s) {
    pi_reset(s.pi);
}

inline void speed_control_update(speed_control_t& s,
        float mechanical_velocity, float dt) {
    s.speed_measured = mechanical_velocity;
    const float error = s.speed_ref - s.speed_measured;
    float iq_pi = pi_update(s.pi, error, dt);
    // Coulomb friction / cogging breakaway feedforward. This opposes
    // motion with roughly constant magnitude regardless of speed, so
    // give it in the direction the rotor is *commanded* to move
    // (not the direction of the error) so the PI no longer has to
    // discover the breakaway torque on its own via the integrator.
    float iq_ff = s.friction_ff * sign_f(s.speed_ref);
    float iq_ref = iq_pi + iq_ff;

    if (iq_ref > s.iq_ref_limit)  iq_ref = s.iq_ref_limit;
    if (iq_ref < -s.iq_ref_limit) iq_ref = -s.iq_ref_limit;

    cc.q_ref = iq_ref;
}

inline speed_control_t sc;

#endif
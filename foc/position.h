#ifndef POSITION_H
#define POSITION_H

#include <cmath> 
#include <foc/control.h>

struct position_control_t {
    pi_controller_t pi{};
    // Target, wrapped mechanical radians, range [-PI, PI].
    float position_ref = 0.0f;
    float position_measured = 0.0f;
    float position_error = 0.0f;

    inline void reset() {
        pi.reset();
        position_measured = 0.0f;
        position_error = 0.0f;
    }

    static inline float wrap_angle(float angle) {
        angle = fmodf(angle + PI, TWO_PI);
        if (angle < 0.0f) {
            angle += TWO_PI;
        }
        return angle - PI;
    }

    static inline float shortest_angle_error(float ref, float measured) {
        return wrap_angle(ref - measured);
    }

    // Accepts degrees, converts and wraps internally — this is the only
    // place position_ref should be assigned from outside the struct.
    inline void set_position_ref_deg(float degrees) {
        constexpr float DEG_TO_RAD = PI / 180.0f;
        position_ref = wrap_angle(degrees * DEG_TO_RAD);
    }

    // measured_position: encoder.read().mechanical_angle (wrapped, [-PI, PI]).
    // dt: position loop period, s.
    // Returns speed_ref [rad/s] to feed into speed_control_t.
    inline float position_control(float measured_position, float dt) {
        position_measured = measured_position;
        position_error = shortest_angle_error(position_ref, measured_position);
        return pi.update(position_error, dt);
    }
};

#endif
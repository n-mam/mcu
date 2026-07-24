#ifndef MOTOR_H
#define MOTOR_H

#include <memory>

#include <mcl/pwm.h>
#include <npb/nm.h>

namespace mcl {

struct drv8833 {

    int _duty = 10;
    mcl::pwm _pwm_a;
    mcl::pwm _pwm_b;
    bool _forward = true;

    drv8833(int pin1, int pin2, int frequency = 20000) :
        _pwm_a(pin1, frequency),
        _pwm_b(pin2, frequency) {

    }
    ~drv8833() {
        stop();
    }
    void stop() {
        _pwm_a.stop();
        _pwm_b.stop();
    }
    void start() {
        _pwm_a.start(_forward ? _duty : 0);
        _pwm_b.start(_forward ? 0 : _duty);
    }
    void set_direction(bool forward) {
        if (_forward == forward) return;
        _forward = forward;
        stop();
        start();
    }
    void set_speed(int duty) {
        _duty = duty;
        _pwm_a.set_duty_cycle(_forward ? _duty : 0);
        _pwm_b.set_duty_cycle(_forward ? 0 : _duty);
    }
};

inline auto initialize_m200(int pin, int freq) {
    // Send a stopped signal (1500 microseconds) for a few seconds to initialize
    // the ESC. You will hear two tones indicating initialization, and then you
    // can send a signal from 1100-1900 µs to operate the thruster. 1500–1900 µs
    // for forward thrust, 1100–1500 µs for reverse.
    auto m200 = std::make_unique<mcl::pwm>(pin, freq);
    m200->start();
    m200->set_duty_cycle(1.5 / 20.0);
    mcl::sleep_ms(10 * 1000);
    return m200;
}

inline auto initialize_blheliS(int pin, int freq, bool calibrate) {
    mcl::sleep_ms(3 * 1000);
    auto blhS = std::make_unique<mcl::pwm>(pin, freq);
    blhS->start();
    if (calibrate) {
        LOG << "blheliS setting max throttle";
        blhS->set_duty_cycle(2.0 / 20.0);
        mcl::sleep_ms(6 * 1000);
        LOG << "blheliS set max throttle done";
        LOG << "blheliS setting min throttle";
        blhS->set_duty_cycle(1.0 / 20.0);
        mcl::sleep_ms(8* 1000);
        LOG << "blheliS set min throttle done";
    } else {
        blhS->set_duty_cycle(2.0 / 20.0);
        blhS->set_duty_cycle(1.0 / 20.0);
        mcl::sleep_ms(5 * 1000);
    }
    return blhS;
}

inline auto initialize_rudder(int pin, int freq) {
    auto rudder = std::make_unique<mcl::pwm>(pin, freq);
    rudder->start();
    rudder->set_duty_cycle(1.5 / 20.0); // centre
    return rudder;
}

} //namespace mcl

#include <motor/test.h>

#endif
#ifndef PWM_H
#define PWM_H

#include <mcl/mcl.h>

#if defined (STM32)
#include <fx/timer.h>
#elif defined (PICO)
#include <hardware/pwm.h>
#endif

#include <cmath>
#include <cstdint>
#include <stdio.h>

namespace mcl {

struct pwm {

    #if defined (PICO)
    int _wrap;
    int _slice;
    int _channel;
    int _frequency;
    uint32_t _divider;
    uint64_t f_clock = 125000000;
    double f_minimum = 1907.34863281;
    #elif defined (STM32)
    Timer _timer;
    #endif

    pwm(int pin, int frequency)
        #if defined (STM32)
        : _timer(TIM2)
        #endif
    {
        #if defined (PICO)
        gpio_set_function(pin, GPIO_FUNC_PWM);
        _slice = pwm_gpio_to_slice_num(pin);
        _channel = pwm_gpio_to_channel(pin);
        if (frequency > f_minimum) {
            _wrap = (f_clock / frequency) - 1;
        } else {
            _wrap = get_wrap_using_divider_ex(frequency);
        }
        _frequency = frequency;
        pwm_set_wrap(_slice, _wrap);
        //printf("pwm::pwm -> frequency %d, wrap %d\n", _frequency, _wrap);
        #else
        _timer.init_channel(1, GPIOA, pin);
        _timer.set_frequency(frequency);
        _timer.set_duty_cycle(1, 0.0f);
        #endif
    }

    ~pwm() {
        stop();
    }

    void start(double duty = 0) {
        #if defined (PICO)
        //printf("pwm::start duty %f\n", duty);
        set_duty_cycle(duty);
        pwm_set_enabled(_slice, true);
        #else
        _timer.start_channel(1);
        _timer.enable();
        #endif
    }

    void stop() {
        printf("pwm::stop...\n");
        set_duty_cycle(0);
        mcl::sleep_ms(250);
        #if defined (PICO)
        pwm_set_enabled(_slice, false);
        #else
        _timer.stop_channel(1);
        _timer.disable();
        #endif
    }

    void set_duty_cycle(double duty) {
        #if defined (PICO)
        auto level = _wrap * duty;
        //printf("set_duty_cycle: duty %f, level %f\n", duty, level);
        pwm_set_chan_level(_slice, _channel, level);
        #elif defined (STM32)
        _timer.set_duty_cycle(1, duty);
        #endif
    }

    #if defined (PICO)
    uint32_t get_wrap_using_divider(uint32_t frequency) {
        _divider = f_clock / frequency / 4096 +
                (f_clock % (frequency * 4096) != 0);
        if (_divider / 16 == 0)
            _divider = 16;
        pwm_set_clkdiv_int_frac(_slice, _divider / 16, _divider & 0xF);
        uint32_t wrap = f_clock * 16 / _divider / frequency - 1;
        //printf("get_wrap_using_divider _divider %d\n", _divider);
        return wrap;
    }

    uint32_t get_wrap_using_divider_ex(uint32_t frequency) {
        auto d = f_clock / frequency / 65535.0;
        _divider = std::ceil(d);
        auto w = (f_clock / _divider / frequency) - 1;
        pwm_set_clkdiv_int_frac(_slice, _divider, 0);
        //printf("get_wrap_using_divider_ex _divider %d\n", _divider);
        return w;
    }
    #endif
};

}

#endif
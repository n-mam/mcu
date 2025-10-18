#ifndef TIMER_H
#define TIMER_H

#include <math.h>
#include <stdint.h>

#include <mcl/mcl.h>

// timer api should be such that each type of timer (advanced, GP and basic) should
// be singletons i.e. multiple calls to create TIM1 should return the same Timer object
// multiple calls to create the same timer should differ in individual channels.. API
// should aloow to set the duty for each channel

struct Channel {
    int _pin;
    int _step;
    float _dutyCycle;
    GPIO_TypeDef *_port;
};

struct Timer *t2 = nullptr;

struct Timer {

    Timer(TIM_TypeDef *timer) : _timer(timer) {
        //t2 = this;
        // Enable clock for the timer
        mcl::enableClockForTimer(_timer);
        // Derive a 1MHz timer input clock from the APB1 timer clock
        _timer->PSC = (mcl::apb1TimerClock() / 1000000) - 1;
        // Reset the timer
        _timer->CR1 = 0;
    }

    bool isAdvancedTimer() {
        return _timer == TIM1;
    }

    void enable() {
        // Enable MOE for advanced timers
        if (isAdvancedTimer()) {
            _timer->BDTR |= TIM_BDTR_MOE;
        }
        // Enable the timer
        _timer->CR1 |= TIM_CR1_CEN;
    }

    void disable() {
        // disable MOE for advanced timers
        if (isAdvancedTimer()) {
            _timer->BDTR &= ~TIM_BDTR_MOE;
        }
        // Disable the timer
        _timer->CR1 &= ~TIM_CR1_CEN;
        // Clear interrupt flags to
        // prevent pending interrupts
        _timer->SR = 0;
    }

    void init_channel(int channel, GPIO_TypeDef *port, int pin) {
        _channels[channel]._pin = pin;
        _channels[channel]._port = port;
        init_gpio(port, pin);
        // channel settings
        if (channel == 1) {
            // CC1 in output mode
            _timer->CCMR1 &= ~(TIM_CCMR1_CC1S);
            // preload enable
            _timer->CCMR1 |= TIM_CCMR1_OC1PE;
            // pwm mode 1
            _timer->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
        } else if (channel == 2) {
            // CC2 in output mode
            _timer->CCMR1 &= ~(TIM_CCMR1_CC2S);
            // preload enable
            _timer->CCMR1 |= TIM_CCMR1_OC2PE;
            // pwm mode 1
            _timer->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
        } else if (channel == 3) {
            // CC3 in output mode
            _timer->CCMR2 &= ~(TIM_CCMR2_CC3S);
            // preload enable
            _timer->CCMR2 |= TIM_CCMR2_OC3PE;
            // pwm mode 1
            _timer->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
        } else if (channel == 4) {
            // CC4 in output mode
            _timer->CCMR2 &= ~(TIM_CCMR2_CC4S);
            // Output compare preload enable
            _timer->CCMR2 |= TIM_CCMR2_OC4PE;
            // pwm mode 1
            _timer->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
        }
    }

    void enable_interrupt() {
        // Enable update event interrupt
        _timer->DIER |= TIM_DIER_UIE;
        // Enable TIM IRQ in NVIC
        if (_timer == TIM2) {
            NVIC_EnableIRQ(TIM2_IRQn);
        } else if (_timer == TIM3) {
            NVIC_EnableIRQ(TIM3_IRQn);
        } else if (_timer == TIM4) {
            NVIC_EnableIRQ(TIM4_IRQn);
        } else if (_timer == TIM5) {
            NVIC_EnableIRQ(TIM5_IRQn);
        }
    }

    void start_channel(int channel) {
        if (channel == 1) {
            // CC1 output enable
            _timer->CCER |= TIM_CCER_CC1E;
        } else if (channel == 2) {
            // CC2 output enable
            _timer->CCER |= TIM_CCER_CC2E;
        } else if (channel == 3) {
            // CC3 output enable
            _timer->CCER |= TIM_CCER_CC3E;
        } else if (channel == 4) {
            // CC4 output enable
            _timer->CCER |= TIM_CCER_CC4E;
        }
    }

    void stop_channel(int channel) {
        if (channel == 1) {
            _timer->CCER &= ~TIM_CCER_CC1E;
        } else if (channel == 2) {
            _timer->CCER &= ~TIM_CCER_CC2E;
        } else if (channel == 3) {
            _timer->CCER &= ~TIM_CCER_CC3E;
        } else if (channel == 4) {
            _timer->CCER &= ~TIM_CCER_CC4E;
        }
    }

    void set_frequency(uint32_t frequency) {
        _frequency = frequency;
        // Timer clock frequency => 1MHz
        uint32_t timerClockFreq = mcl::apb1TimerClock() / (_timer->PSC + 1);
        // Calculate ARR count value to have the desired output PWM frequency
        _arr = timerClockFreq / _frequency - 1;
         // Set auto reload register for the desired output signal frequency
         _timer->ARR = _arr;
        //init_sine_table();
    }

    void set_duty_cycle(int channel, double dutyCycle) {
        _channels[channel]._dutyCycle = dutyCycle;
        // CCR value configures the PWM duty cycle
        uint32_t ccr = static_cast<uint32_t>(dutyCycle * (_arr + 1));
        // CC value for desired PWM duty cycle
        if (channel == 1) {
            _timer->CCR1 = ccr;
        } else if (channel == 2) {
            _timer->CCR2 = ccr;
        } else if (channel == 3) {
            _timer->CCR3 = ccr;
        } else if (channel == 4) {
            _timer->CCR4 = ccr;
        }
    }

    // First we scale down APB1 timer to derive a 1 MHz timer input clock
    // We then set ARR to configure the actual PWM frequency to say 50 Hz
    // This implies that we have 50 pwm pulses in 1 sec, each of which is
    // available for a single sinusoidal duty cycle increment step.
    void init_sine_table() {
        auto f = _frequency;
        double step = M_PI / f;
        for (auto i = 0; i <= f; i++) {
            _lut.push_back(sin(i * step));
            std::cout << _lut[i] << std::endl;
        }
    }

    void sinusoidal_next_step_for_channel(int channel) {
        auto& step = _channels[channel]._step;
        set_duty_cycle(channel, _lut[step++]);
        step %= _lut.size();
    }

    private:

    uint32_t _arr;
    uint32_t _frequency;
    TIM_TypeDef *_timer;
    std::vector<double> _lut;
    Channel _channels[4] = { 0 };

    void init_gpio(GPIO_TypeDef *port, int pin) {
        // Enable clock for the GPIO port
        mcl::enableClockForGpio(port);
        // Clear mode bits for the pin (00 reset state)
        port->MODER &= ~(0b11 << (2 * pin));
        // Set mode to alternate function (10 AF mode)
        port->MODER |= (0b10 << (2 * pin));
        // Set alternate function to timer (AF01)
        if (pin < 8) {
            port->AFR[0] &= ~(0xF << (4 * pin)); // Clear AFRL bits
            port->AFR[0] |= (2 << (4 * pin));    // AF1 for timer
        } else {
            port->AFR[1] &= ~(0xF << (4 * (pin - 8))); // Clear AFRH bits
            port->AFR[1] |= (2 << (4 * (pin - 8)));    // AF1 for timer
        }
    }
};

extern "C" {
    void TIM2_IRQHandler(void) {
        if (TIM2->SR & TIM_SR_UIF) {
            // Clear interrupt flag
            TIM2->SR &= ~TIM_SR_UIF;
            if (t2) {
                t2->sinusoidal_next_step_for_channel(1);
                t2->sinusoidal_next_step_for_channel(2);
                t2->sinusoidal_next_step_for_channel(3);
            }
        }
    }
}

#endif
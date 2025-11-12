#ifndef TIMER_H
#define TIMER_H

#include <math.h>
#include <stdint.h>

#include <mcl/mcl.h>

// timer api should be such that each type of timer (advanced, GP and basic) should
// be singletons i.e. multiple calls to create TIM1 should return the same Timer object
// multiple calls to create the same timer should differ in individual channels.. API
// should allow to set the duty cycle individually for each channel

struct channel {
    int _pin;
    int _pin_n;
    int _step;
    float _dutyCycle;
    GPIO_TypeDef *_port;
    GPIO_TypeDef *_port_n;
};

struct Timer *t2 = nullptr;

struct Timer {

    Timer(TIM_TypeDef *timer) : _timer(timer) {
        //t2 = this;
        // Enable clock for timer
        mcl::enableClockForTimer(_timer);
        // Derive a 1MHz timer input clock from APB1/2 timer clock
        _timer->PSC = (timerClock() / 1000000) - 1;
        // Reset the timer
        _timer->CR1 = 0;
        // Enable auto-reload preload
        _timer->CR1 |= TIM_CR1_ARPE;
    }

    uint32_t timerClock() {
        if (isAdvance()) {
            return mcl::apb2TimerClock();
        } else {
            return mcl::apb1TimerClock();
        }
    }

    void init_gpio(GPIO_TypeDef *port, const std::vector<int>& pins, uint32_t af) {
        // Enable clock for GPIO
        mcl::enableClockForGpio(port);
        for (const auto& pin : pins) {
            // Clear mode bits for the pin (00 reset state)
            port->MODER &= ~(0b11 << (2 * pin));
            // Set mode to alternate function (10 AF mode)
            port->MODER |= (0b10 << (2 * pin));
            // Set alternate function to timer (AF01)
            if (pin < 8) {
                port->AFR[0] &= ~(0xF << (4 * pin)); // Clear AFRL bits
                port->AFR[0] |= (af << (4 * pin)); // AF for timer
            } else {
                port->AFR[1] &= ~(0xF << (4 * (pin - 8))); // Clear AFRH bits
                port->AFR[1] |= (af << (4 * (pin - 8))); // AF for timer
            }
        }
    }

    void init_channel(int ch, GPIO_TypeDef *port, int pin, GPIO_TypeDef *port_n, int pin_n) {
        _channels[ch]._pin = pin;
        _channels[ch]._port = port;
        _channels[ch]._pin_n = pin_n;
        _channels[ch]._port_n = port_n;
        // channel settings
        if (ch == 1) {
            // CC1 in output mode
            _timer->CCMR1 &= ~(TIM_CCMR1_CC1S);
            // CCR preload enable
            _timer->CCMR1 |= TIM_CCMR1_OC1PE;
            // pwm mode 1
            _timer->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
        } else if (ch == 2) {
            // CC2 in output mode
            _timer->CCMR1 &= ~(TIM_CCMR1_CC2S);
            // CCR preload enable
            _timer->CCMR1 |= TIM_CCMR1_OC2PE;
            // pwm mode 1
            _timer->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
        } else if (ch == 3) {
            // CC3 in output mode
            _timer->CCMR2 &= ~(TIM_CCMR2_CC3S);
            // CCR preload enable
            _timer->CCMR2 |= TIM_CCMR2_OC3PE;
            // pwm mode 1
            _timer->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
        } else if (ch == 4) {
            // CC4 in output mode
            _timer->CCMR2 &= ~(TIM_CCMR2_CC4S);
            // CCR preload enable
            _timer->CCMR2 |= TIM_CCMR2_OC4PE;
            // pwm mode 1
            _timer->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
        }
    }

    void set_dead_time(uint32_t dt_ns) {
        // dead time is clocked directly using using timer input clock i.e.
        // APB2, which can be either scaled or as-is. This is same as CLK_PSC
        // BEFORE it gets divided by the timer's PSC factor post which it becomes
        // CLK_CNT (input to the counter)
        // Time period of DTS in nanoseconds
        // ~ 10.41ns - with a 96MHz scaled APB1 timer input clock
        if (!isAdvance()) return;
        double T_dts_ns = (1.0 / timerClock()) * 1e9;
        uint32_t dt_ticks = dt_ns / T_dts_ns;
        _timer->BDTR = (_timer->BDTR & ~TIM_BDTR_DTG) | (dt_ticks & 0x7F);
        // this is around 24 which is less than 63. since it falls in the
        // first (linear) dead-time range, where DTG = DT_ticks TODO
    }

    void start_channel(int ch, bool eco) {
        if (ch == 1) {
            // CC1 output enable and enable complementary output
            _timer->CCER |= TIM_CCER_CC1E | ((eco && isAdvance()) ? TIM_CCER_CC1NE : 0);
        } else if (ch == 2) {
            // CC2 output enable and enable complementary output
            _timer->CCER |= TIM_CCER_CC2E | ((eco && isAdvance()) ? TIM_CCER_CC2NE : 0);
        } else if (ch == 3) {
            // CC3 output enable and enable complementary output
            _timer->CCER |= TIM_CCER_CC3E | ((eco && isAdvance()) ? TIM_CCER_CC3NE : 0);
        } else if (ch == 4) {
            // CC4 output enable no and complementary output on channel 4
            _timer->CCER |= TIM_CCER_CC4E;
        }
    }

    void stop_channel(int ch) {
        if (ch == 1) {
            _timer->CCER &= ~TIM_CCER_CC1E;
        } else if (ch == 2) {
            _timer->CCER &= ~TIM_CCER_CC2E;
        } else if (ch == 3) {
            _timer->CCER &= ~TIM_CCER_CC3E;
        } else if (ch == 4) {
            _timer->CCER &= ~TIM_CCER_CC4E;
        }
    }

    void set_frequency(uint32_t frequency) {
        _frequency = frequency;
        // Timer clock frequency => 1MHz
        uint32_t timerClockFreq = timerClock() / (_timer->PSC + 1);
        // Calculate ARR count value to have the desired output PWM frequency
        _arr = timerClockFreq / _frequency - 1;
         // Set auto reload register for the desired output signal frequency
         _timer->ARR = _arr;
        //init_sine_table();
    }

    void set_duty_cycle(int ch, double dutyCycle) {
        _channels[ch]._dutyCycle = dutyCycle;
        // CCR value configures the PWM duty cycle
        uint32_t ccr = static_cast<uint32_t>(dutyCycle * (_arr + 1));
        // CC value for desired PWM duty cycle
        if (ch == 1) {
            _timer->CCR1 = ccr;
        } else if (ch == 2) {
            _timer->CCR2 = ccr;
        } else if (ch == 3) {
            _timer->CCR3 = ccr;
        } else if (ch == 4) {
            _timer->CCR4 = ccr;
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

    bool isAdvance() {
        return _timer == TIM1;
    }

    void enable() {
        // Force an update event
        _timer->EGR |= TIM_EGR_UG;
        // Reset counter
        _timer->CNT = 0;
        if (isAdvance()) {
            // Disable break input
            _timer->BDTR &= ~TIM_BDTR_BKE;
            // Main output enable
            _timer->BDTR |= TIM_BDTR_MOE;
        }
        // Enable the timer
        _timer->CR1 |= TIM_CR1_CEN;
    }

    void disable() {
        // disable MOE for advanced timers
        if (isAdvance()) {
            _timer->BDTR &= ~TIM_BDTR_MOE;
        }
        // Disable the timer
        _timer->CR1 &= ~TIM_CR1_CEN;
        // Clear interrupt flags to
        // prevent pending interrupts
        _timer->SR = 0;
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
    channel _channels[4+1] = { 0 };
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
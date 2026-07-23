#ifndef PWM_H
#define PWM_H

#include <mcl/mcl.h>

#if defined (STM32)
#include <timer.h>
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
        _timer.init_channel(1, GPIOA, pin, nullptr, -1);
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
        _timer.start_channel(1, false);
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

inline void test_led() {
    mcl::pwm _pwm(5, PWM_FREQ_LED);
    _pwm.start(100);
    while (true) {
        for (uint16_t i = 0; i < 100; i++) {
            _pwm.set_duty_cycle(0.01 * i);
            mcl::sleep_ms(50);
        }
        for (uint16_t i = 100; i > 0; i--) {
            _pwm.set_duty_cycle(0.01 * i);
            mcl::sleep_ms(50);
        }
    }
}

inline void toggle_default_led() {
    #if defined (STM32F411xE)
    // STM32F411CEU6 black pill
    // Enable AHB1 clock to GPIO C
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    // Set GPIOC's pin 13 mode to GP output
    GPIOC->MODER |= GPIO_MODER_MODER13_0;
    // Set GPIOC's pin 13 to "very high" speed output
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED13;
    while (true) {
        std::cout <<"toggle PC13" << std::endl;
        GPIOC->ODR ^= (1 << 13);
        // systick 1ms timer test
        mcl::sleep_ms(500);
    }
    #elif defined (STM32F446xx)
    // Enable AHB1 clock to GPIO A
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Set GPIOA's pin 5 mode to GP output
    GPIOA->MODER |= GPIO_MODER_MODER5_0;
    // Set GPIOA's pin 5 to "very high" speed output
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED5;
    while (true) {
        std::cout <<"toggle PA5" << std::endl;
        GPIOA->ODR ^= (1 << 5);
        // systick 1ms timer test
        mcl::sleep_ms(500);
    }
    #elif defined (STM32F7)
    // NUCLEO-F767ZI STM32F767ZI
    // User LD2: a blue user LED is connected to PB7
    // Enable AHB1 clock to GPIO C
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    // Set GPIOB's pin 7 mode to GP output
    GPIOB->MODER |= GPIO_MODER_MODER7_0;
    // Set GPIOB's pin 7 to "very high" speed output
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR7;
    while (true) {
        std::cout <<"toggle PB7" << std::endl;
        GPIOB->ODR ^= (1 << 7);
        // systick 1ms timer test
        mcl::sleep_ms(500);
    }
    #elif defined (PICO)
        #if defined(PICO_DEFAULT_LED_PIN)
            // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
            // so we can use normal GPIO functionality to turn the led on and off
            gpio_init(PICO_DEFAULT_LED_PIN);
            gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
        #elif defined(CYW43_WL_GPIO_LED_PIN)
            // Unlike Pico, the on-board LED on Pico W is not connected to a
            // pin on RP2040, but instead to a GPIO pin on the wireless chip
        #endif
        while (true) {
            #if defined(PICO_DEFAULT_LED_PIN)
                // Just set the GPIO on or off
                gpio_put(PICO_DEFAULT_LED_PIN, 1);
            #elif defined(CYW43_WL_GPIO_LED_PIN)
                // Ask the wifi "driver" to set the GPIO on or off
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            #endif
            mcl::sleep_ms(500);
            #if defined(PICO_DEFAULT_LED_PIN)
                // Just set the GPIO on or off
                gpio_put(PICO_DEFAULT_LED_PIN, 0);
            #elif defined(CYW43_WL_GPIO_LED_PIN)
                // Ask the wifi "driver" to set the GPIO on or off
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            #endif
            mcl::sleep_ms(500);
        }
    #endif
}

}

#endif
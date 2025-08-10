#ifndef MCL_H
#define MCL_H

#include <string>
#include <chrono>
#include <thread>
#include <cstdint>
#include <iomanip>
#include <iostream>

#if defined (PICO)
#include "pico/time.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#if defined(PICO_CYW43_SUPPORTED)
#include <net/server.h>
#endif
#endif

#if defined (STM32F4)
#include <stm32f4xx.h>
#elif defined (STM32F7)
#include <stm32f7xx.h>
#elif defined (STM32H7)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#include <vcp.h>
#include <h7/h7.h>
extern "C" void OpenAMP_init(void);
extern "C" void OPENAMP_check_for_message();
extern "C" int openamp_send_message(const char *data, int len);
#endif

#if defined (STM32F4)
extern "C" void initialise_monitor_handles(void);
#if defined (STM32F411xE)
extern "C" void cdc_main_irq(void (*)(char));
extern "C" void cdc_write_data(const char *, size_t);
#endif
#endif

#include <npb/nm.h>
#include <mcl/log.h>
#include <mcl/fx/clock.h>

// pins
constexpr uint8_t IR_PIN = 2;
constexpr uint8_t SDA_PIN_PRS = 0;
constexpr uint8_t SCL_PIN_PRS = 0;
constexpr uint8_t SDA_PIN_IMU = 16;
constexpr uint8_t SCL_PIN_IMU = 17;
constexpr uint8_t PWM_PIN_MOTOR = 15;
constexpr uint8_t PWM_PIN_RUD_SERVO = 0;
constexpr uint8_t PWM_PIN_ELE_SERVO = 0;

// frequencies
constexpr uint8_t PWM_FREQ_MOTOR = 50;
constexpr uint8_t PWM_FREQ_SERVO = 50;
constexpr uint16_t PWM_FREQ_LED = 5000;
constexpr uint16_t PWM_FREQ_MOTOR_DRV8833 = 20000;

using TExitCallback = std::function<bool (void)>;

namespace mcl {
#if defined (STM32) && defined (STM32H7)
void ipc_send_message(const std::string& message) {
    int status = openamp_send_message(message.c_str(), message.size());
    stlink_uart("openamp_send_message rc %d\n", status);
}
#endif

#if defined (PICO_CYW43_SUPPORTED)
inline void serverCallback(uint8_t* buf, int len) {
    //mcl::dumpBuffer(reinterpret_cast<uint8_t*>(buf), len);
    nanomsg::decode(buf, len);
    //std::cout << "--------------" << std::endl;
}
#endif

std::vector<std::string> uart_messages;
inline void uart_char_recv(void *param) {
    static std::string s_message;
    #if defined (PICO)
    s_message += stdio_getchar_timeout_us(0);
    #else
    s_message += *((char *)param);
    #endif
    if (s_message.size() > 3 &&
        s_message[s_message.size() - 1] == 'x' &&
        s_message[s_message.size() - 2] == 'x' &&
        s_message[s_message.size() - 3] == 'x') {
            uart_messages.push_back(std::string
                (s_message.begin(), s_message.end() - 3));
            s_message.clear();
    }
}
void check_for_message() {
    if (uart_messages.size()) {
        auto msg = uart_messages.back();
        std::cout << mcl::dumpBuffer((const uint8_t *)msg.c_str(), msg.size());
        nanomsg::decode((uint8_t *)msg.c_str(), msg.size());
        uart_messages.pop_back();
    }
}

#if defined (PICO)
struct repeating_timer timer;
bool uart_message_timer(__unused struct repeating_timer *t) {
    check_for_message();
    return true;
}
#endif

inline void initialize_logging(mcl::log::level l) {
    #if defined (PICO_CYW43_SUPPORTED)
    auto server = getInstance<tcp::server>();
    server->setRecvCallback(serverCallback);
    server->start("AirFiber-AERx73", "paeKahng7ao9to9x");
    #endif
    log::setLogLevel(l);
    log::setLogSink<std::string>(
        [](int level, int sink, auto log) {
            if (sink == mcl::log::sink::net) {
                #if defined (PICO_CYW43_SUPPORTED)
                getInstance<tcp::server>()->
                    send_data(log.c_str(), log.size());
                cyw43_poll();
                #endif
            } else if (sink == mcl::log::sink::uart) {
                #if defined(STM32H7)
                stlink_uart(log.c_str());
                #endif
            } else if (sink == mcl::log::sink::con) {
                #if defined (PICO_CYW43_SUPPORTED)
                nanomsg::write_nano_msg_log(log.c_str(), mcl::log::sink::net);
                #elif defined (STM32F411xE)
                cdc_write_data(log.c_str(), log.size());
                #else
                std::cout << mcl::time_ms() << ": " << log << std::endl;
                #endif
            } else if (sink == mcl::log::sink::c2c) {
                #if defined (STM32) && defined (STM32H7)
                ipc_send_message(log);
                #endif
            } else if (sink == mcl::log::sink::cdc) {
                #if defined (STM32F411xE)
                cdc_write_data(log.c_str(), log.size());
                #endif
            } else if (sink == mcl::log::sink::spi) {
                // todo
            } else {
                std::cout << "invalid sink" << std::endl;
            }
        });
}

inline void initialize() {
    #if defined (PICO)
    stdio_init_all();
    add_repeating_timer_ms(350, uart_message_timer, NULL, &timer);
    stdio_set_chars_available_callback(uart_char_recv, NULL);
    #elif defined (STM32)
    // Initialize clock
    #if defined (STM32F411xE)
    clock_init({25, 192, 2, 4});
    #elif defined (STM32F446xx)
    clock_init({8, 360, 2, 2, 2});
    #elif defined (STM32F767xx)
    clock_init({4, 216, 2, 9});
    #elif defined (STM32H7)
    initialize_h7();
    OpenAMP_init();
    #endif
    #if defined (STM32F411xE)
    cdc_main_irq([](char ch){
        uart_char_recv(&ch);
        check_for_message();
    });
    #endif
    #if defined (STM32F4)
    // Initialize semihosting
    initialise_monitor_handles();
    #endif
    #ifndef STM32H7
    // Enable 1ms systick interrupts
    SysTick_Config(SystemCoreClock / 1000U);
    // Set SysTick interrupt priority
    NVIC_SetPriority(SysTick_IRQn, 0);
    #endif
    #endif
    mcl::sleep_ms(2500);
    mcl::initialize_logging(mcl::log::info);
    #if defined (STM32)
    std::cout << "SystemCoreClock: " << SystemCoreClock << std::endl;
    #endif
}

void send_discovery() {
    #if defined (PICO_CYW43_SUPPORTED)
    nanomsg::write_nano_msg_discovery(
        getInstance<tcp::server>()->self_ip, mcl::log::sink::net);
    #endif
    LOG << "test log message !";
    mcl::sleep_ms(2000);
}

}

#endif

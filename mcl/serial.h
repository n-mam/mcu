#ifndef SERIAL_H
#define SERIAL_H

#if defined (PICO)

#elif defined (STM32)
#include <uart.h>
#endif

namespace mcl {

struct serial {

    serial(uint8_t txPin, uint8_t rxPin, uint32_t baudRate = 115200, uint8_t wordLength = 8, bool enableParity = false)
        #if defined (STM32F4)
        : uart2(txPin, rxPin, USART2, GPIOA, baudRate, wordLength, enableParity)
        #elif defined (STM32F7)
        : uart2(txPin, rxPin, USART6, GPIOG, baudRate, wordLength, enableParity)
        #elif defined (STM32H7)
        : uart2(txPin, rxPin, UART4, GPIOA, baudRate, wordLength, enableParity)
        #endif
    {

    }

    void transmit(const uint8_t *message, size_t len) {
        #if defined (STM32)
        uart2.transmit(reinterpret_cast<const uint8_t*>(message), len);
        #endif
    }

    #if defined (STM32)
    mcl::uart uart2;
    #endif
};

void test_serial() {
    #if defined (STM32F4)
    // Create USART instance with TX on PA2, RX on PA3
    mcl::serial serial(2, 3, 115200, 8, false);
    const char *message = "Hello, USART! This is a message from blackpill\n";
    #elif defined (STM32F7)
    // Create USART instance with TX on PG14, RX on PG9
    mcl::serial serial(14, 9, 115200, 8, false);
    const char *message = "Hello, USART! This is a message from F7\n";
    #elif defined (STM32H7)
    // todo: copied from f7
    // Create USART instance with TX on PG14, RX on PG9
    mcl::serial serial(14, 9, 115200, 8, false);
    const char *message = "Hello, USART! This is a message from F7\n";
    #elif defined (PICO)
    // todo
    mcl::serial serial(1, 2, 115200, 8, false);
    const char *message = "UASRT to be implemented for pico\n";
    #endif
    while (1) {
        serial.transmit(reinterpret_cast<const uint8_t*>(message), strlen(message));
        mcl::sleep_ms(1000);
    }
}

}

#endif
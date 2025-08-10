#ifndef SERIAL_H
#define SERIAL_H

#if defined (PICO)

#elif defined (STM32)
#include <fx/uart.h>
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

}

#endif
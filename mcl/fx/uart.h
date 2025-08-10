#ifndef UART_H
#define UART_H

#include <stdint.h>

namespace mcl {

struct uart {

    uart(uint16_t txPin, uint16_t rxPin, USART_TypeDef *instance, GPIO_TypeDef *gpioPort = GPIOA,
            uint32_t baudRate = 115200, uint8_t wordLength = 8, bool enableParity = false)
        : txPin(txPin), rxPin(rxPin), gpioPort(gpioPort), instance(instance) {
            init(txPin, rxPin, baudRate, wordLength, enableParity);
    }

    void init(uint16_t txPin, uint16_t rxPin, uint32_t baudRate, uint8_t wordLength, bool enableParity) {
        // Enable clock for GPIO
        mcl::enableClockForGpio(gpioPort);
        // Clear mode bits for tx and rx
        gpioPort->MODER &= ~(0b11 << (txPin * 2));
        gpioPort->MODER &= ~(0b11 << (rxPin * 2));
        // Configure tx and rx pins for AF mode
        gpioPort->MODER |= (0b10 << (txPin * 2)) | (0b10 << (rxPin * 2));
        // Configure tx and rx pins for UART AF mode
        #if defined (STM32F4)
        uint8_t af_uart = 0b111; // AF7 for UART on STM32F4
        #elif defined (STM32F7)
        uint8_t af_uart = 0b1000; // AF8 for UART on STM32F7
        #elif defined (STM32H7)
        uint8_t af_uart = 0b1000; // AF8 for UART on STM32H7
        #endif
        if (txPin > 7) {
            gpioPort->AFR[1] |= (af_uart << ((txPin - 8) * 4));
        } else {
            gpioPort->AFR[0] |= (af_uart << (txPin * 4));
        }
        if (rxPin > 7) {
            gpioPort->AFR[1] |= (af_uart << ((rxPin - 8) * 4));
        } else {
            gpioPort->AFR[0] |= (af_uart << (rxPin * 4));
        }
        // Disable USART
        instance->CR1 &= ~USART_CR1_UE;
        // Enable clock for USART
        mcl::enableClockForUart(instance);
        // Configure word length
        if (wordLength == 9) {
            instance->CR1 |= USART_CR1_M; // Set M bit for 9 bits
        }
        // Configure parity
        if (enableParity) {
            instance->CR1 |= USART_CR1_PCE; // Enable parity control
        }
        // Set baud rate
        #if defined (STM32F4)
        instance->BRR = apb1PeripheralClock() / baudRate;
        #elif defined (STM32F7)
        instance->BRR = 108000000 / baudRate; //checkme
        #elif defined (STM32H7)
        instance->BRR = apb1PeripheralClock() / baudRate;
        #endif
        // Enable Transmitter and Receiver
        instance->CR1 |= USART_CR1_TE | USART_CR1_RE;
        // Enable USART
        instance->CR1 |= USART_CR1_UE;
    }

    void transmit(uint8_t data) {
        #if defined (STM32F4)
        while (!(instance->SR & USART_SR_TXE));
        instance->DR = data;
        #elif defined (STM32F7)
        while (!(instance->ISR & USART_ISR_TXE));
        instance->TDR = data;
        #elif defined(STM32H7)
        while (!(instance->ISR & USART_ISR_TXE_TXFNF));
        instance->TDR = data;
        #endif
    }

    void transmit(const uint8_t *data, uint16_t size) {
        for (uint16_t i = 0; i < size; ++i) {
            transmit(data[i]);
        }
    }

    uint8_t receive() {
        #if defined (STM32F4)
        while (!(instance->SR & USART_SR_RXNE));
        return instance->DR;
        #elif defined (STM32F7)
        while (!(instance->ISR & USART_ISR_RXNE));
        return instance->RDR;
        #elif defined(STM32H7)
        while (!(instance->ISR & USART_ISR_RXNE_RXFNE));
        return instance->RDR;
        #endif
    }

    void receive(uint8_t *data, uint16_t size) {
        for (uint16_t i = 0; i < size; ++i) {
            data[i] = receive();
        }
    }

    private:

    uint16_t txPin;
    uint16_t rxPin;
    GPIO_TypeDef *gpioPort;
    USART_TypeDef *instance;
};

}

#endif
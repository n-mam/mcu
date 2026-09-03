#ifndef UART_H
#define UART_H

#include <stdint.h>

namespace mcl {

struct uart {

    uart(uint16_t txPin, uint16_t rxPin,
            USART_TypeDef *instance,
            GPIO_TypeDef *gpioPort = GPIOA,
            uint32_t baudRate = 115200,
            uint8_t wordLength = 8,
            bool enableParity = false)
        : txPin(txPin),
          rxPin(rxPin),
          gpioPort(gpioPort),
          instance(instance),
          txDmaStream(DMA1_Stream6),
          rxDmaStream(DMA1_Stream5),
          rxReadPosition(0) {
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
        // Enable USART TX and RX DMA requests
        instance->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
        // Enable USART
        instance->CR1 |= USART_CR1_UE;
        // Initialize TX DMA
        initTxDma();
        // Initialize RX DMA
        initRxDma();
    }

    void initRxDma() {
        // DMA1 clock is already enabled by initTxDma(),
        // but keeping this here makes the function self-contained.
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
        // Disable RX DMA stream
        rxDmaStream->CR &= ~DMA_SxCR_EN;
        // Wait until actually disabled
        while (rxDmaStream->CR & DMA_SxCR_EN);
        // Clear configuration
        rxDmaStream->CR = 0;
        // USART2_RX:
        // DMA1 Stream 5, Channel 4
        rxDmaStream->CR |= (4 << DMA_SxCR_CHSEL_Pos);
        // Peripheral -> memory
        rxDmaStream->CR &= ~DMA_SxCR_DIR;
        // Increment memory address
        rxDmaStream->CR |= DMA_SxCR_MINC;
        // Peripheral size = 8 bits
        rxDmaStream->CR &= ~DMA_SxCR_PSIZE;
        // Memory size = 8 bits
        rxDmaStream->CR &= ~DMA_SxCR_MSIZE;
        // Circular mode
        rxDmaStream->CR |= DMA_SxCR_CIRC;
        // USART2 data register
        rxDmaStream->PAR = (uint32_t)&instance->DR;
        // RX buffer
        rxDmaStream->M0AR = (uint32_t)rxBuffer;
        // Number of bytes in buffer
        rxDmaStream->NDTR = RX_BUFFER_SIZE;
        // Start RX DMA
        rxDmaStream->CR |= DMA_SxCR_EN;
    }


    void initTxDma() {
        // Enable DMA1 clock
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
        // Disable DMA stream
        txDmaStream->CR &= ~DMA_SxCR_EN;
        // Wait until DMA stream is actually disabled
        while (txDmaStream->CR & DMA_SxCR_EN);
        // Clear all configuration
        txDmaStream->CR = 0;
        // USART2_TX:
        // DMA1 Stream 6, Channel 4
        txDmaStream->CR |= (4 << DMA_SxCR_CHSEL_Pos);
        // Memory address increments after each byte
        txDmaStream->CR |= DMA_SxCR_MINC;
        // Memory -> peripheral
        txDmaStream->CR |= DMA_SxCR_DIR_0;
        // Peripheral size = 8 bits
        txDmaStream->CR &= ~DMA_SxCR_PSIZE;
        // Memory size = 8 bits
        txDmaStream->CR &= ~DMA_SxCR_MSIZE;
        // Normal mode, not circular
        txDmaStream->CR &= ~DMA_SxCR_CIRC;
        // USART2 data register is the peripheral destination
        txDmaStream->PAR = (uint32_t)&instance->DR;
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

    void transmitDma(const uint8_t *data, uint16_t size) {
        // Don't start another transfer while DMA is active
        if (txDmaStream->CR & DMA_SxCR_EN) return;
        // Don't overflow our internal buffer
        if (size > TX_BUFFER_SIZE) return;
        // Copy caller's data into UART-owned buffer
        memcpy(txBuffer, data, size);
        // Clear DMA1 Stream 6 flags
        DMA1->HIFCR =
            DMA_HIFCR_CFEIF6  |
            DMA_HIFCR_CDMEIF6 |
            DMA_HIFCR_CTEIF6  |
            DMA_HIFCR_CHTIF6  |
            DMA_HIFCR_CTCIF6;
        // DMA reads from our persistent buffer
        txDmaStream->M0AR = (uint32_t)txBuffer;
        // Number of bytes
        txDmaStream->NDTR = size;
        // Start DMA
        txDmaStream->CR |= DMA_SxCR_EN;
    }

    void transmit(const uint8_t *data, uint16_t size) {
        transmitDma(data, size);
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

    std::string processRx() {
        const uint16_t writePosition =
            RX_BUFFER_SIZE - rxDmaStream->NDTR;
        while (rxReadPosition != writePosition) {
            const char c = static_cast<char>(rxBuffer[rxReadPosition]);
            rxReadPosition = (rxReadPosition + 1) % RX_BUFFER_SIZE;
            if (c == '\r') continue;
            if (c == '\n') {
                if (commandLength == 0) continue;
                std::string command(commandBuffer, commandLength);
                commandLength = 0;
                return command;
            }
            if (commandLength < COMMAND_BUFFER_SIZE - 1) {
                commandBuffer[commandLength++] = c;
            } else {
                commandLength = 0;
            }
        }
        return {};
    }


    bool txBusy() {
        return (txDmaStream->CR & DMA_SxCR_EN) != 0;
    }

    private:

    uint16_t txPin;
    uint16_t rxPin;
    GPIO_TypeDef *gpioPort;
    USART_TypeDef *instance;
    DMA_Stream_TypeDef *txDmaStream;
    DMA_Stream_TypeDef *rxDmaStream;
    static constexpr uint16_t TX_BUFFER_SIZE = 768;
    uint8_t txBuffer[TX_BUFFER_SIZE];
    static constexpr uint16_t RX_BUFFER_SIZE = 256;
    uint8_t rxBuffer[RX_BUFFER_SIZE];
    static constexpr uint16_t COMMAND_BUFFER_SIZE = 128;
    char commandBuffer[COMMAND_BUFFER_SIZE];
    uint16_t rxReadPosition;
    uint16_t commandLength = 0;
};

}

#endif
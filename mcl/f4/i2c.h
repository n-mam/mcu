#ifndef I2C_F4_H
#define I2C_F4_H

#include <fx/clock.h>

#include <stdint.h>
#include <iostream>

namespace mcl {

inline void i2c_init(uint8_t scl, uint8_t sda, uint freq, I2C_TypeDef* I2Cx, GPIO_TypeDef *GPIOx) {
    // Enable clock to GPIO
    enableClockForGpio(GPIOx);
	// Clear mode first
	GPIOx->MODER &= ~((0b11 << (scl * 2)) | (0b11 << (sda * 2)));
    // Mark port B's pin 6 and 7 to operate in AF mode (0b10)
    GPIOx->MODER |= (0b10 << (scl * 2)) | (0b10 << (sda * 2));
    // Mark I2c(AF4) as the AF for sda and scl in AFR[0/1]
	if (sda < 8) {
        GPIOx->AFR[0] |= (0b0100 << (sda * 4));
	} else {
		GPIOx->AFR[1] |= (0b0100 << (sda * 4));
	}
	if (scl < 8) {
        GPIOx->AFR[0] |= (0b0100 << (scl * 4));
	} else {
		GPIOx->AFR[1] |= (0b0100 << (scl * 4));
	}
    // Set both pins as open drain
    GPIOx->OTYPER |= (0b1 << scl) | (0b1 << sda);
    // Mark pins as high speed (0b11)
    GPIOx->OSPEEDR |= (0b11 << (scl * 2)) | (0b11 << (sda * 2));
    // Enable internal pull-up resistors for the specified pins (01) if needed
    GPIOx->PUPDR &= ~(0b11 << (sda * 2)); // Clear previous settings
    GPIOx->PUPDR |= (1 << (sda * 2));  // Enable pull-up
    GPIOx->PUPDR &= ~(0b11 << (scl * 2)); // Clear previous settings
    GPIOx->PUPDR |= (1 << (scl * 2));  // Enable pull-up
    // Enable APB1 clock to I2Cx
	if (I2Cx == I2C1) {
    	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	} else if (I2Cx == I2C2) {
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	}
    // Disable I2Cx
    I2Cx->CR1 &= ~I2C_CR1_PE;
    // Write 1 to 15th position to reset I2C
    I2Cx->CR1 |= (1 << 15);
    // Write 0 to 15th position to pull the I2C from reset
    I2Cx->CR1 &= ~(1 << 15);
    // Configure I2C prepheral clock frequency
    // The minimum allowed frequency is 2 MHz, the maximum frequency is
    // limited by the maximum APB1 frequency and cannot exceed 50 MHz
    // (peripheral intrinsic maximum limit).
    uint32_t apb1FrequencyMhz = apb1PeripheralClock() / 1000000;
    I2Cx->CR2 |= apb1FrequencyMhz;
    // Frequency at which I2C SCL should operate
    // 100KHz scl implies a time period of 10us
    // For Standard mode (100 kHz), both Thigh and Tlow are typically
    // required to be at least 4.7µs. 10us Time period implies a Thigh
    // and Tlow of 5us. For SM Mode: Thigh = CCR * TPCLK1
    // Thigh = Tr(scl) + Tw(scl) = 1000ns + 4.0us = (1 + 4) us (as per DS)
    // TPCLK1 is 1/fPLCK1 == 1/apb1FrequencyMhz, therefore
    // 5us = CCR * (1/apb1FrequencyMhz)
    if (freq <= 100*1000) { // 100 KHz
        I2Cx->CCR = (1 + 4) * apb1FrequencyMhz;
        // TRISE = (Tr(scl) / TPCLK1) + 1
        // TRISE = (1000 ns / (1/48) us) + 1 = 49
        I2Cx->TRISE = apb1FrequencyMhz + 1; //0x31 49
    } else { // 400 KHz
        const uint8_t DutyCycle = 0;
        if (DutyCycle == 0) {
            auto ccr = apb1PeripheralClock() / (freq * 3);
            ccr = (ccr < 1) ? 1 : ccr;
            I2Cx->CCR = ccr;
        } else {
            auto ccr = apb1PeripheralClock() / (freq * 25);
            ccr = (ccr < 1) ? 1 : ccr;
            I2Cx->CCR = ccr;
        }
        I2Cx->TRISE = ((apb1FrequencyMhz * 300) / 1000) + 1;
    }
    // Enable I2Cx
    I2Cx->CR1 |= I2C_CR1_PE;
}

inline void i2c_deinit() {

}

void send_device_address(uint8_t dev_addr, uint8_t rw, I2C_TypeDef* I2Cx = I2C1) {
    // Send device address with read/write bit set/unset respectively
    I2Cx->DR = (dev_addr << 1) | rw;
    while (!(I2Cx->SR1 & I2C_SR1_ADDR)) {};
    // Clear ADDR flag
    (void)I2Cx->SR2;
}

inline void i2c_start(I2C_TypeDef* I2Cx = I2C1) {
    // Wait until the BUSY flag is cleared
    // while (I2Cx->SR2 & I2C_SR2_BUSY) { std::cout << I2Cx->SR2 << std::endl; }
    // Set the START bit in CR1 register to initiate start condition
    I2Cx->CR1 |= I2C_CR1_START;
    // Wait until START bit is set
    while (!(I2Cx->SR1 & I2C_SR1_SB)) {}
}

inline void i2c_stop(I2C_TypeDef* I2Cx = I2C1) {
    // Set the STOP bit in CR1 register to initiate stop condition
    I2Cx->CR1 |= I2C_CR1_STOP;
    // Wait until the STOP bit is cleared
    while (I2Cx->CR1 & I2C_CR1_STOP) {}
    // Wait until the BUSY flag is cleared
    while (I2Cx->SR2 & I2C_SR2_BUSY) {}
}

inline int8_t i2c_write(uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, uint8_t len, bool sendReg, I2C_TypeDef* I2Cx = I2C1) {
    // Start condition
    i2c_start();
    // Send device address with write bit set
    send_device_address(dev_addr, 0x00);
    if (sendReg) {
        // Send memory address
        I2Cx->DR = mem_addr;
        while (!(I2Cx->SR1 & I2C_SR1_BTF)) {}
    }
    // Send data
    for (int i = 0; i < len; i++) {
        // Wait for TXE flag to be set
        while (!(I2Cx->SR1 & I2C_SR1_TXE)) {};
        I2Cx->DR = data[i];
        while (!(I2Cx->SR1 & I2C_SR1_BTF)) {}
    }
    // Stop condition
    i2c_stop();
    return len;
}

inline int8_t i2c_read(uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, uint16_t len, bool sendReg, I2C_TypeDef* I2Cx = I2C1) {
    // Start condition
    i2c_start();
    if (sendReg) {
        // Send device address with write bit set
        send_device_address(dev_addr, 0x00);
        // Send memory address
        I2Cx->DR = mem_addr;
        while (!(I2Cx->SR1 & I2C_SR1_BTF)) {}
        // Repeated Start condition
        i2c_start();
    }
    // Send device address with read bit set
    send_device_address(dev_addr, 0x01);
    // Enable ACK
    I2Cx->CR1 |= I2C_CR1_ACK;
    // Read data
    for (int i = 0; i < len; i++) {
        if (i == (len - 1)) {
            // After second last byte: Disable ACK
            I2Cx->CR1 &= ~I2C_CR1_ACK;
            // After second last byte: Generate Stop condition
            i2c_stop();
            // Wait for RXNE flag to be set
            while (!(I2Cx->SR1 & I2C_SR1_RXNE)) {};
            data[i] = I2Cx->DR;
            // Wait for BTF flag to be set
            while (I2Cx->SR1 & I2C_SR1_BTF) {};
        } else {
            // Read intermediate bytes
            while (!(I2Cx->SR1 & I2C_SR1_RXNE)) {};
            data[i] = I2Cx->DR;
        }
    }
    //dumpBuffer(data, len);
    return len;
}

inline void i2c_bus_scan(I2C_TypeDef* I2Cx) {
    // I2C address range is from 8 to 119. Address 0 is reserved as
    // a broadcast address, addresses 1 to 7 are reserved for other
    // purposes, and addresses 120 to 127 are reserved for future use
    i2c_init(6, 7, 400*1000, I2Cx, GPIOB);
    //i2c_init(6, 7, 100*1000);
    for (uint8_t i = 8; i <= 119; i++) {
        auto rc = 0;
        // Generate start condition
        i2c_start();
        // Send device address with write bit set
        I2Cx->DR = (i << 1) | 0;
        // Wait for end of address transmission
        while (!(I2Cx->SR1 & I2C_SR1_ADDR)) {
            // check if the Acknowledge Failure flag is set
            if (I2Cx->SR1 & I2C_SR1_AF) {
                // Device did not acknowledge
                rc = 1;
                // Clear Acknowledge Failure flag
                I2Cx->SR1 &= ~I2C_SR1_AF;
                break;
            }
        }
        // Clear ADDR flag
        (void)I2Cx->SR2;
        // Generate stop condition
        i2c_stop();
        if (rc == 0) {
            LOG << "I2C device detected at bus address: " << std::hex << (unsigned)i;
        }
    }
}

}

#endif
#ifndef I2C_H7_H
#define I2C_H7_H

#include <stdint.h>
#include <iostream>

#include <fx/clock.h>

namespace mcl {

inline void i2c_init(uint8_t scl, uint8_t sda, uint freq = 400*1000, I2C_TypeDef* I2Cx = I2C1, GPIO_TypeDef *GPIOx = GPIOB) {
	// Enable AHB1 clock to GPIO
	if (GPIOx == GPIOA) {
    	RCC->AHB1ENR |= RCC_AHB4ENR_GPIOAEN;
	} else if (GPIOx == GPIOB) {
		RCC->AHB1ENR |= RCC_AHB4ENR_GPIOBEN;
	}
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
	// if (I2Cx == I2C1) {
    // 	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
	// } else if (I2Cx == I2C2) {
	// 	RCC->APB1ENR2 |= RCC_APB1ENR1_I2C1EN;
	// }
    // Disable I2Cx
    I2Cx->CR1 &= ~I2C_CR1_PE;
    // Raise 1000ns
    // Fall 300ns
    // SM 100khz
    // Analog filter enabled
	I2Cx->TIMINGR |= 0x40D32A31;
    // Enable I2Cx
    I2Cx->CR1 |= I2C_CR1_PE;
}

inline int8_t i2c_write(uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, uint8_t len, bool sendReg, I2C_TypeDef* I2Cx = I2C1) {
	// Clear CR2
	I2Cx->CR2 &= 0U;
	// Set slave address
	I2Cx->CR2 |= dev_addr << 1;
	// Set transfer size (for address + data)
	I2Cx->CR2 |= (1 + len) << I2C_CR2_NBYTES_Pos;
	// Set automatic end mode
	I2Cx->CR2 |= I2C_CR2_AUTOEND;
	// Set data transmit register before start
	I2Cx->TXDR = mem_addr;
	// Generate start condition
	I2Cx->CR2 |= I2C_CR2_START;
	mcl::sleep_ms(50);
	// wait for the STOP flag to be raised
	while (!(I2Cx->ISR & I2C_ISR_STOPF)) {
		// check TXIS flag value in ISR register
		if (I2Cx->ISR & I2C_ISR_TXIS) {
			I2Cx->TXDR = *(data++);
        }
	}
	// Clear stop flag
	I2Cx->ICR |= I2C_ICR_STOPCF;
	return len;
}

inline int8_t i2c_read(uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, uint16_t len, bool sendReg, I2C_TypeDef* I2Cx = I2C1) {
	// Clear CR2
	I2Cx->CR2 &= 0U;
	// Set slave address
	I2Cx->CR2 = dev_addr << 1;
	// 7-bit addressing
	I2Cx->CR2 &= ~I2C_CR2_ADD10;
	// Set transfer len to 1 for mem_addr write operation
	I2Cx->CR2 |= (1 << I2C_CR2_NBYTES_Pos);
	// Set the mode to write mode
	I2Cx->CR2 &= ~I2C_CR2_RD_WRN;
	// Set software based (TC) end for address transmission
	I2Cx->CR2 &= ~I2C_CR2_AUTOEND;
	// Generate start
	I2Cx->CR2 |= I2C_CR2_START;
	// Wait for address transfer to complete
	while(!(I2Cx->ISR & I2C_ISR_TC)) {
		// Check if TX buffer is empty
		if(I2Cx->ISR & I2C_ISR_TXE) {
			// send the memory address
			I2Cx->TXDR = mem_addr;
		}
	}
	// Reset I2C
	I2Cx->CR1 &= ~I2C_CR1_PE;
	I2Cx->CR1 |= I2C_CR1_PE;
	// Set slave address
	I2Cx->CR2 = dev_addr << 1;
	// Set mode to read operation
	I2Cx->CR2 |= I2C_CR2_RD_WRN;
	// Set length to the required length
	I2Cx->CR2 |= (len << I2C_CR2_NBYTES_Pos);
	// Set automatic end mode for read
	I2Cx->CR2 |= I2C_CR2_AUTOEND;
	// Generate start condition
	I2Cx->CR2 |= I2C_CR2_START;
	// wait until stop is generated
	while(!(I2Cx->ISR & I2C_ISR_STOPF)) {
        // RX buffer is not empty
        if (I2Cx->ISR & I2C_ISR_RXNE) {
            *data++ = I2Cx->RXDR;
        }
	}
	// Clear stop flag
	I2Cx->ICR |= I2C_ICR_STOPCF;
	return len;
}

}

#endif
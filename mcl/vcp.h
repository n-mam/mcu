#ifndef VCP_H
#define VCP_H

#include <cstring>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

#include <stm32h7xx_hal.h>

extern "C" {

UART_HandleTypeDef huart;

void stlink_uart(const char *format, ...) {
    char buf[128] = { 0 };
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    HAL_UART_Transmit(&huart, (const uint8_t *)buf, strlen(buf), 10);
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if (huart->Instance == USART3) {
        // Initializes the peripherals clock
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
            return;
        }
        // Peripheral clock enable
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        // USART3 GPIO Configuration
        // PD8     ------> USART3_TX
        // PD9     ------> USART3_RX
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    }
}

void MX_USART3_UART_Init(void) {
    huart.Instance = USART3;
    huart.Init.BaudRate = 115200;
    huart.Init.Mode = UART_MODE_TX_RX;
    huart.Init.Parity = UART_PARITY_NONE;
    huart.Init.StopBits = UART_STOPBITS_1;
    huart.Init.WordLength = UART_WORDLENGTH_8B;
    huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart.Init.OverSampling = UART_OVERSAMPLING_16;
    huart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart) != HAL_OK) {
        return;
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        return;
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        return;
    }
    if (HAL_UARTEx_DisableFifoMode(&huart) != HAL_OK) {
        return;
    }
}

} /* extern "C" */

#endif
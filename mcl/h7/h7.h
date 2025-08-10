#ifndef H7_H
#define H7_H

#include <stm32h7xx_hal.h>

extern "C" {

#if defined (CORE_CM7)

void MPU_Config(void) {
    MPU_Region_InitTypeDef MPU_InitStruct = {0};
    // Disables the MPU
    HAL_MPU_Disable();
    // Initializes and configures the Region and the memory to be protected
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x38000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
    MPU_InitStruct.SubRegionDisable = 0x0;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    // Enables the MPU
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    // Supply configuration update enable
    HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
    //Configure the main internal regulator output voltage
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
    // Initializes the RCC Oscillators according to the
    // specified parameters in the RCC_OscInitTypeDef structure.
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 100;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    // Initializes the CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void initialize_h7(void) {
    MPU_Config();
    SCB_EnableICache();
    SCB_EnableDCache();
    // Wait until cpu2 boots and enters in stop mode
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET);
    // Reset of all peripherals, Initializes the Flash interface and the Systick
    HAL_Init();
    // Configure the system clock
    SystemClock_Config();
    // When system initialization is finished, Cortex-M7
    // will release Cortex-M4 by means of HSEM notification
    // HW semaphore Clock enable
    __HAL_RCC_HSEM_CLK_ENABLE();
    // Take HSEM
    HAL_HSEM_FastTake(HSEM_ID_0);
    // Release HSEM in order to notify the CPU2(CM4)
    HAL_HSEM_Release(HSEM_ID_0, 0);
    // Wait until CPU2 wakes up from stop mode
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET);
    // stlink vcp
    MX_USART3_UART_Init();
}

void HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    HAL_NVIC_SetPriority(HSEM1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(HSEM1_IRQn); // Enable IRQ for CM7
}

void HSEM1_IRQHandler(void) {
    HAL_HSEM_IRQHandler();
}

#elif defined (CORE_CM4)

void initialize_h7(void) {
    // HW semaphore Clock enable
    __HAL_RCC_HSEM_CLK_ENABLE();
    // Activate HSEM notification for Cortex-M4
    HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
    // Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
    // perform system initialization (system clock config, external memory configuration.. )
    HAL_PWREx_ClearPendingEvent();
    HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
    // Clear HSEM flag
    __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
    // Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();
    // stlink vcp
    //MX_USART3_UART_Init();
}

void HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    HAL_NVIC_SetPriority(HSEM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(HSEM2_IRQn); // Enable IRQ for CM4
}

void HSEM2_IRQHandler(void) {
    HAL_HSEM_IRQHandler();
}

#endif

void NMI_Handler(void) {
    while (1) {}
}
void HardFault_Handler(void) {
    while (1) {}
}
void MemManage_Handler(void) {
    while (1) {}
}
void BusFault_Handler(void) {
    while (1) {}
}
void UsageFault_Handler(void) {
    while (1) {}
}
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) {
    HAL_IncTick();
}

} /* extern "C" */

#endif
#ifndef CLOCK_H
#define CLOCK_H

#include <vector>
#include <thread>
#include <stdint.h>

namespace mcl {

#ifndef STM32H7
extern "C" {
    volatile uint32_t ticks = 0;
    void SysTick_Handler(void) { ticks++; }
}
#endif

inline auto get_tick() {
    #ifndef STM32H7
    return ticks;
    #else
    return HAL_GetTick();
    #endif
}

inline uint64_t time_ms() {
    #if defined (PICO)
    return time_us_64() / 1000;
    #elif defined (STM32)
    return get_tick();
    #endif
}

void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

inline void delay_ms(uint32_t milliseconds) {
    uint32_t start = get_tick();
    uint32_t end = start + milliseconds;
    if (end < start) {
        // Wait for ticks to wrap around
        while (get_tick() > start);
    }
    // Wait for ticks to reach end
    while (get_tick() < end);
}

inline void sleep_ms(unsigned int milliseconds) {
    #if defined (PICO)
    ::sleep_ms(milliseconds);
    #elif defined (STM32)
    delay_ms(milliseconds);
    #else
    std::this_thread::sleep_for(
        std::chrono::milliseconds(
            static_cast<long>(milliseconds)));
    #endif
}

#if defined (STM32)
inline uint32_t apb1TimerClock() {
    #if defined (STM32F411xE)
    return 96000000;
    #elif defined (STM32F446xx)
    return 90000000;
    #elif defined (STM32F767)
    return 108000000;
    #elif defined (STM32H755)
    return 200000000;
    #else
    return 0;
    #endif
}

inline uint32_t apb1PeripheralClock() {
    #if defined (STM32F411xE)
    return 48000000;
    #elif defined (STM32F446xx)
    return 45000000;
    #elif defined (STM32F7)
    return 54000000;
    #elif defined (STM32H7)
    return 100000000;
    #endif
}

// apb2 timer clock
// apb2 peripheral clock

#if defined(STM32F4) || defined(STM32F7)
void configure_flash() {
    #if defined (STM32F4)
    // Enable prefetch
    FLASH->ACR |= FLASH_ACR_PRFTEN;
    // Enable instruction cache
    FLASH->ACR |= FLASH_ACR_ICEN;
    // Enable data cache
    FLASH->ACR |= FLASH_ACR_DCEN;
    // Clear current latency settings
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    #if defined (STM32F411xE)
    // Set flash wait states
    FLASH->ACR |= FLASH_ACR_LATENCY_3WS;
    #elif defined (STM32F446xx)
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;
    #endif
    #elif defined (STM32F7)
    FLASH->ACR |= FLASH_ACR_PRFTEN;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_7WS;
    // I and D Caches are enabled by default on F7
    // SCB_EnableICache();
    // SCB_EnableDCache();
    SCB_InvalidateICache();
    SCB_InvalidateDCache();
    #endif
}

void configure_pll(const std::vector<uint16_t> factors) {
    uint16_t M, N, P, Q, R;
    M = N = P = Q = R = 0;
    if (factors.size() > 0) M = factors[0];
    if (factors.size() > 1) N = factors[1];
    if (factors.size() > 2) P = factors[2];
    if (factors.size() > 3) Q = factors[3];
    if (factors.size() > 4) R = factors[4];
    // Clear PLLM
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
    // Set PLLM to 25
    RCC->PLLCFGR |= (M << RCC_PLLCFGR_PLLM_Pos);
    // Clear PLLN
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
    // Set PLLN to 192
    RCC->PLLCFGR |= (N << RCC_PLLCFGR_PLLN_Pos);
    // Clear PLLP
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;
    // just ensure PLLP is cleared.
    // This results in PLLP = 00 (divide by 2)
    // Clear PLLQ (USB and SDIO clocks)
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;
    // Set PLLQ to 4
    RCC->PLLCFGR |= (Q << RCC_PLLCFGR_PLLQ_Pos);
    #if defined (STM32F7)
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLR;
    // Set PLLR to 2
    RCC->PLLCFGR |= (2 << RCC_PLLCFGR_PLLR_Pos);
    #endif
    // Set HSE as the PLL source
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
}

void clock_init(const std::vector<uint16_t> factors) {
    // configure flash
    configure_flash();
    // enable power control clock
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    // set voltage scale mode 1
    #if defined (STM32F4)
    PWR->CR |= PWR_CR_VOS_0 | PWR_CR_VOS_1;
    #elif defined (STM32F7)
    PWR->CR1 |= PWR_CR1_VOS_0 | PWR_CR1_VOS_1;
    #endif
    // enable HSE oscillator
    RCC->CR |= RCC_CR_HSEON;
    // wait until HSE is ready
    while ((RCC->CR & RCC_CR_HSERDY) == 0);
    // configure PLL
    configure_pll(factors);
    // turn PLL on
    RCC->CR |= RCC_CR_PLLON;
    // wait until the PLL is ready
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
    // enable overdrive on f446 to 180 Mhz
    #if defined (STM32F446xx)
    // set the ODEN bit to enable Over-Drive mode
    PWR->CR |= PWR_CR_ODEN;
    // wait for the ODRDY flag to be set in the PWR_CSR register
    while ((PWR->CSR & PWR_CSR_ODRDY) == 0) {}
    // set the ODSW bit to switch the voltage regulator to Over-Drive mode
    PWR->CR |= PWR_CR_ODSWEN;
    // wait for the ODSWRDY flag to be set in the PWR_CSR register
    while ((PWR->CSR & PWR_CSR_ODSWRDY) == 0) {}
    #endif
    // clear AHB prescaler to zero (no division)
    RCC->CFGR &= ~RCC_CFGR_HPRE;
    #if defined (STM32F411xE)
    // Set the APB1 Low speed prescaler to 2
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    // Set the APB2 high speed prescaler to 1
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
    #elif defined(STM32F446xx)
    // Set the APB1 Low speed prescaler to 4
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
    // Set the APB2 high speed prescaler to 2
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
    #elif defined (STM32F7)
    // Set the APB1 Low speed prescaler to 4
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // APB1 prescaler to 4
    // Set the APB2 high speed prescaler to 2
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; // APB2 prescaler to 2
    #endif
    // Switch system clock to PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    // Wait until PLL is used as the system clock
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    // Update SystemCoreClock
    SystemCoreClockUpdate();
}
#endif

inline void enableClockForTimer(TIM_TypeDef *timer) {
    if (timer == TIM2) {
        #if defined (STM32F4) || defined (STM32F7)
            RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
        #elif defined (STM32H7)
            RCC->APB1LENR |= RCC_APB1LENR_TIM2EN;
        #endif
    } else if (timer == TIM3) {
        #if defined (STM32F4) || defined (STM32F7)
            RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
        #elif defined (STM32H7)
            RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
        #endif
    } else if (timer == TIM4) {
        #if defined (STM32F4) || defined (STM32F7)
            RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
        #elif defined (STM32H7)
            RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;
        #endif
    } else if (timer == TIM5) {
        #if defined (STM32F4) || defined (STM32F7)
            RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
        #elif defined (STM32H7)
            RCC->APB1LENR |= RCC_APB1LENR_TIM5EN;
        #endif
    }
}

inline void enableClockForGpio(GPIO_TypeDef *port) {
    #if defined (STM32F4) || defined (STM32F7)
    if (port == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    } else if (port == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    } else if (port == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    }
    #if defined (STM32F7)
    else if (port == GPIOG) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    }
    #endif
    #elif defined (STM32H7)
    if (port == GPIOA) {
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;
    } else if (port == GPIOB) {
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
    } else if (port == GPIOC) {
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;
    } else if (port == GPIOG) {
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIOGEN;
    } else if (port == GPIOE) {
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;
    } else if (port == GPIOD) {
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;
    }
    #endif
}

inline void enableClockForUart(USART_TypeDef *instance) {
    #if defined (STM32F4) || defined (STM32F7)
    if (instance == USART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    } else if (instance == USART2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    } else if (instance == USART6) {
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    }
    #elif defined (STM32H7)
    if (instance == USART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    } else if (instance == USART2) {
        RCC->APB1LENR |= RCC_APB1LENR_USART2EN;
    } else if (instance == USART6) {
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    } else if (instance == UART4) {
        RCC->APB1LENR |= RCC_APB1LENR_UART4EN;
    }
    #endif
}
#endif

}

#endif
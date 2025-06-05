#include "stm32f10x.h"

uint32_t SystemCoreClock = 72000000; // Установи сюда свою рабочую частоту

void SystemInit(void) {
    RCC->CR |= RCC_CR_HSEON;                     // Включаем HSE (внешний кварц)
    while (!(RCC->CR & RCC_CR_HSERDY));          // Ждём, пока HSE стабилизируется

    RCC->CFGR |= RCC_CFGR_PLLSRC;                // PLL от HSE
    RCC->CFGR |= RCC_CFGR_PLLMULL9;              // Множитель 9 -> 8 МГц * 9 = 72 МГц

    RCC->CR |= RCC_CR_PLLON;                     // Включаем PLL
    while (!(RCC->CR & RCC_CR_PLLRDY));          // Ждём, пока PLL стабилизируется

    RCC->CFGR |= RCC_CFGR_SW_PLL;                // Переключаем системную тактовую на PLL
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Ждём завершения переключения
}



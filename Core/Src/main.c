#include "stm32f10x.h"
#include "system_stm32f10x.h"

#define LED_COUNT 23

// GPIO ports for each LED
GPIO_TypeDef* led_ports[LED_COUNT] = {
    GPIOB, GPIOB, GPIOB, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA,
    GPIOA, GPIOA, GPIOA, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB,
    GPIOB, GPIOB, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA
};

// Corresponding pin numbers
uint8_t led_pins[LED_COUNT] = {
    11, 10, 1, 7, 6, 5, 4, 3,
    2, 1, 0, 9, 8, 7, 6, 5,
    4, 3, 15, 12, 11, 10, 9
};

volatile uint32_t systick_ms = 0;
volatile uint8_t run_sequence = 0;

// Simple millisecond delay using SysTick
void delay_ms(uint32_t ms) {
    uint32_t start = systick_ms;
    while ((systick_ms - start) < ms);
}

// SysTick interrupt handler (1ms tick)
void SysTick_Handler(void) {
    systick_ms++;
}

// Configure GPIO and EXTI
void GPIO_Config(void) {
    // Enable clocks for GPIOA, GPIOB, GPIOC and AFIO
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_IOPCEN |
                    RCC_APB2ENR_AFIOEN;

    // Configure LED pins as push-pull output, 10 MHz
    for (int i = 0; i < LED_COUNT; i++) {
        GPIO_TypeDef* port = led_ports[i];
        uint8_t pin = led_pins[i];

        if (pin < 8) {
            port->CRL &= ~(0xF << (pin * 4));
            port->CRL |=  (0x1 << (pin * 4)); // Output mode, max speed 10 MHz
        } else {
            uint8_t shift = (pin - 8) * 4;
            port->CRH &= ~(0xF << shift);
            port->CRH |=  (0x1 << shift);
        }

        port->BRR = (1 << pin); // Set LOW initially
    }

    // Configure PA0 as input with pull-up
    GPIOA->CRL &= ~(0xF << (0 * 4));
    GPIOA->CRL |=  (0x8 << (0 * 4)); // Input with pull-up/down
    GPIOA->ODR |= (1 << 0); // Pull-up enabled

    // EXTI0 configuration for PA0 (falling edge)
    AFIO->EXTICR[0] &= ~AFIO_EXTICR1_EXTI0; // Select PA0
    EXTI->IMR |= EXTI_IMR_MR0;              // Unmask EXTI0
    EXTI->FTSR |= EXTI_FTSR_TR0;            // Trigger on falling edge
    NVIC_EnableIRQ(EXTI0_IRQn);             // Enable EXTI0 IRQ
}

// EXTI0 interrupt handler
void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR |= EXTI_PR_PR0; // Clear pending bit
        run_sequence = 1;
    }
}

// Run LED sequence
void run_gpio_sequence(void) {
    for (int i = 0; i < LED_COUNT; i++) {
        led_ports[i]->BSRR = (1 << led_pins[i]); // Set pin
        delay_ms(50);
        led_ports[i]->BRR = (1 << led_pins[i]);  // Reset pin
    }
    run_sequence = 0;
}

int main(void) {
    SystemInit(); // CMSIS system clock config
    SysTick_Config(SystemCoreClock / 1000); // 1ms SysTick
    GPIO_Config();

    while (1) {
        if (run_sequence) {
            run_gpio_sequence();
        }
    }
}

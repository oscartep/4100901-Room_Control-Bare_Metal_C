# Módulo RCC (Reset and Clock Control)

El periférico RCC es fundamental para el funcionamiento del microcontrolador, ya que gestiona la habilitación/deshabilitación de los relojes para los periféricos. ***Sin un reloj habilitado, un periférico no funcionará***.

**Archivos:** `Inc/rcc.h`, `Src/rcc.c`

**Referencia Principal:** RM0351, Sección "6. Reset and clock control (RCC)".

## Objetivos del Módulo RCC
1.  Proporcionar funciones para habilitar los relojes de los periféricos GPIO, TIM3, USART2, y SYSCFG.
2.  Definir las frecuencias de reloj que se usarán como base para la configuración de otros periféricos (SysTick, UART, TIM), consistentes con los talleres anteriores del curso.

## 1. `Inc/rcc.h`

```c
#ifndef RCC_H
#define RCC_H

#include <stdint.h>
#include "gpio.h"

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t ICSCR;
    volatile uint32_t CFGR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t PLLSAI1CFGR;
    volatile uint32_t PLLSAI2CFGR;
    volatile uint32_t CIER;
    volatile uint32_t CIFR;
    volatile uint32_t CICR;
    uint32_t RESERVED1;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    uint32_t RESERVED2;
    volatile uint32_t APB1RSTR1;
    volatile uint32_t APB1RSTR2;
    volatile uint32_t APB2RSTR;
    uint32_t RESERVED3;
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    uint32_t RESERVED4;
    volatile uint32_t APB1ENR1;
    volatile uint32_t APB1ENR2;
    volatile uint32_t APB2ENR;
} RCC_TypeDef;


#define RCC_BASE (0x40021000U)
#define RCC ((RCC_TypeDef *)RCC_BASE)

// Macros
#define SYSCLK_FREQ_HZ    4000000UL      // 4MHz
#define HCLK_FREQ_HZ      SYSCLK_FREQ_HZ // HCLK Prescaler = 1
#define PCLK1_FREQ_HZ     HCLK_FREQ_HZ   // APB1 Prescaler = 1
#define PCLK2_FREQ_HZ     HCLK_FREQ_HZ   // APB2 Prescaler = 1
#define TIM_PCLK_FREQ_HZ  PCLK1_FREQ_HZ  // TIM3 está en APB1

// Prototipos de funciones
void rcc_gpio_clock_enable(GPIO_TypeDef *gpio_port);
void rcc_syscfg_clock_enable(void);
void rcc_usart2_clock_enable(void);
void rcc_tim3_clock_enable(void);

#endif // RCC_H

```

## 2. `Src/rcc.c`

```c
#include "rcc.h"

void rcc_gpio_clock_enable(GPIO_TypeDef *gpio_port)
{
    if (gpio_port == GPIOA) {
        RCC->AHB2ENR |= 0x01 << 0;
    } else if (gpio_port == GPIOB) {
        RCC->AHB2ENR |= 0x01 << 1;
    } else if (gpio_port == GPIOC) {
        RCC->AHB2ENR |= 0x01 << 2;
    }
    // Añadir más puertos GPIO si son necesarios (D, E, F, G, H)
}

void rcc_syscfg_clock_enable(void)
{
    RCC->APB2ENR |= 0x01 << 0; // SYSCFG clock enable
}

void rcc_usart2_clock_enable(void)
{
    RCC->APB1ENR1 |= 0x01 << 17; // USART2 clock enable
}

void rcc_tim3_clock_enable(void)
{
    RCC->APB1ENR1 |= 0x01 << 1; // TIM3 clock enable
}

```

Siguiente módulo: [System Tick Timer (SYSTICK.md)](SYSTICK.md).
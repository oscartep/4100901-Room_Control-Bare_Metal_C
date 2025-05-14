# Módulo TIM (Timers) - PWM con TIM3_CH1

Los temporizadores de propósito general (TIM) son periféricos muy versátiles que pueden usarse para medir tiempo, generar retardos, contar eventos y generar señales PWM (Pulse Width Modulation), entre otras cosas. Usaremos TIM3 para generar una señal PWM en su canal 1 (TIM3_CH1), que está disponible en el pin PA6.

**Archivos:** `Inc/tim.h`, `Src/tim.c`

**Pin:** PA6 (TIM3_CH1)

**Función Alternativa:** AF2 para TIM3_CH1 en PA6.

**Referencia Principal:** RM0351, Sección "31. General-purpose timers (TIM2/3/4/5)".

## Objetivos del Módulo TIM
1.  Definir la estructura `TIM_TypeDef` para el acceso a registros de un temporizador de propósito general.
2.  Configurar TIM3 en modo PWM para generar una señal en el pin PA6 (TIM3_CH1).
3.  Establecer una frecuencia para la señal PWM (ej. 1 kHz), utilizando `TIM_PCLK_FREQ_HZ` (definido en `rcc.h`).
4.  Proporcionar una función para cambiar el ciclo de trabajo (duty cycle) de la señal PWM.

## 1. `Inc/tim.h`

```c
#ifndef TIM_H
#define TIM_H

#include <stdint.h>


typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RESERVED1;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t RESERVED2;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
} TIM_TypeDef;


#define TIM3_BASE           (0x40000400UL) // TIM3 está en APB1
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)

// Prototipos de funciones
void tim3_ch1_pwm_init(uint32_t pwm_freq_hz);

void tim3_ch1_pwm_set_duty_cycle(uint8_t duty_cycle_percent); // duty_cycle en % (0-100)

#endif // TIM_H

```

## 2. `Src/tim.c`

```c
#include "tim.h"
#include "rcc.h"  // Para rcc_tim3_clock_enable y TIM_PCLK_FREQ_HZ
#include "gpio.h" // Para configurar pin PB4

void tim3_ch1_pwm_init(uint32_t pwm_freq_hz)
{
    // 1. Configurar PA6 como Alternate Function (AF2) para TIM3_CH1
    gpio_setup_pin(GPIOA, 6, GPIO_MODE_AF, 2);

    // 2. Habilitar el reloj para TIM3
    rcc_tim3_clock_enable();

    // 3. Configurar TIM3
    TIM3->PSC = 100 - 1; // (4MHz / 100 = 40kHz)
    TIM3->ARR = (TIM_PCLK_FREQ_HZ / 100 / pwm_freq_hz) - 1; // 40kHz / pwm_freq_hz

    // Configurar el Canal 1 (CH1) en modo PWM 1
    TIM3->CCMR1 = (6U << 4);                    // PWM mode 1 on CH1
    TIM3->CCER  |= (1 << 0);                    // Enable CH1 output

    // Finalmente, habilitar el contador del timer (CEN bit en CR1)
    TIM3->CR1 |= 0x01 << 0;
}


void tim3_ch1_pwm_set_duty_cycle(uint8_t duty_cycle_percent)
{
    if (duty_cycle_percent > 100) {
        duty_cycle_percent = 100;
    }

    // Calcular el valor de CCR1 basado en el porcentaje y el valor de ARR guardado
    // CCR1 = ( (ARR + 1) * DutyCycle_Percent ) / 100
    // Cuidado con el orden de operaciones para evitar truncamiento prematuro.
    uint16_t tim3_ch1_arr_value = TIM3->ARR;
    uint32_t ccr_value = (((uint32_t)tim3_ch1_arr_value + 1U) * duty_cycle_percent) / 100U;

    TIM3->CCR1 = ccr_value;
}

```

### Integración:

* La función tim3_ch1_pwm_init() se llamará desde main(), pasándole la frecuencia PWM deseada (ej. 1000 Hz).

* La función tim3_ch1_pwm_set_duty_cycle() se usará desde room_control.c para establecer el brillo inicial del LED PWM y potencialmente para cambiarlo dinámicamente.

Siguiente módulo: [Nested Vectored Interrupt Controller (NVIC.md)](NVIC.md).

# Módulo TIM (Timers) - PWM con TIM3_CH1

Los temporizadores de propósito general (TIM) son periféricos muy versátiles que pueden usarse para medir tiempo, generar retardos, contar eventos y generar señales PWM (Pulse Width Modulation), entre otras cosas. Usaremos TIM3 para generar una señal PWM en su canal 1 (TIM3_CH1), que está disponible en el pin PB4.

**Archivos:** `Inc/tim.h`, `Src/tim.c`

**Pin:** PB4 (TIM3_CH1)

**Función Alternativa:** AF2 para TIM3_CH1 en PB4.

**Referencia Principal:** RM0351, Sección "27. General-purpose timers (TIM2/3/4/5)".

**Referencia de Taller Anterior:** `workshop_tim2_pwm_c.md` (adaptado para TIM3).

## Objetivos del Módulo TIM
1.  Definir la estructura `TIM_TypeDef` para el acceso a registros de un temporizador de propósito general.
2.  Configurar TIM3 en modo PWM para generar una señal en el pin PB4 (TIM3_CH1).
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
void tim3_ch1_pwm_init(uint32_t pwm_freq_hz); // Usa TIM_PCLK_FREQ_HZ de rcc.h
void tim3_ch1_pwm_set_duty_cycle(uint8_t duty_cycle_percent); // duty_cycle en % (0-100)

#endif // TIM_H

```

## 2. `Src/tim.c`

```c
#include "tim.h"
#include "gpio.h" // Para configurar pin PB4
#include "rcc.h"  // Para rcc_tim3_clock_enable y TIM_PCLK_FREQ_HZ

// Defines para los bits de los registros TIM (basados en workshop_tim2_pwm_c.md o RM0351)
// TIM_CR1 bits
#define TIM_CR1_CEN_Pos         0U  // Counter enable
#define TIM_CR1_CEN             (1UL << TIM_CR1_CEN_Pos)
#define TIM_CR1_ARPE_Pos        7U  // Auto-reload preload enable
#define TIM_CR1_ARPE            (1UL << TIM_CR1_ARPE_Pos)

// TIM_CCMR1 bits (para CH1, que usa los bits más bajos de CCMR1)
#define TIM_CCMR1_CC1S_Pos      0U  // Capture/Compare 1 selection
#define TIM_CCMR1_CC1S_Msk      (0x3UL << TIM_CCMR1_CC1S_Pos) // Bits 0-1
#define TIM_CCMR1_OC1PE_Pos     3U  // Output Compare 1 preload enable
#define TIM_CCMR1_OC1PE         (1UL << TIM_CCMR1_OC1PE_Pos)
#define TIM_CCMR1_OC1M_Pos      4U  // Output Compare 1 mode
#define TIM_CCMR1_OC1M_Msk      (0x7UL << TIM_CCMR1_OC1M_Pos) // Bits 4-6
#define TIM_CCMR1_OC1M_PWM1     (0x6UL << TIM_CCMR1_OC1M_Pos) // 0b110: PWM mode 1

// TIM_CCER bits (para CH1)
#define TIM_CCER_CC1E_Pos       0U  // Capture/Compare 1 output enable
#define TIM_CCER_CC1E           (1UL << TIM_CCER_CC1E_Pos)
#define TIM_CCER_CC1P_Pos       1U  // Capture/Compare 1 output polarity
#define TIM_CCER_CC1P           (1UL << TIM_CCER_CC1P_Pos) // 0: active high, 1: active low

// TIM_EGR bits
#define TIM_EGR_UG_Pos          0U  // Update generation
#define TIM_EGR_UG              (1UL << TIM_EGR_UG_Pos)


// Guardar el valor de ARR para cálculos de duty cycle si es necesario
static uint16_t tim3_ch1_arr_value = 0;

/**
 * @brief Inicializa TIM3 Canal 1 para generar una señal PWM.
 * @param pwm_freq_hz: Frecuencia deseada para la señal PWM en Hz.
 */
void tim3_ch1_pwm_init(uint32_t pwm_freq_hz) {
    // 1. Habilitar el reloj para TIM3 y GPIOB
    rcc_tim3_clock_enable();
    // rcc_gpio_clock_enable(GPIOB); // Se hace en gpio_pin_setup

    // 2. Configurar PB4 como Alternate Function (AF2) para TIM3_CH1
    //    PB4: AF, Push-Pull, Medium Speed (o la que se necesite para la frecuencia PWM)
    gpio_pin_setup(GPIOB, 4, GPIO_MODE_AF, GPIO_OTYPE_PUSHPULL, GPIO_OSPEED_MEDIUM, GPIO_PUPD_NONE, 2);

    // 3. Configurar TIM3
    //    El reloj de TIM3 es TIM_PCLK_FREQ_HZ (definido en rcc.h, usualmente PCLK1).
    //    Asumimos que TIM_PCLK_FREQ_HZ es 4MHz según talleres anteriores.

    // Calcular Prescaler (PSC) y Auto-Reload Register (ARR)
    // PWM_Freq = TIM_PCLK_FREQ_HZ / ((PSC + 1) * (ARR + 1))
    // Para una buena resolución de duty cycle, se suele querer un ARR grande (ej. 999 para 1000 pasos).
    // Si ARR = 999 (para 1000 niveles de duty cycle, 0-999):
    // PSC + 1 = TIM_PCLK_FREQ_HZ / (pwm_freq_hz * 1000)
    // PSC = (TIM_PCLK_FREQ_HZ / (pwm_freq_hz * 1000)) - 1
    // Ejemplo: TIM_PCLK = 4MHz, pwm_freq = 1kHz (1000 Hz)
    // PSC = (4000000 / (1000 * 1000)) - 1 = (4000000 / 1000000) - 1 = 4 - 1 = 3.
    // ARR = 99.
    tim3_ch1_arr_value = 99; // Para 100 pasos de resolución (1%)
    uint16_t psc_value = (TIM_PCLK_FREQ_HZ / (pwm_freq_hz * (tim3_ch1_arr_value + 1))) - 1;

    TIM3->PSC = psc_value;
    TIM3->ARR = tim3_ch1_arr_value;

    // Configurar el Canal 1 (CH1) en modo PWM 1
    // CCMR1 (Capture/Compare Mode Register 1) - OC1M bits para CH1
    // OC1M = 0b110 (PWM mode 1: activo mientras CNT < CCR1, inactivo después)
    // OC1PE = 1 (Output Compare 1 Preload Enable - CCR1 se actualiza en UEV)
    // CC1S = 00 (Channel is configured as output)
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk); // Limpiar CC1S y OC1M
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_PWM1;  // Establecer modo PWM 1
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;      // Habilitar preload para CCR1

    // Configurar polaridad y habilitar salida del Canal 1
    // CCER (Capture/Compare Enable Register) - CC1P y CC1E bits
    // CC1P = 0 (Polaridad: Activo Alto - por defecto, para que el LED encienda con nivel alto)
    // CC1E = 1 (Habilitar salida del canal)
    TIM3->CCER &= ~TIM_CCER_CC1P; // Activo Alto (polaridad no invertida)
    TIM3->CCER |= TIM_CCER_CC1E;  // Habilitar salida del Canal 1

    // Establecer el valor inicial del ciclo de trabajo (Duty Cycle) en CCR1
    // CCR1 = ((ARR + 1) * DutyCycle_Percent) / 100
    // Por ejemplo, 0% inicialmente.
    TIM3->CCR1 = 0;

    // Habilitar Auto-Reload Preload (ARPE bit en CR1)
    // Esto significa que el valor de ARR (y otros registros preload como CCRx si OCxPE=1)
    // se actualiza solo en un evento de actualización (UEV). Es buena práctica.
    TIM3->CR1 |= TIM_CR1_ARPE;

    // Opcional: Generar un evento de actualización para cargar los registros preload (PSC, ARR, CCRx)
    // en los registros activos.
    TIM3->EGR |= TIM_EGR_UG;

    // Finalmente, habilitar el contador del timer (CEN bit en CR1)
    TIM3->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Establece el ciclo de trabajo (duty cycle) para la señal PWM en TIM3 CH1.
 * @param duty_cycle_percent: Ciclo de trabajo en porcentaje (0 a 100).
 */
void tim3_ch1_pwm_set_duty_cycle(uint8_t duty_cycle_percent) {
    if (duty_cycle_percent > 100) {
        duty_cycle_percent = 100;
    }

    // Calcular el valor de CCR1 basado en el porcentaje y el valor de ARR guardado
    // CCR1 = ( (ARR + 1) * DutyCycle_Percent ) / 100
    // Cuidado con el orden de operaciones para evitar truncamiento prematuro.
    uint32_t ccr_value = (((uint32_t)tim3_ch1_arr_value + 1U) * duty_cycle_percent) / 100U;

    TIM3->CCR1 = ccr_value;
}

```

### Integración:

* La función tim3_ch1_pwm_init() se llamará desde main(), pasándole la frecuencia PWM deseada (ej. 1000 Hz).

* La función tim3_ch1_pwm_set_duty_cycle() se usará desde room_control.c para establecer el brillo inicial del LED PWM y potencialmente para cambiarlo dinámicamente.

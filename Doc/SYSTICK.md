# Módulo SysTick (System Timer)

SysTick es un temporizador simple de cuenta descendente de 24 bits integrado en el núcleo Cortex-M. Es comúnmente usado para generar una base de tiempo periódica (tick del sistema) para tareas como retardos, timeouts o una planificación simple de tareas.

**Archivos:** `Inc/systick.h`, `Src/systick.c`

**Referencia Principal:** PM0214 (Cortex-M4 Devices Generic User Guide), Sección 4.5 SysTick timer (STK).

## Objetivos del Módulo SysTick
1.  Definir la estructura `SysTick_TypeDef` para el acceso a registros.
2.  Configurar SysTick para generar una interrupción cada 1 milisegundo, asumiendo una frecuencia de reloj `SYSCLK_FREQ_HZ` (definida en `rcc.h` como 4MHz).
3.  Implementar la rutina de servicio de interrupción (`SysTick_Handler`) para la lógica que requiera ejecución periódica (ej. contadores de tiempo).
4.  Proporcionar funciones para obtener el valor actual de un contador de ticks y para generar retardos (bloqueantes).

## 1. `Inc/systick.h`

```c
#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

typedef struct {
    volatile uint32_t CTRL;  // Control and Status Register, Offset: 0x00
    volatile uint32_t LOAD;  // Reload Value Register,    Offset: 0x04
    volatile uint32_t VAL;   // Current Value Register,   Offset: 0x08
    volatile uint32_t CALIB; // Calibration Register,     Offset: 0x0C
} SysTick_TypeDef;


#define SYSTICK_BASE        (0xE000E010UL)
#define SysTick             ((SysTick_TypeDef *)SYSTICK_BASE)

// Prototipos de funciones
void systick_init_1ms(void);
uint32_t systick_get_tick(void);
void systick_delay_ms(uint32_t ms);

#endif // SYSTICK_H

```

## 2. `Src/systick.c`

```c
#include "systick.h"
#include "rcc.h" // Para SYSCLK_FREQ_HZ


// Variable global para contar los ticks de milisegundos
// 'volatile' es importante porque esta variable es modificada en una ISR
// y leída en el flujo principal del programa.
static volatile uint32_t g_systick_ms_count = 0;


void systick_init_1ms(void)
{
    // 1. Calcular el valor de recarga para 1 ms
    uint32_t reload_value = (SYSCLK_FREQ_HZ / 1000U) - 1U; // (4000000 / 1000) - 1 = 3999

    // 2. Configurar el registro de recarga (SysTick_LOAD)
    SysTick->LOAD = reload_value & 0x00FFFFFFUL;

    // 3. Poner a cero el valor actual del temporizador (SysTick_VAL)
    SysTick->VAL = 0x00000000UL;

    // 4. Configurar el registro de control (SysTick_CTRL)
    SysTick->CTRL = (0x01 << 2) | // Usa reloj del procesador (HCLK)
                    (0x01 << 1) | // Habilita interrupción de SysTick
                    (0x01 << 0) ; // Habilita el contador SysTick
}

uint32_t systick_get_tick(void)
{
    return g_systick_ms_count;
}


void systick_delay_ms(uint32_t ms)
{
    uint32_t start_tick = systick_get_tick(); 
    while ((systick_get_tick() - start_tick) < ms) {
        // Espera hasta que hayan transcurridos los milisegundos deseados
    }
}


void SysTick_Handler(void)
{
    g_systick_ms_count++; // Incrementar el contador de ticks global
}

```

### Uso e Integración:

* En main.c, se llamará a systick_init_1ms().

* La variable g_systick_ms_count o la función systick_get_tick() se usará para gestionar temporizaciones a nivel global del programa.


Siguiente módulo: [General Purpose Input/Output (GPIO.md)](GPIO.md).

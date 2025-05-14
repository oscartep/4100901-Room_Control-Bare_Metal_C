# Lógica de Aplicación (`room_control`)

Este módulo es donde implementarás la lógica principal de tu "Sistema de Control Básico de una Sala". A diferencia de los módulos de drivers de periféricos (GPIO, SysTick, UART, TIM, NVIC), `room_control` no gestiona directamente los registros de hardware. En su lugar, **utiliza las funciones proporcionadas por los drivers** para interactuar con los periféricos y construir el comportamiento deseado.

**Archivos:** `Inc/room_control.h`, `Src/room_control.c`

## Objetivos del Módulo `room_control`
1.  Definir las variables de estado necesarias para la aplicación (ej. temporizadores, flags).
2.  Implementar las funciones que serán llamadas por las Rutinas de Servicio de Interrupción (ISRs) para reaccionar a eventos:
    *   Una función para manejar las tareas periódicas activadas por `SysTick_Handler`.
    *   Una función para manejar la pulsación del botón B1, activada por `EXTI15_10_IRQHandler`.
    *   Una función para manejar los datos recibidos por UART, activada por `USART2_IRQHandler`.
3.  Organizar la lógica para cumplir con los requisitos funcionales:
    *   Parpadeo del LED de "heartbeat" (LD2).
    *   Encendido temporal (3s) de un LED externo al presionar B1, con mensajes UART.
    *   Eco de caracteres UART.
    *   Control de un LED con PWM (brillo inicial y posible modificación).

## 1. `Inc/room_control.h`

```c
#ifndef ROOM_CONTROL_H
#define ROOM_CONTROL_H

#include <stdint.h>

/**
 * @brief Función a ser llamada por EXTI15_10_IRQHandler cuando se detecta
 *        la pulsación del botón B1.
 */
void room_control_on_button_press(void);

/**
 * @brief Función a ser llamada por USART2_IRQHandler cuando se recibe un carácter.
 * @param received_char El carácter recibido por UART.
 */
void room_control_on_uart_receive(char received_char);

/**
 * @brief (Opcional) Función para realizar inicializaciones específicas de la lógica
 *        de room_control, si las hubiera (ej. inicializar variables de estado).
 *        Las inicializaciones de periféricos se harán en main().
 */
void room_control_app_init(void);

#endif // ROOM_CONTROL_H

```

```c
#include "room_control.h"

#include "gpio.h"    // Para controlar LEDs y leer el botón (aunque el botón es por EXTI)
#include "systick.h" // Para obtener ticks y manejar retardos/tiempos
#include "uart.h"    // Para enviar mensajes
#include "tim.h"     // Para controlar el PWM

void room_control_app_init(void)
{
    // Inicializar variables de estado si es necesario.
    // Por ejemplo, asegurar que los LEDs estén apagados al inicio

    // tim3_ch1_pwm_set_duty_cycle(50); // Establecer un duty cycle inicial para el PWM LED
}

void room_control_on_button_press(void)
{
    // TODO: Implementar anti-rebote
    // TODO: Procesar la presion para realizar acciones
    uart2_send_string("Boton B1: Presionado.\r\n");
}

void room_control_on_uart_receive(char received_char)
{
    // TODO: Procesar el carácter para realizar acciones
    // Ejemplo: si recibe 'h' o 'H', encender el LED PWM al 100%.
    //          si recibe 'l' o 'L', apagar el LED PWM (0%).
    //          si recibe 't', hacer toggle del LED ON/OFF.
}

```

### Pistas para los Estudiantes:

* Variables de Estado: Piensen qué información necesita recordar su aplicación entre llamadas a las funciones handler (ej. cuándo se encendió un LED para saber cuándo apagarlo, el último estado de un toggle). Declárenlas como static dentro de room_control.c para que no sean visibles globalmente, y volatile si son modificadas por una ISR y leídas en otra parte (o viceversa).

* Anti-Rebote (Debounce): Para el botón, es crucial implementar un mecanismo anti-rebote. Una forma simple es registrar el systick_get_tick() de la última pulsación válida y ignorar nuevas pulsaciones si ocurren demasiado pronto (ej. dentro de 50-200 ms).

* Conexión con ISRs: Las ISRs definidas en los módulos de drivers (systick.c, nvic.c, uart.c) deben ser modificadas para llamar a las funciones room_control_on_... correspondientes. Esto se hace incluyendo room_control.h en esos archivos .c y añadiendo la llamada.


Siguiente módulo: [Main (MAIN.md)](MAIN.md).
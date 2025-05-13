# Lógica de Control (`room_control`)

Este módulo es el corazón de la aplicación. Integra la funcionalidad de los diferentes periféricos (GPIO, SysTick, NVIC/EXTI, UART, TIM) para implementar el comportamiento deseado del "Sistema de Control Básico", replicando la funcionalidad del taller introductorio pero en C puro.

**Archivos:** `Inc/room_control.h`, `Src/room_control.c`

## Objetivos del Módulo `room_control`
1.  Proporcionar una función de inicialización `room_control_init()` que configure todos los periféricos necesarios llamando a las funciones de inicialización de los módulos correspondientes.
2.  Manejar el parpadeo del LED de "heartbeat" (LD2) usando el tick de SysTick.
3.  Gestionar la lógica del botón B1 (PC13):
    *   Al detectarse una pulsación (manejada por `EXTI15_10_IRQHandler`), ejecutar una acción.
    *   La acción incluye: encender el LED externo ON/OFF, enviar un mensaje por UART, e iniciar un temporizador de 3 segundos para apagar dicho LED.
4.  Manejar el timeout para apagar el LED externo ON/OFF (usando el tick de SysTick) y enviar el mensaje UART correspondiente.
5.  Implementar el eco de caracteres recibidos por UART (manejado por `USART2_IRQHandler`).
6.  Controlar el LED PWM con un ciclo de trabajo inicial y permitir su modificación (opcionalmente vía UART).

## 1. `Inc/room_control.h`

```c
#ifndef ROOM_CONTROL_H
#define ROOM_CONTROL_H

#include <stdint.h>
// No es necesario incluir los headers de periféricos aquí si room_control.c los incluye.

// Definiciones de pines y constantes de la aplicación
// (Se podrían mover a un archivo de configuración global si el proyecto crece)

// LD2 (Heartbeat LED en PA5)
#define LD2_GPIO_PORT           GPIOA
#define LD2_PIN                 5

// Botón B1 (Usuario en PC13)
#define USER_BUTTON_GPIO_PORT   GPIOC
#define USER_BUTTON_PIN         13

// LED Externo ON/OFF (ej. PA4)
#define LED_EXT_ONOFF_GPIO_PORT GPIOA
#define LED_EXT_ONOFF_PIN       4
#define LED_EXT_ONOFF_DURATION_MS 3000U // 3 segundos

// LED Externo PWM (PB4 - TIM3_CH1)
#define LED_EXT_PWM_GPIO_PORT   GPIOB
#define LED_EXT_PWM_PIN         4
#define PWM_FREQUENCY_HZ        1000U // 1 kHz
#define PWM_INITIAL_DUTY        50    // 50%

// Intervalos y temporizadores
#define HEARTBEAT_INTERVAL_MS   500U
#define BUTTON_DEBOUNCE_MS      200U


// Prototipos de funciones públicas del módulo
void room_control_init(void);
void room_control_systick_periodic_handler(void); // Llamada desde SysTick_Handler
void room_control_button_pressed_handler(void);   // Llamada desde EXTI15_10_IRQHandler
void room_control_uart_receive_handler(char data); // Llamada desde USART2_IRQHandler

#endif // ROOM_CONTROL_H

```

## 2. `Src/room_control.c`

```c
#include "room_control.h" // Define constantes y prototipos
#include "rcc.h"
#include "gpio.h"
#include "systick.h"
#include "nvic.h"    // Para configurar EXTI y USART2 IRQs
#include "uart.h"
#include "tim.h"
#include <stdio.h>   // Para sprintf (opcional, si se envían números formateados)
#include <string.h>  // Para strlen (opcional, si se usa para longitudes de string)

// Variables de estado del módulo (static para encapsulación)
static volatile uint32_t led_onoff_active_until_tick = 0; // Tick en el que el LED ON/OFF debe apagarse
static volatile uint32_t last_button_press_tick = 0;    // Para anti-rebote del botón

// Buffer para mensajes UART
static char uart_message_buffer[80];

/**
 * @brief Inicializa todos los subsistemas y la lógica de control.
 */
void room_control_init(void) {
    // 1. Inicialización básica del sistema de reloj (RCC)
    //    (Asumimos que rcc_system_basic_init() o similar se llamó en main o aquí si es necesario)
    //    Ya no se llama a rcc_system_basic_init() aquí, se asume que main lo hizo.

    // 2. Inicializar SysTick para generar interrupciones cada 1ms
    systick_init_1ms(); // Usa SYSCLK_FREQ_HZ de rcc.h

    // 3. Configurar GPIOs
    // LD2 (PA5) para heartbeat
    gpio_pin_setup(LD2_GPIO_PORT, LD2_PIN, GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSHPULL, GPIO_OSPEED_LOW, GPIO_PUPD_NONE, 0);
    // LED Externo ON/OFF (PA4)
    gpio_pin_setup(LED_EXT_ONOFF_GPIO_PORT, LED_EXT_ONOFF_PIN, GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSHPULL, GPIO_OSPEED_LOW, GPIO_PUPD_NONE, 0);
    gpio_write_pin(LED_EXT_ONOFF_GPIO_PORT, LED_EXT_ONOFF_PIN, GPIO_PIN_RESET); // Apagado inicialmente

    // 4. Configurar interrupción EXTI para el botón B1 (PC13)
    //    Esto incluye configurar PC13 como entrada, SYSCFG, EXTI y habilitar en NVIC.
    nvic_exti_pc13_button_config();

    // 5. Inicializar UART2 para comunicación serial
    //    Esto configura PA2/PA3 como AF, baudrate, TE/RE, RXNEIE.
    //    También llama a nvic_usart2_irq_config() internamente o debe ser llamado.
    uart2_init(); // Asume que nvic_usart2_irq_config() es llamada por uart2_init o en main

    // Enviar mensaje de bienvenida
    uart2_send_string("Sistema de Control Basico C-Puro - v1.0\r\n");
    uart2_send_string("Listo.\r\n");

    // 6. Inicializar TIM3 Canal 1 para PWM en PB4
    //    Esto configura PB4 como AF, y TIM3 para PWM.
    tim3_ch1_pwm_init(PWM_FREQUENCY_HZ);
    tim3_ch1_pwm_set_duty_cycle(PWM_INITIAL_DUTY);
}

/**
 * @brief Manejador periódico llamado desde SysTick_Handler cada 1ms.
 *        Gestiona el parpadeo del LED heartbeat y el timeout del LED ON/OFF.
 */
void room_control_systick_periodic_handler(void) {
    static uint32_t heartbeat_counter_ms = 0;
    uint32_t current_tick = systick_get_tick(); // Obtener el tick actual

    // Manejo del Heartbeat LED (LD2)
    heartbeat_counter_ms++;
    if (heartbeat_counter_ms >= HEARTBEAT_INTERVAL_MS) {
        gpio_toggle_pin(LD2_GPIO_PORT, LD2_PIN);
        heartbeat_counter_ms = 0;
    }

    // Manejo del timeout del LED Externo ON/OFF
    if (led_onoff_active_until_tick > 0) { // Si hay un timeout programado
        if (current_tick >= led_onoff_active_until_tick) {
            gpio_write_pin(LED_EXT_ONOFF_GPIO_PORT, LED_EXT_ONOFF_PIN, GPIO_PIN_RESET);
            led_onoff_active_until_tick = 0; // Desactivar el timeout

            uart2_send_string("LED_EXT_ONOFF: Apagado (Timeout).\r\n");
        }
    }
}

/**
 * @brief Manejador llamado desde EXTI15_10_IRQHandler cuando se presiona el botón B1.
 */
void room_control_button_pressed_handler(void) {
    uint32_t current_tick = systick_get_tick();

    // Anti-rebote simple basado en tiempo
    if ((current_tick - last_button_press_tick) < BUTTON_DEBOUNCE_MS) {
        return; // Rebote, ignorar
    }
    last_button_press_tick = current_tick; // Actualizar el tiempo de la última pulsación válida

    // Acción al presionar el botón
    gpio_write_pin(LED_EXT_ONOFF_GPIO_PORT, LED_EXT_ONOFF_PIN, GPIO_PIN_SET);
    // Programar el apagado del LED
    led_onoff_active_until_tick = current_tick + LED_EXT_ONOFF_DURATION_MS;

    uart2_send_string("Boton B1: Presionado. LED_EXT_ONOFF: Encendido.\r\n");

    // Opcional: Cambiar el duty cycle del PWM como ejemplo
    static uint8_t pwm_duty_toggle = 0;
    if (pwm_duty_toggle == 0) {
        tim3_ch1_pwm_set_duty_cycle(25);
        pwm_duty_toggle = 1;
    } else {
        tim3_ch1_pwm_set_duty_cycle(75);
        pwm_duty_toggle = 0;
    }
    sprintf(uart_message_buffer, "PWM Duty: %d%%\r\n", (pwm_duty_toggle == 1) ? 25 : 75);
    uart2_send_string(uart_message_buffer);
}

/**
 * @brief Manejador llamado desde USART2_IRQHandler cuando se recibe un carácter.
 * @param data El carácter recibido por UART.
 */
void room_control_uart_receive_handler(char data) {
    // Eco simple del carácter recibido
    uart2_send_char(data);
    // uart2_send_string("\r\n"); // Opcional: nueva línea después del eco

    // Aquí se podría añadir lógica más compleja para procesar comandos recibidos.
    // Por ejemplo, si se recibe '0'-'9', cambiar el duty cycle del PWM.
    if (data >= '0' && data <= '9') {
        uint8_t new_duty = (data - '0') * 10; // '0'->0%, '1'->10%, ..., '9'->90%
        if (new_duty == 0 && data == '0') new_duty = 0; // Asegurar que '0' sea 0%
        else if (new_duty == 0 && data != '0') new_duty = 100; // Si '0' es por (data-'0')*10=0, pero data no es '0', es un error.
                                                               // Esto es solo un ejemplo rápido. Mejorar parseo.
        
        if (data == '0') new_duty = 0;
        else if (data == '1') new_duty = 10;
        // ...
        else if (data == '9') new_duty = 90;
        // else if (data == 'A' || data == 'a') new_duty = 100; // Para 100%

        tim3_ch1_pwm_set_duty_cycle(new_duty);
        sprintf(uart_message_buffer, "PWM Duty via UART: %d%%\r\n", new_duty);
        uart2_send_string(uart_message_buffer);
    } else if (data == 'L' || data == 'l') {
        gpio_toggle_pin(LED_EXT_ONOFF_GPIO_PORT, LED_EXT_ONOFF_PIN);
        uart2_send_string("LED_EXT_ONOFF Toggled via UART.\r\n");
    }
}

```


### Integración y Flujo:

* main() llama a room_control_init(): Esto configura todos los periféricos y estados iniciales.

* SysTick_Handler() llama a room_control_systick_periodic_handler(): Cada milisegundo, se actualiza la lógica del heartbeat y se verifica si el LED ON/OFF debe apagarse.

* EXTI15_10_IRQHandler() llama a room_control_button_pressed_handler(): Cuando se presiona el botón B1, se ejecutan las acciones correspondientes (encender LED, enviar UART, programar timeout).

* USART2_IRQHandler() llama a room_control_uart_receive_handler(): Cuando llega un carácter por UART, se hace eco y se puede procesar para comandos.

* El bucle principal en main.c puede quedar vacío o con __WFI(), ya que toda la lógica es reactiva (basada en interrupciones) o periódica (SysTick).


Este módulo room_control actúa como el "director de orquesta" para todos los demás módulos de periféricos.
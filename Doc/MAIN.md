# Función Principal (`main.c`)

El archivo `main.c` contiene el punto de entrada de la aplicación (`main` function) después de que el código de arranque (`startup_stm32l476rgtx.s`) haya realizado las inicializaciones básicas del microcontrolador.

En este taller, `main()` será responsable de:
1.  Realizar la configuración inicial del sistema de reloj (RCC).
2.  Inicializar cada uno de los periféricos que se utilizarán (GPIOs, SysTick, NVIC para interrupciones EXTI y USART, USART2, TIM3 para PWM) llamando directamente a las funciones de inicialización de los respectivos módulos de drivers.
3.  Llamar a una función de inicialización de la lógica de la aplicación (`room_control_app_init()`), si es necesaria para establecer estados iniciales de la aplicación.
4.  Entrar en un bucle infinito donde el sistema esperará interrupciones para realizar su trabajo.

**Archivo:** `Src/main.c`

## Objetivos de `main.c`
1.  Servir como el punto de configuración centralizado para todos los periféricos del sistema.
2.  Iniciar la lógica de la aplicación.
3.  Mantener el procesador en un estado de espera eficiente (bajo consumo) una vez que todo está configurado.

## Estructura de `Src/main.c` (Esqueleto Sugerido)

```c
#include "gpio.h"
#include "systick.h"
#include "nvic.h"
#include "uart.h"
#include "tim.h"
#include "room_control.h"

void heartbeat_led_toggle(void)
{
    static uint32_t last_tick = 0;
    if (systick_get_tick() - last_tick >= 500) { // Cambia cada 500 ms
        gpio_toggle_pin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
        last_tick = systick_get_tick();
    }
}


int main(void)
{
    // Inicialización de SysTick
    systick_init_1ms(); // Utiliza SYSCLK_FREQ_HZ (ej. 4MHz) de rcc.h

    // LED Heartbeat
    gpio_setup_pin(GPIOA, HEARTBEAT_LED_PIN, GPIO_MODE_OUTPUT, 0);

    // LED Externo ON/OFF
    gpio_setup_pin(GPIOA, EXTERNAL_LED_ONOFF_PIN, GPIO_MODE_OUTPUT, 0);

    // Botón B1
    gpio_setup_pin(GPIOC, 13, GPIO_MODE_INPUT, 0);
    nvic_exti_pc13_button_enable();

    // USART2
    uart2_init(115200);
    nvic_usart2_irq_enable();

    // TIM3 Canal 1 para PWM
    tim3_ch1_pwm_init(1000); // ej. 1000 Hz
    tim3_ch1_pwm_set_duty_cycle(70); // ej. 50%

    // Inicialización de la Lógica de la Aplicación (room_control)
    room_control_app_init();

    // Mensaje de bienvenida o estado inicial (puede estar en room_control_app_init o aquí)
    uart2_send_string("\r\nSistema Inicializado. Esperando eventos...\r\n");
    while (1) {
        heartbeat_led_toggle();
    }
}

```
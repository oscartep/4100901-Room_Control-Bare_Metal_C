/**
 ******************************************************************************
 * @file           : room_control.c
 * @author         : sam C
 * @brief          : Room control driver for STM32L476RGTx
 ******************************************************************************
 */
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

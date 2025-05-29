/**
 ******************************************************************************
 * @file           : room_control.c
 * @author         : Sam C
 * @brief          : Room control driver for STM32L476RGTx
 ******************************************************************************
 */
#include "room_control.h"
#include "gpio.h"
#include "systick.h"
#include "uart.h"
#include "tim.h"

// Variables de control del LED
static uint8_t led_on = 0;
static uint32_t led_start_time = 0;

void room_control_app_init(void)
{
    gpio_write_pin(EXTERNAL_LED_ONOFF_PORT, EXTERNAL_LED_ONOFF_PIN, GPIO_PIN_RESET);
    tim3_ch1_pwm_set_duty_cycle(70); 
}

void room_control_on_button_press(void)
{
    uart2_send_string("Boton presionado.\r\n");

    gpio_write_pin(EXTERNAL_LED_ONOFF_PORT, EXTERNAL_LED_ONOFF_PIN, GPIO_PIN_SET);
    led_on = 1;
    led_start_time = systick_get_tick();
}

void room_control_update(void)
{
    if (led_on && (systick_get_tick() - led_start_time >= 3000)) {
        gpio_write_pin(EXTERNAL_LED_ONOFF_PORT, EXTERNAL_LED_ONOFF_PIN, GPIO_PIN_RESET);
        led_on = 0;
        uart2_send_string("LED apagado por tres segundos.\r\n");
    }
}

void room_control_on_uart_receive(char received_char)
{
    switch (received_char) {
        case 'h':
        case 'H':
            tim3_ch1_pwm_set_duty_cycle(100);
            uart2_send_string("LED al 100%\r\n");
            break;
        case 'l':
        case 'L':
            tim3_ch1_pwm_set_duty_cycle(0);
            uart2_send_string("LED al 0%\r\n");
            break;
        case 't':
        case 'T':
            gpio_toggle_pin(EXTERNAL_LED_ONOFF_PORT, EXTERNAL_LED_ONOFF_PIN);
            uart2_send_string("LED ON/OFF\r\n");
            break;
        default:
            uart2_send_string("Comando no reconocido\r\n");
            break;
    }
}

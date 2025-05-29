#include "gpio.h"
#include "systick.h"
#include "nvic.h"
#include "uart.h"
#include "tim.h"
#include "room_control.h"

void heartbeat_led_toggle(void)
{
    static uint32_t last_tick = 0;
    if (systick_get_tick() - last_tick >= 500) {
        gpio_toggle_pin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
        last_tick = systick_get_tick();
    }
}

int main(void)
{
    systick_init_1ms();

    gpio_setup_pin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, GPIO_MODE_OUTPUT, 0);
    gpio_setup_pin(EXTERNAL_LED_ONOFF_PORT, EXTERNAL_LED_ONOFF_PIN, GPIO_MODE_OUTPUT, 0);

    gpio_setup_pin(USER_BUTTON_PORT, USER_BUTTON_PIN, GPIO_MODE_INPUT, 0);
    nvic_exti_pc13_button_enable();

    uart2_init(115200);
    nvic_usart2_irq_enable();

    tim3_ch1_pwm_init(1000);
    tim3_ch1_pwm_set_duty_cycle(70);

    room_control_app_init();

    uart2_send_string("\r\n Inicio del programa
                            Elije la accion a realizar
                            h: LED 100%
                            l: LED 0%
                            t: LED ON/OF
                            Presiona el boton para enceder el led pro 3 seg \r\n");

    while (1) {
        heartbeat_led_toggle();
        room_control_update(); 
    }
}

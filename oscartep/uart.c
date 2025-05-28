#include "uart.h"
#include "rcc.h"
#include "gpio.h"
#include "room_control.h"

#define USART_ISR_TXE_Pos       7U
#define USART_ISR_TXE           (1UL << USART_ISR_TXE_Pos)
#define USART_ISR_RXNE_Pos      5U
#define USART_ISR_RXNE          (1UL << USART_ISR_RXNE_Pos)

void uart2_init(uint32_t baud_rate)
{
    gpio_setup_pin(GPIOA, 2, GPIO_MODE_AF, 7);
    gpio_setup_pin(GPIOA, 3, GPIO_MODE_AF, 7);
    rcc_usart2_clock_enable();

    USART2->CR1 &= ~(0x01 << 0);
    uint32_t usart_div = (PCLK1_FREQ_HZ + (baud_rate / 2U)) / baud_rate;
    USART2->BRR = usart_div;
    USART2->CR1 |= (0x01 << 2 | 0x01 << 3);
    USART2->CR1 |= 0x01 << 0;
}

void uart2_send_char(char c)
{
    while (!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = (uint8_t)c;
}

void uart2_send_string(const char *str)
{
    while (*str != '\0') {
        uart2_send_char(*str++);
    }
}

void USART2_IRQHandler(void)
{
    if (USART2->ISR & USART_ISR_RXNE) {
        char received_char = (char)(USART2->RDR & 0xFF);
        room_control_on_uart_receive(received_char);
    }
}

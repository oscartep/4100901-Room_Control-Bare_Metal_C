/**
 ******************************************************************************
 * @file           : uart.h
 * @author         : Sam C
 * @brief          : Header file for UART driver for STM32L476RGTx
 ******************************************************************************
 */
#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "rcc.h"  // Para PCLK1_FREQ_HZ

// Estructura para los registros de USART
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t BRR;
    volatile uint32_t GTPR;
    volatile uint32_t RTOR;
    volatile uint32_t RQR;
    volatile uint32_t ISR;
    volatile uint32_t ICR;
    volatile uint32_t RDR;
    volatile uint32_t TDR;
} USART_TypeDef;

#define USART2_BASE         (0x40004400UL)
#define USART2              ((USART_TypeDef *) USART2_BASE)


void uart2_init(uint32_t baud_rate);

void uart2_send_char(char c);
void uart2_send_string(const char *str);

#endif // UART_H

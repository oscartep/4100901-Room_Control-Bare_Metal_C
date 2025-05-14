# Módulo UART (Universal Asynchronous Receiver/Transmitter)

El periférico USART/UART permite la comunicación serial asíncrona, comúnmente utilizada para la comunicación con un PC (terminal serial) u otros dispositivos. Usaremos USART2, que está conectado al ST-Link y se presenta como un puerto COM virtual en el PC.

**Archivos:** `Inc/uart.h`, `Src/uart.c`

**Pines:** PA2 (USART2_TX), PA3 (USART2_RX)

**Función Alternativa:** AF7 para USART2 en estos pines.

**Referencia Principal:** RM0351, Sección "40. Universal synchronous asynchronous receiver transmitter (USART)".

## Objetivos del Módulo UART
1.  Definir la estructura `USART_TypeDef` para el acceso a registros.
2.  Configurar USART2 para comunicación a 115200 baudios, 8N1 (8 bits de datos, sin paridad, 1 bit de parada), utilizando `PCLK1_FREQ_HZ` (definido en `rcc.h`).
3.  Habilitar la interrupción de recepción (RXNEIE) en el periférico USART2.
4.  Implementar funciones para enviar un carácter y una cadena de caracteres (polling).
5.  Implementar la rutina de servicio de interrupción (`USART2_IRQHandler`) para manejar los datos recibidos.

## 1. `Inc/uart.h`

```c
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

// Prototipos de funciones
void uart2_init(uint32_t baud_rate);

void uart2_send_char(char c);
void uart2_send_string(const char *str);

#endif // UART_H

```

## 2. `Src/uart.c`

```c
#include "uart.h"
#include "rcc.h"  // Para rcc_usart2_clock_enable y PCLK1_FREQ_HZ
#include "gpio.h" // Para configurar pines PA2, PA3


// USART_ISR bits
#define USART_ISR_TXE_Pos       7U  // Transmit Data Register Empty
#define USART_ISR_TXE           (1UL << USART_ISR_TXE_Pos)
#define USART_ISR_RXNE_Pos      5U  // Read Data Register Not Empty
#define USART_ISR_RXNE          (1UL << USART_ISR_RXNE_Pos)

void uart2_init(uint32_t baud_rate)
{
    // 1. Configurar pines PA2 (TX) y PA3 (RX) como Alternate Function (AF7)
    gpio_setup_pin(GPIOA, 2, GPIO_MODE_AF, 7);
    gpio_setup_pin(GPIOA, 3, GPIO_MODE_AF, 7);

    // 2. Habilitar el reloj para USART2
    rcc_usart2_clock_enable();

    // 3. Configurar USART2
    //    Deshabilitar USART antes de configurar (importante si se reconfigura)
    USART2->CR1 &= ~(0x01 << 0);

    // Configurar Baud Rate (USARTDIV en BRR)
    // USARTDIV = fCK_USART / BaudRate
    uint32_t usart_div = (PCLK1_FREQ_HZ + (baud_rate / 2U)) / baud_rate; // Con redondeo
    USART2->BRR = usart_div;

    // Habilitar Transmisor (TE) y Receptor (RE)
    USART2->CR1 |= (0x01 << 2 | 0x01 << 3);

    // Finalmente, habilitar USART (UE bit en CR1)
    USART2->CR1 |= 0x01 << 0;
}

void uart2_send_char(char c)
{
    // Esperar hasta que el buffer de transmisión (TDR) esté vacío.
    // Se verifica el flag TXE (Transmit data register empty) en el registro ISR.
    while (!(USART2->ISR & USART_ISR_TXE));

    // Escribir el dato en el Transmit Data Register (TDR).
    USART2->TDR = (uint8_t)c;
}

void uart2_send_string(const char *str)
{
    while (*str != '\0') {
        uart2_send_char(*str);
        str++;
    }
}

void USART2_IRQHandler(void)
{
    // Verificar si la interrupción fue por RXNE (dato recibido y RDR no vacío)
    if (USART2->ISR & USART_ISR_RXNE) {
        // Leer el dato del RDR. Esta acción usualmente limpia el flag RXNE.
        char received_char = (char)(USART2->RDR & 0xFF);
        uart2_send_char(received_char); // Eco del carácter recibido 
        // Procesar el carácter recibido.
    }
}

```

### Integración:

* La función uart2_init() se llamará desde main().

* La función nvic_usart2_irq_config() (de nvic.c) también se llamará desde main() o dentro de uart2_init() antes de habilitar el USART_CR1_UE.

Siguiente módulo: [General Purpose Timer (TIM.md)](TIM.md).
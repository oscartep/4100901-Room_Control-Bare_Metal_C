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
    volatile uint32_t ISR;
    volatile uint32_t ICR;
    volatile uint32_t RDR;
    volatile uint32_t TDR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_TypeDef;

#define USART2_BASE         (0x40004400UL)
#define USART2              ((USART_TypeDef *) USART2_BASE)

// Prototipos de funciones
void uart2_init(void); // Asume PCLK1_FREQ_HZ y baudrate de 115200
void uart2_send_char(char c);
void uart2_send_string(const char *str);

#endif // UART_H

```

## 2. `Src/uart.c`

```c
#include "uart.h"
#include "gpio.h" // Para configurar pines PA2, PA3
#include "rcc.h"  // Para rcc_usart2_clock_enable y PCLK1_FREQ_HZ
#include "nvic.h" // Para nvic_usart2_irq_config y la declaración de USART2_IRQn

// Defines para los bits de los registros USART (basados en workshop_uart.c o RM0351)
// USART_CR1 bits
#define USART_CR1_UE_Pos        0U  // USART Enable
#define USART_CR1_UE            (1UL << USART_CR1_UE_Pos)
#define USART_CR1_RE_Pos        2U  // Receiver Enable
#define USART_CR1_RE            (1UL << USART_CR1_RE_Pos)
#define USART_CR1_TE_Pos        3U  // Transmitter Enable
#define USART_CR1_TE            (1UL << USART_CR1_TE_Pos)
#define USART_CR1_RXNEIE_Pos    5U  // RXNE Interrupt Enable
#define USART_CR1_RXNEIE        (1UL << USART_CR1_RXNEIE_Pos)
// (Se pueden añadir M0, M1, PCE, PS si se necesitan configurar)

// USART_ISR bits
#define USART_ISR_TXE_Pos       7U  // Transmit Data Register Empty
#define USART_ISR_TXE           (1UL << USART_ISR_TXE_Pos)
#define USART_ISR_RXNE_Pos      5U  // Read Data Register Not Empty
#define USART_ISR_RXNE          (1UL << USART_ISR_RXNE_Pos)

// USART_ICR bits (Interrupt Clear Register)
#define USART_ICR_ORECF_Pos     3U  // Overrun Error Clear Flag (si se maneja error)
#define USART_ICR_ORECF         (1UL << USART_ICR_ORECF_Pos)


/**
 * @brief Inicializa USART2 para comunicación serial.
 *        Configuración: 115200 baudios, 8N1.
 *        Habilita la interrupción de recepción.
 */
void uart2_init(void) {
    uint32_t baud_rate = 115200U;

    // 1. Habilitar el reloj para USART2 y GPIOA
    rcc_usart2_clock_enable();
    // rcc_gpio_clock_enable(GPIOA); // Esto se hace en gpio_pin_setup

    // 2. Configurar pines PA2 (TX) y PA3 (RX) como Alternate Function (AF7)
    // PA2 (TX): AF, Push-Pull, High Speed
    gpio_pin_setup(GPIOA, 2, GPIO_MODE_AF, GPIO_OTYPE_PUSHPULL, GPIO_OSPEED_HIGH, GPIO_PUPD_NONE, 7);
    // PA3 (RX): AF, (Pull-up puede ser útil para evitar ruido si la línea flota)
    gpio_pin_setup(GPIOA, 3, GPIO_MODE_AF, GPIO_OTYPE_PUSHPULL, GPIO_OSPEED_HIGH, GPIO_PUPD_PULLUP, 7);

    // 3. Configurar USART2
    //    Deshabilitar USART antes de configurar (importante si se reconfigura)
    USART2->CR1 &= ~USART_CR1_UE;

    // Configurar longitud de palabra: 8 bits (M0=0, M1=0 en CR1). Por defecto es así.
    // Configurar paridad: Sin paridad (PCE=0 en CR1). Por defecto es así.
    // Configurar número de bits de parada: 1 bit (STOP[1:0]=00 en CR2). Por defecto.

    // Configurar Baud Rate (USARTDIV en BRR)
    // USARTDIV = fCK_USART / BaudRate
    // Para USART2, fCK_USART es PCLK1 (si PPRE1 en RCC_CFGR es /1, lo cual asumimos).
    // PCLK1_FREQ_HZ está definido en rcc.h (4MHz según talleres anteriores).
    uint32_t usart_div = (PCLK1_FREQ_HZ + (baud_rate / 2U)) / baud_rate; // Con redondeo
    USART2->BRR = usart_div; // Para 4MHz y 115200 baud, BRR = 34.72 -> 35 (0x23)
                            // (4000000 / 115200) = 34.72
                            // workshop_uart.c dice: (HSI_FREQ + (BAUD_RATE/2)) / BAUD_RATE
                            // (4000000 + 57600) / 115200 = 4057600 / 115200 = 35.22 -> 35 (0x23)

    // Habilitar Transmisor (TE) y Receptor (RE)
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    // Habilitar interrupción de recepción (RXNEIE - Read Data Register Not Empty Interrupt Enable)
    // Esto hará que se genere una interrupción cuando RDR tenga un dato.
    USART2->CR1 |= USART_CR1_RXNEIE;

    // Habilitar la interrupción de USART2 en el NVIC
    // Esta función se define en nvic.c
    nvic_usart2_irq_config();

    // Finalmente, habilitar USART (UE bit en CR1)
    USART2->CR1 |= USART_CR1_UE;
}

/**
 * @brief Envía un único carácter por USART2.
 * @param c: Carácter a enviar.
 */
void uart2_send_char(char c) {
    // Esperar hasta que el buffer de transmisión (TDR) esté vacío.
    // Se verifica el flag TXE (Transmit data register empty) en el registro ISR.
    while (!(USART2->ISR & USART_ISR_TXE));

    // Escribir el dato en el Transmit Data Register (TDR).
    USART2->TDR = (uint8_t)c;
}

/**
 * @brief Envía una cadena de caracteres (string) por USART2.
 * @param str: Puntero a la cadena de caracteres terminada en null.
 */
void uart2_send_string(const char *str) {
    while (*str != '\0') {
        uart2_send_char(*str);
        str++;
    }
}

/**
 * @brief Rutina de Servicio de Interrupción para USART2.
 *        Este nombre debe coincidir exactamente con el definido en la tabla de vectores
 *        del archivo de arranque (startup_stm32l476rgtx.s).
 *        Se llama cuando ocurre una interrupción de USART2 (ej. dato recibido).
 */
void USART2_IRQHandler(void) {
    // Verificar si la interrupción fue por RXNE (dato recibido y RDR no vacío)
    if (USART2->ISR & USART_ISR_RXNE) {
        // Leer el dato del RDR. Esta acción usualmente limpia el flag RXNE.
        // Si no lo limpia automáticamente en esta MCU, se debe limpiar
        // escribiendo en USART_ICR_RXNECF (si existe ese bit para RXNE) o
        // asegurándose de que la lectura de RDR sea suficiente.
        // Para STM32L4, la lectura de RDR limpia RXNE.
        char received_char = (char)(USART2->RDR & 0xFF);

        // Procesar el carácter recibido.
        // Esto se conectará con la lógica en room_control.c.
        // Ejemplo: room_control_uart_receive_action(received_char);
    }

    // Aquí se podrían manejar otros flags de interrupción de USART2 si se habilitaron
    // (ej. TXE para transmisión por interrupción, TC para transmisión completa, errores ORE, FE, NE).
    // if (USART2->ISR & USART_ISR_ORE) { // Overrun error
    //    USART2->ICR |= USART_ICR_ORECF; // Clear overrun flag
    // }
}

```

### Integración:

* La función uart2_init() se llamará desde main().

* La función nvic_usart2_irq_config() (de nvic.c) también se llamará desde main() o dentro de uart2_init() antes de habilitar el USART_CR1_UE.


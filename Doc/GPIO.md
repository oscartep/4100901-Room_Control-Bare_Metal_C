# Módulo GPIO (General-Purpose Input/Output)

El periférico GPIO permite configurar los pines del microcontrolador como entradas o salidas digitales, o para funciones alternativas (como UART, PWM, SPI, I2C).

**Archivos:** `Inc/gpio.h`, `Src/gpio.c`

**Referencia Principal:** RM0351, Sección "8. General-purpose I/Os (GPIO)".

## Objetivos del Módulo GPIO
1.  Proporcionar una estructura `GPIO_TypeDef` para el acceso a registros.
2.  Proporcionar funciones para configurar un pin como:
    *   Salida (Push-Pull).
    *   Entrada (con o sin Pull-up/Pull-down).
    *   Función Alternativa.
3.  Proporcionar funciones para escribir (set/reset/toggle) y leer el estado de un pin.

## 1. `Inc/gpio.h`

```c
#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFRL;
    volatile uint32_t AFRH;
} GPIO_TypeDef;


#define GPIOA_BASE (0x48000000U)
#define GPIOB_BASE (0x48000400U)
#define GPIOC_BASE (0x48000800U)

#define GPIOA ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC ((GPIO_TypeDef *)GPIOC_BASE)


// Modos de Pin (para GPIOx_MODER)
#define GPIO_MODE_INPUT     0x00U // 00: Input mode (reset state)
#define GPIO_MODE_OUTPUT    0x01U // 01: General purpose output mode
#define GPIO_MODE_AF        0x02U // 10: Alternate function mode
#define GPIO_MODE_ANALOG    0x03U // 11: Analog mode

// Tipos de Salida (para GPIOx_OTYPER)
#define GPIO_OTYPE_PUSHPULL  0x00U // 0: Output push-pull (reset state)
#define GPIO_OTYPE_OPENDRAIN 0x01U // 1: Output open-drain

// Velocidades de Salida (para GPIOx_OSPEEDR)
#define GPIO_OSPEED_LOW     0x00U // 00: Low speed
#define GPIO_OSPEED_MEDIUM  0x01U // 01: Medium speed
#define GPIO_OSPEED_HIGH    0x02U // 10: High speed
#define GPIO_OSPEED_VHIGH   0x03U // 11: Very high speed

// Pull-up/Pull-down (para GPIOx_PUPDR)
#define GPIO_PUPD_NONE      0x00U // 00: No pull-up, pull-down
#define GPIO_PUPD_PULLUP    0x01U // 01: Pull-up
#define GPIO_PUPD_PULLDOWN  0x02U // 10: Pull-down

// Estados de Pin
#define GPIO_PIN_RESET      0U
#define GPIO_PIN_SET        1U

// Prototipos de funciones
void gpio_pin_setup(GPIO_TypeDef *gpio_port,
                    uint8_t pin_number,
                    uint8_t mode,
                    uint8_t output_type,
                    uint8_t output_speed,
                    uint8_t pull_up_down,
                    uint8_t alternate_function);

void gpio_write_pin(GPIO_TypeDef *gpio_port, uint8_t pin_number, uint8_t pin_state);
void gpio_toggle_pin(GPIO_TypeDef *gpio_port, uint8_t pin_number);
uint8_t gpio_read_pin(GPIO_TypeDef *gpio_port, uint8_t pin_number);

#endif // GPIO_H

```

## 2. `Src/gpio.c`

```c
#include "gpio.h"
#include "rcc.h"
#include "nvic.h"

void gpio_pin_setup(GPIO_TypeDef *gpio_port,
                    uint8_t pin_number,
                    uint8_t mode,
                    uint8_t output_type,
                    uint8_t output_speed,
                    uint8_t pull_up_down,
                    uint8_t alternate_function)
{

    // 1. Habilitar el reloj para el puerto GPIO correspondiente
    //    La función rcc_gpio_clock_enable se encarga de esto.
    rcc_gpio_clock_enable(gpio_port);

    // 2. Configurar el modo del pin (Input, Output, AF, Analog)
    //    Cada pin usa 2 bits en MODER. Limpiar y luego establecer.
    gpio_port->MODER &= ~(0x03U << (pin_number * 2)); // Limpiar los 2 bits del pin
    gpio_port->MODER |= (mode << (pin_number * 2));   // Establecer el modo

    // 3. Configurar el tipo de salida (Push-Pull o Open-Drain)
    //    Solo si es modo Output o AF. Cada pin usa 1 bit en OTYPER.
    if (mode == GPIO_MODE_OUTPUT || mode == GPIO_MODE_AF) {
        gpio_port->OTYPER &= ~(0x01U << pin_number);         // Limpiar el bit
        gpio_port->OTYPER |= (output_type << pin_number); // Establecer el tipo
    }

    // 4. Configurar la velocidad de salida
    //    Solo si es modo Output o AF. Cada pin usa 2 bits en OSPEEDR.
    if (mode == GPIO_MODE_OUTPUT || mode == GPIO_MODE_AF) {
        gpio_port->OSPEEDR &= ~(0x03U << (pin_number * 2));    // Limpiar los 2 bits del pin
        gpio_port->OSPEEDR |= (output_speed << (pin_number * 2)); // Establecer la velocidad
    }

    // 5. Configurar Pull-up/Pull-down
    //    Cada pin usa 2 bits en PUPDR.
    gpio_port->PUPDR &= ~(0x03U << (pin_number * 2));       // Limpiar los 2 bits del pin
    gpio_port->PUPDR |= (pull_up_down << (pin_number * 2)); // Establecer pull-up/down

    // 6. Configurar la función alternativa
    //    Solo si es modo AF. Cada pin usa 4 bits.
    //    Pines 0-7 usan AFRL, pines 8-15 usan AFRH.
    if (mode == GPIO_MODE_AF) {
        uint32_t temp_af_val = alternate_function;
        if (pin_number < 8) { // AFRL
            gpio_port->AFRL &= ~(0x0FU << (pin_number * 4));          // Limpiar los 4 bits del pin
            gpio_port->AFRL |= (temp_af_val << (pin_number * 4)); // Establecer AF
        } else { // AFRH
            gpio_port->AFRH &= ~(0x0FU << ((pin_number - 8) * 4));       // Limpiar los 4 bits del pin
            gpio_port->AFRH |= (temp_af_val << ((pin_number - 8) * 4)); // Establecer AF
        }
    }
}

void gpio_write_pin(GPIO_TypeDef *gpio_port, uint8_t pin_number, uint8_t pin_state)
{
    if (pin_state == GPIO_PIN_SET) {
        // Establecer el bit correspondiente en BSRR (parte baja para SET)
        gpio_port->BSRR = (1U << pin_number);
    } else {
        gpio_port->BRR = (1U << pin_number);
    }
}

void gpio_toggle_pin(GPIO_TypeDef *gpio_port, uint8_t pin_number)
{
    gpio_port->ODR ^= (1U << pin_number);
}

uint8_t gpio_read_pin(GPIO_TypeDef *gpio_port, uint8_t pin_number)
{
    // Leer el bit correspondiente del Input Data Register (IDR)
    if ((gpio_port->IDR >> pin_number) & 0x01U) {
        return GPIO_PIN_SET;
    } else {
        return GPIO_PIN_RESET;
    }
}

/**
 * @brief Rutina de Servicio de Interrupción para EXTI líneas 10 a 15.
 *        Este nombre debe coincidir exactamente con el definido en la tabla de vectores
 *        del archivo de arranque (startup_stm32l476rgtx.s).
 *        Esta ISR puede ser llamada por room_control.c si la lógica es compleja.
 */
void EXTI15_10_IRQHandler(void) {
    // 1. Verificar si la interrupción fue de la línea EXTI13
    if ((EXTI->PR1 & (1U << 13)) != 0) {
        // 2. Limpiar el flag de pendiente de la interrupción (escribiendo '1')
        EXTI->PR1 |= (1U << 13);

        // 3. Ejecutar la acción correspondiente al botón presionado.
        //    Llamar a room_control_button_action();
    }
}


```

### Pines Específicos para el Taller y su Configuración:

* PA5 (LD2 - Heartbeat): 
```c
gpio_pin_setup(GPIOA, 5, GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSHPULL, GPIO_OSPEED_LOW, GPIO_PUPD_NONE, 0);
```

* PA4 (LED Externo ON/OFF):
```c
gpio_pin_setup(GPIOA, 4, GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSHPULL, GPIO_OSPEED_LOW, GPIO_PUPD_NONE, 0);
```

* PC13 (Botón B1 - para EXTI):
```c
gpio_pin_setup(GPIOC, 13, GPIO_MODE_INPUT, 0, 0, GPIO_PUPD_NONE, 0); // (El pull-up es externo en la Nucleo).
```

* PB4 (LED Externo PWM - TIM3_CH1):
```c
gpio_pin_setup(GPIOB, 4, GPIO_MODE_AF, GPIO_OTYPE_PUSHPULL, GPIO_OSPEED_MEDIUM, GPIO_PUPD_NONE, 2); (AF2 para TIM3 en PB4)
```

* PA2 (USART2_TX):
```c
gpio_pin_setup(GPIOA, 2, GPIO_MODE_AF, GPIO_OTYPE_PUSHPULL, GPIO_OSPEED_HIGH, GPIO_PUPD_NONE, 7); (AF7 para USART2 en PA2)
```

* PA3 (USART2_RX):
```c
gpio_pin_setup(GPIOA, 3, GPIO_MODE_AF, GPIO_OTYPE_PUSHPULL, GPIO_OSPEED_HIGH, GPIO_PUPD_PULLUP, 7);
```

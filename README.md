# Taller Práctico: Sistema de Control Básico en C Puro (STM32L476RG)

**Universidad Nacional de Colombia - Sede Manizales**
**Curso:** Estructuras Computacionales (4100901)

## Introducción

Este taller tiene como objetivo principal consolidar los conocimientos adquiridos en los capítulos previos sobre la arquitectura del STM32L476RG y el manejo de sus periféricos mediante programación en C puro, accediendo directamente a los registros de hardware.

Replicaremos la funcionalidad del "Sistema de Control Básico" implementado en la práctica introductoria (que utilizaba HAL y STM32CubeMX), pero esta vez, construiremos cada módulo desde cero, basándonos en las implementaciones realizadas en los talleres de SysTick, GPIO, EXTI, TIM/PWM y UART.

**Objetivo General:**
Implementar un sistema embebido que gestione LEDs, lea un botón, controle un LED con PWM y se comunique vía UART, utilizando únicamente acceso directo a registros en C puro.

**Funcionalidades a Implementar:**
1.  **Heartbeat LED:** Parpadeo del LED integrado (LD2) como señal de actividad del sistema.
2.  **Control de LED Externo por Botón:**
    *   Detectar la pulsación del botón de usuario (B1) mediante interrupción externa (EXTI).
    *   Encender un LED externo durante 3 segundos tras la pulsación.
3.  **Comunicación UART:**
    *   Enviar mensajes al PC para indicar eventos (pulsación de botón, timeout del LED).
    *   Implementar el procesamiento de los caracteres recibidos desde el PC como comandos.
4.  **Control PWM de LED Externo:**
    *   Generar una señal PWM utilizando TIM3_CH1 para controlar la intensidad de un segundo LED externo.

## Estructura de la Guía

Esta guía está dividida en varias secciones para facilitar el proceso de desarrollo:

1.  **[Configuración del Entorno (Doc/SETUP.md)](Doc/SETUP.md):** Preparación del hardware y el entorno de desarrollo.
2.  **[Módulo SysTick (Doc/SYSTICK.md)](Doc/SYSTICK.md):** Configuración del temporizador del sistema para tareas periódicas.
3.  **[Módulo RCC (Doc/RCC.md)](Doc/RCC.md):** Configuración del sistema de reloj.
4.  **[Módulo GPIO (Doc/GPIO.md)](Doc/GPIO.md):** Configuración y manejo de pines de entrada/salida.
5.  **[Módulo UART (Doc/UART.md)](Doc/UART.md):** Implementación de comunicación serial.
6.  **[Módulo TIM/PWM (Doc/TIM.md)](Doc/TIM.md):** Generación de señales PWM.
7.  **[Módulo EXTI (Doc/EXTI.md)](Doc/EXTI.md):** Configuración de interrupciones externas para el botón.
8.  **[Lógica de Control (Doc/ROOM_CONTROL.md)](Doc/ROOM_CONTROL.md):** Integración de la lógica de la aplicación.
9. **[Función Principal (Doc/MAIN.md)](Doc/MAIN.md):** Estructura del archivo `main.c`.

¡Manos a la obra! Comienza por revisar la [Configuración del Entorno (Doc/SETUP.md)](Doc/SETUP.md).
/**
 ******************************************************************************
 * @file           : nvic.h
 * @author         : Sam C
 * @brief          : Header file for NVIC driver for STM32L476RGTx
 ******************************************************************************
 */
#ifndef NVIC_H
#define NVIC_H

#include <stdint.h>

typedef struct {
    volatile uint32_t MEMRMP;
    volatile uint32_t CFGR1;
    volatile uint32_t EXTICR[4];
    volatile uint32_t SCSR;
    volatile uint32_t CFGR2;
    volatile uint32_t SWPR;
    volatile uint32_t SKR;
} SYSCFG_TypeDef;

typedef struct {
    volatile uint32_t IMR1;
    volatile uint32_t EMR1;
    volatile uint32_t RTSR1;
    volatile uint32_t FTSR1;
    volatile uint32_t SWIER1;
    volatile uint32_t PR1;
    uint32_t RESERVED1[2];
    volatile uint32_t IMR2;
    volatile uint32_t EMR2;
    volatile uint32_t RTSR2;
    volatile uint32_t FTSR2;
    volatile uint32_t SWIER2;
    volatile uint32_t PR2;
} EXTI_TypeDef;

// Estructura para los registros del NVIC (ISER - Interrupt Set-Enable Register)
typedef struct {
    volatile uint32_t ISER[8U];         /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
    uint32_t RESERVED0[24U];
    volatile uint32_t ICER[8U];         /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
    uint32_t RESERVED1[24U];
    volatile uint32_t ISPR[8U];         /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
    uint32_t RESERVED2[24U];
    volatile uint32_t ICPR[8U];         /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
    uint32_t RESERVED3[24U];
    volatile uint32_t IABR[8U];         /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
    uint32_t RESERVED4[56U];
    volatile uint8_t  IP[240U];         /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
    uint32_t RESERVED5[644U];
    volatile uint32_t STIR;             /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
} NVIC_Type;

#define SYSCFG_BASE         (0x40010000UL)
#define EXTI_BASE           (0x40010400UL)
#define NVIC_BASE           (0xE000E100UL)

#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI                ((EXTI_TypeDef *)   EXTI_BASE)
#define NVIC                ((NVIC_Type *)      NVIC_BASE)


// IRQn enumerations (ejemplos, deben coincidir con la tabla de vectores del STM32L476RG)
// Estos valores son los números de interrupción, no los índices de ISER.
typedef enum {
    /* STM32L4xxxx Specific Interrupt Numbers */
    WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                        */
    PVD_PVM_IRQn                = 1,      /*!< PVD/PVM through EXTI Line detection Interrupt    */
    TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI */
    RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line       */
    FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                           */
    RCC_IRQn                    = 5,      /*!< RCC global Interrupt                             */
    EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                             */
    EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                             */
    EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                             */
    EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                             */
    EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                             */
    DMA1_Channel1_IRQn          = 11,     /*!< DMA1 Channel 1 global Interrupt                  */
    DMA1_Channel2_IRQn          = 12,     /*!< DMA1 Channel 2 global Interrupt                  */
    DMA1_Channel3_IRQn          = 13,     /*!< DMA1 Channel 3 global Interrupt                  */
    DMA1_Channel4_IRQn          = 14,     /*!< DMA1 Channel 4 global Interrupt                  */
    DMA1_Channel5_IRQn          = 15,     /*!< DMA1 Channel 5 global Interrupt                  */
    DMA1_Channel6_IRQn          = 16,     /*!< DMA1 Channel 6 global Interrupt                  */
    DMA1_Channel7_IRQn          = 17,     /*!< DMA1 Channel 7 global Interrupt                  */
    ADC1_2_IRQn                 = 18,     /*!< ADC1, ADC2 global Interrupts                     */
    CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                */
    CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                               */
    CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                               */
    CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                               */
    EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                    */
    TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break interrupt and TIM15 global interrupt  */
    TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM16 global interrupt */
    TIM1_TRG_COM_TIM17_IRQn     = 26,     /*!< TIM1 Trigger,Commutation Interrupt, TIM17 global */
    TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                   */
    TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                            */
    TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                            */
    TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                            */
    I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                             */
    I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                             */
    I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                             */
    I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                             */
    SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                            */
    SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                            */
    USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                          */
    USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                          */
    USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                          */
    EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                  */
    RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt  */
    DFSDM1_FLT0_IRQn            = 42,     /*!< DFSDM1 Filter 0 global Interrupt                 */
    TIM8_BRK_IRQn               = 43,     /*!< TIM8 Break Interrupt                             */
    TIM8_UP_IRQn                = 44,     /*!< TIM8 Update Interrupt                            */
    TIM8_TRG_COM_IRQn           = 45,     /*!< TIM8 Trigger and Commutation Interrupt           */
    TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                   */
    ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                            */
    FMC_IRQn                    = 48,     /*!< FMC global Interrupt                             */
    SDMMC1_IRQn                 = 49,     /*!< SDMMC1 global Interrupt                          */
    TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                            */
    SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                            */
    UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                           */
    UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                           */
    TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts*/
    TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                            */
    DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                  */
    DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                  */
    DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                  */
    DMA2_Channel4_IRQn          = 59,     /*!< DMA2 Channel 4 global Interrupt                  */
    DMA2_Channel5_IRQn          = 60,     /*!< DMA2 Channel 5 global Interrupt                  */
    DFSDM1_FLT1_IRQn            = 61,     /*!< DFSDM1 Filter 1 global Interrupt                 */
    DFSDM1_FLT2_IRQn            = 62,     /*!< DFSDM1 Filter 2 global Interrupt                 */
    DFSDM1_FLT3_IRQn            = 63,     /*!< DFSDM1 Filter 3 global Interrupt                 */
    COMP_IRQn                   = 64,     /*!< COMP1 and COMP2 Interrupts                       */
    LPTIM1_IRQn                 = 65,     /*!< LP TIM1 interrupt                                */
    LPTIM2_IRQn                 = 66,     /*!< LP TIM2 interrupt                                */
    OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                      */
    DMA2_Channel6_IRQn          = 68,     /*!< DMA2 Channel 6 global Interrupt                  */
    DMA2_Channel7_IRQn          = 69,     /*!< DMA2 Channel 7 global Interrupt                  */
    LPUART1_IRQn                = 70,     /*!< LP UART1 interrupt                               */
    QUADSPI_IRQn                = 71,     /*!< Quad SPI global interrupt                        */
    I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                             */
    I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                             */
    SAI1_IRQn                   = 74,     /*!< Serial Audio Interface 1 global interrupt        */
    SAI2_IRQn                   = 75,     /*!< Serial Audio Interface 2 global interrupt        */
    SWPMI1_IRQn                 = 76,     /*!< SWPMI1 global interrupt                          */
    TSC_IRQn                    = 77,     /*!< Touch Sense Controller global interrupt          */
    LCD_IRQn                    = 78,     /*!< LCD global interrupt                             */
    AES_IRQn                    = 79,     /*!< AES global interrupt                             */
    RNG_IRQn                    = 80,     /*!< RNG global interrupt                             */
    FPU_IRQn                    = 81      /*!< FPU global interrupt                             */
} IRQn_Type;


void nvic_exti_pc13_button_enable(void); // Configura EXTI13 y habilita su IRQ en NVIC
void nvic_usart2_irq_enable(void);       // Habilita USART2 IRQ en NVIC (la config. de USART2 se hace en uart.c)

#endif // NVIC_H

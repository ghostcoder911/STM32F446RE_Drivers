/*
 * stm32f446xx.h
 *
 * Device header file for STM32F446xx MCU
 * Reference Manual: RM0390
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))


/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10C )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0          ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1          ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2          ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3          ((__vo uint32_t*)0xE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR   ((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4


/**********************************START: Memory Base Addresses **********************************/
/*
 * Base addresses of Flash and SRAM memories for STM32F446xx
 * Flash: Up to 512KB
 * SRAM1: 112KB (0x1C000 bytes)
 * SRAM2: 16KB (0x4000 bytes)
 */

#define FLASH_BASEADDR                      0x08000000U     /*!< Flash memory base address */
#define SRAM1_BASEADDR                      0x20000000U     /*!< SRAM1 base address (112KB) */
#define SRAM2_BASEADDR                      0x2001C000U     /*!< SRAM2 base address (16KB) */
#define ROM_BASEADDR                        0x1FFF0000U     /*!< System memory (ROM) base address */
#define SRAM                                SRAM1_BASEADDR


/**********************************START: Bus Domain Base Addresses **********************************/
/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR                     0x40000000U
#define APB1PERIPH_BASEADDR                 PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR                 0x40010000U
#define AHB1PERIPH_BASEADDR                 0x40020000U
#define AHB2PERIPH_BASEADDR                 0x50000000U
#define AHB3PERIPH_BASEADDR                 0xA0000000U


/**********************************START: AHB1 Peripheral Base Addresses **********************************/
/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * STM32F446xx has GPIO ports A to H (no GPIOI)
 */

#define GPIOA_BASEADDR                      (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                      (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR                      (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR                      (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR                      (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR                      (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR                      (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR                      (AHB1PERIPH_BASEADDR + 0x1C00)

#define CRC_BASEADDR                        (AHB1PERIPH_BASEADDR + 0x3000)
#define RCC_BASEADDR                        (AHB1PERIPH_BASEADDR + 0x3800)
#define FLASH_IF_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x3C00)
#define BKPSRAM_BASEADDR                    (AHB1PERIPH_BASEADDR + 0x4000)
#define DMA1_BASEADDR                       (AHB1PERIPH_BASEADDR + 0x6000)
#define DMA2_BASEADDR                       (AHB1PERIPH_BASEADDR + 0x6400)


/**********************************START: AHB2 Peripheral Base Addresses **********************************/
/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 */

#define USB_OTG_FS_BASEADDR                 (AHB2PERIPH_BASEADDR + 0x0000)
#define DCMI_BASEADDR                       (AHB2PERIPH_BASEADDR + 0x50000)


/**********************************START: AHB3 Peripheral Base Addresses **********************************/
/*
 * Base addresses of peripherals which are hanging on AHB3 bus
 */

#define FMC_BASEADDR                        (AHB3PERIPH_BASEADDR + 0x0000)
#define QUADSPI_BASEADDR                    0xA0001000U


/**********************************START: APB1 Peripheral Base Addresses **********************************/
/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define TIM2_BASEADDR                       (APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR                       (APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR                       (APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR                       (APB1PERIPH_BASEADDR + 0x0C00)
#define TIM6_BASEADDR                       (APB1PERIPH_BASEADDR + 0x1000)
#define TIM7_BASEADDR                       (APB1PERIPH_BASEADDR + 0x1400)
#define TIM12_BASEADDR                      (APB1PERIPH_BASEADDR + 0x1800)
#define TIM13_BASEADDR                      (APB1PERIPH_BASEADDR + 0x1C00)
#define TIM14_BASEADDR                      (APB1PERIPH_BASEADDR + 0x2000)
#define RTC_BKP_BASEADDR                    (APB1PERIPH_BASEADDR + 0x2800)
#define WWDG_BASEADDR                       (APB1PERIPH_BASEADDR + 0x2C00)
#define IWDG_BASEADDR                       (APB1PERIPH_BASEADDR + 0x3000)

#define SPI2_BASEADDR                       (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR                       (APB1PERIPH_BASEADDR + 0x3C00)
#define SPDIFRX_BASEADDR                    (APB1PERIPH_BASEADDR + 0x4000)

#define USART2_BASEADDR                     (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR                     (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR                      (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR                      (APB1PERIPH_BASEADDR + 0x5000)

#define I2C1_BASEADDR                       (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR                       (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR                       (APB1PERIPH_BASEADDR + 0x5C00)
#define FMPI2C1_BASEADDR                    (APB1PERIPH_BASEADDR + 0x6000)

#define CAN1_BASEADDR                       (APB1PERIPH_BASEADDR + 0x6400)
#define CAN2_BASEADDR                       (APB1PERIPH_BASEADDR + 0x6800)
#define CEC_BASEADDR                        (APB1PERIPH_BASEADDR + 0x6C00)
#define PWR_BASEADDR                        (APB1PERIPH_BASEADDR + 0x7000)
#define DAC_BASEADDR                        (APB1PERIPH_BASEADDR + 0x7400)


/**********************************START: APB2 Peripheral Base Addresses **********************************/
/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define TIM1_BASEADDR                       (APB2PERIPH_BASEADDR + 0x0000)
#define TIM8_BASEADDR                       (APB2PERIPH_BASEADDR + 0x0400)

#define USART1_BASEADDR                     (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR                     (APB2PERIPH_BASEADDR + 0x1400)

#define ADC1_BASEADDR                       (APB2PERIPH_BASEADDR + 0x2000)
#define ADC2_BASEADDR                       (APB2PERIPH_BASEADDR + 0x2100)
#define ADC3_BASEADDR                       (APB2PERIPH_BASEADDR + 0x2200)
#define ADC_COMMON_BASEADDR                 (APB2PERIPH_BASEADDR + 0x2300)

#define SDIO_BASEADDR                       (APB2PERIPH_BASEADDR + 0x2C00)

#define SPI1_BASEADDR                       (APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR                       (APB2PERIPH_BASEADDR + 0x3400)

#define SYSCFG_BASEADDR                     (APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR                       (APB2PERIPH_BASEADDR + 0x3C00)

#define TIM9_BASEADDR                       (APB2PERIPH_BASEADDR + 0x4000)
#define TIM10_BASEADDR                      (APB2PERIPH_BASEADDR + 0x4400)
#define TIM11_BASEADDR                      (APB2PERIPH_BASEADDR + 0x4800)

#define SAI1_BASEADDR                       (APB2PERIPH_BASEADDR + 0x5800)
#define SAI2_BASEADDR                       (APB2PERIPH_BASEADDR + 0x5C00)


/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * Please check your Device RM (RM0390 for STM32F446xx)
 */

/*
 * Peripheral register definition structure for GPIO
 */
typedef struct
{
    __vo uint32_t MODER;                    /*!< GPIO port mode register,                       Address offset: 0x00 */
    __vo uint32_t OTYPER;                   /*!< GPIO port output type register,                Address offset: 0x04 */
    __vo uint32_t OSPEEDR;                  /*!< GPIO port output speed register,               Address offset: 0x08 */
    __vo uint32_t PUPDR;                    /*!< GPIO port pull-up/pull-down register,          Address offset: 0x0C */
    __vo uint32_t IDR;                      /*!< GPIO port input data register,                 Address offset: 0x10 */
    __vo uint32_t ODR;                      /*!< GPIO port output data register,                Address offset: 0x14 */
    __vo uint32_t BSRR;                     /*!< GPIO port bit set/reset register,              Address offset: 0x18 */
    __vo uint32_t LCKR;                     /*!< GPIO port configuration lock register,         Address offset: 0x1C */
    __vo uint32_t AFR[2];                   /*!< AFR[0]: GPIO alternate function low register,  Address offset: 0x20 */
                                            /*!< AFR[1]: GPIO alternate function high register, Address offset: 0x24 */
} GPIO_RegDef_t;


/*
 * Peripheral register definition structure for RCC
 */
typedef struct
{
    __vo uint32_t CR;                       /*!< RCC clock control register,                            Address offset: 0x00 */
    __vo uint32_t PLLCFGR;                  /*!< RCC PLL configuration register,                        Address offset: 0x04 */
    __vo uint32_t CFGR;                     /*!< RCC clock configuration register,                      Address offset: 0x08 */
    __vo uint32_t CIR;                      /*!< RCC clock interrupt register,                          Address offset: 0x0C */
    __vo uint32_t AHB1RSTR;                 /*!< RCC AHB1 peripheral reset register,                    Address offset: 0x10 */
    __vo uint32_t AHB2RSTR;                 /*!< RCC AHB2 peripheral reset register,                    Address offset: 0x14 */
    __vo uint32_t AHB3RSTR;                 /*!< RCC AHB3 peripheral reset register,                    Address offset: 0x18 */
    uint32_t      RESERVED0;                /*!< Reserved,                                              Address offset: 0x1C */
    __vo uint32_t APB1RSTR;                 /*!< RCC APB1 peripheral reset register,                    Address offset: 0x20 */
    __vo uint32_t APB2RSTR;                 /*!< RCC APB2 peripheral reset register,                    Address offset: 0x24 */
    uint32_t      RESERVED1[2];             /*!< Reserved,                                              Address offset: 0x28-0x2C */
    __vo uint32_t AHB1ENR;                  /*!< RCC AHB1 peripheral clock enable register,             Address offset: 0x30 */
    __vo uint32_t AHB2ENR;                  /*!< RCC AHB2 peripheral clock enable register,             Address offset: 0x34 */
    __vo uint32_t AHB3ENR;                  /*!< RCC AHB3 peripheral clock enable register,             Address offset: 0x38 */
    uint32_t      RESERVED2;                /*!< Reserved,                                              Address offset: 0x3C */
    __vo uint32_t APB1ENR;                  /*!< RCC APB1 peripheral clock enable register,             Address offset: 0x40 */
    __vo uint32_t APB2ENR;                  /*!< RCC APB2 peripheral clock enable register,             Address offset: 0x44 */
    uint32_t      RESERVED3[2];             /*!< Reserved,                                              Address offset: 0x48-0x4C */
    __vo uint32_t AHB1LPENR;                /*!< RCC AHB1 peripheral clock enable in low power mode,    Address offset: 0x50 */
    __vo uint32_t AHB2LPENR;                /*!< RCC AHB2 peripheral clock enable in low power mode,    Address offset: 0x54 */
    __vo uint32_t AHB3LPENR;                /*!< RCC AHB3 peripheral clock enable in low power mode,    Address offset: 0x58 */
    uint32_t      RESERVED4;                /*!< Reserved,                                              Address offset: 0x5C */
    __vo uint32_t APB1LPENR;                /*!< RCC APB1 peripheral clock enable in low power mode,    Address offset: 0x60 */
    __vo uint32_t APB2LPENR;                /*!< RCC APB2 peripheral clock enable in low power mode,    Address offset: 0x64 */
    uint32_t      RESERVED5[2];             /*!< Reserved,                                              Address offset: 0x68-0x6C */
    __vo uint32_t BDCR;                     /*!< RCC Backup domain control register,                    Address offset: 0x70 */
    __vo uint32_t CSR;                      /*!< RCC clock control & status register,                   Address offset: 0x74 */
    uint32_t      RESERVED6[2];             /*!< Reserved,                                              Address offset: 0x78-0x7C */
    __vo uint32_t SSCGR;                    /*!< RCC spread spectrum clock generation register,         Address offset: 0x80 */
    __vo uint32_t PLLI2SCFGR;               /*!< RCC PLLI2S configuration register,                     Address offset: 0x84 */
    __vo uint32_t PLLSAICFGR;               /*!< RCC PLLSAI configuration register,                     Address offset: 0x88 */
    __vo uint32_t DCKCFGR;                  /*!< RCC dedicated clocks configuration register,           Address offset: 0x8C */
    __vo uint32_t CKGATENR;                 /*!< RCC clocks gated enable register,                      Address offset: 0x90 */
    __vo uint32_t DCKCFGR2;                 /*!< RCC dedicated clocks configuration register 2,         Address offset: 0x94 */
} RCC_RegDef_t;


/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
    __vo uint32_t IMR;                      /*!< Interrupt mask register,                       Address offset: 0x00 */
    __vo uint32_t EMR;                      /*!< Event mask register,                           Address offset: 0x04 */
    __vo uint32_t RTSR;                     /*!< Rising trigger selection register,             Address offset: 0x08 */
    __vo uint32_t FTSR;                     /*!< Falling trigger selection register,            Address offset: 0x0C */
    __vo uint32_t SWIER;                    /*!< Software interrupt event register,             Address offset: 0x10 */
    __vo uint32_t PR;                       /*!< Pending register,                              Address offset: 0x14 */
} EXTI_RegDef_t;


/*
 * Peripheral register definition structure for SPI
 */
typedef struct
{
    __vo uint32_t CR1;                      /*!< SPI control register 1,                        Address offset: 0x00 */
    __vo uint32_t CR2;                      /*!< SPI control register 2,                        Address offset: 0x04 */
    __vo uint32_t SR;                       /*!< SPI status register,                           Address offset: 0x08 */
    __vo uint32_t DR;                       /*!< SPI data register,                             Address offset: 0x0C */
    __vo uint32_t CRCPR;                    /*!< SPI CRC polynomial register,                   Address offset: 0x10 */
    __vo uint32_t RXCRCR;                   /*!< SPI RX CRC register,                           Address offset: 0x14 */
    __vo uint32_t TXCRCR;                   /*!< SPI TX CRC register,                           Address offset: 0x18 */
    __vo uint32_t I2SCFGR;                  /*!< SPI_I2S configuration register,                Address offset: 0x1C */
    __vo uint32_t I2SPR;                    /*!< SPI_I2S prescaler register,                    Address offset: 0x20 */
} SPI_RegDef_t;


/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
    __vo uint32_t MEMRMP;                   /*!< SYSCFG memory remap register,                  Address offset: 0x00 */
    __vo uint32_t PMC;                      /*!< SYSCFG peripheral mode configuration register, Address offset: 0x04 */
    __vo uint32_t EXTICR[4];                /*!< SYSCFG external interrupt configuration,       Address offset: 0x08-0x14 */
    uint32_t      RESERVED1[2];             /*!< Reserved,                                      Address offset: 0x18-0x1C */
    __vo uint32_t CMPCR;                    /*!< SYSCFG compensation cell control register,     Address offset: 0x20 */
    uint32_t      RESERVED2[2];             /*!< Reserved,                                      Address offset: 0x24-0x28 */
    __vo uint32_t CFGR;                     /*!< SYSCFG configuration register,                 Address offset: 0x2C */
} SYSCFG_RegDef_t;


/*
 * Peripheral register definition structure for I2C
 */
typedef struct
{
    __vo uint32_t CR1;                      /*!< I2C control register 1,                        Address offset: 0x00 */
    __vo uint32_t CR2;                      /*!< I2C control register 2,                        Address offset: 0x04 */
    __vo uint32_t OAR1;                     /*!< I2C own address register 1,                    Address offset: 0x08 */
    __vo uint32_t OAR2;                     /*!< I2C own address register 2,                    Address offset: 0x0C */
    __vo uint32_t DR;                       /*!< I2C data register,                             Address offset: 0x10 */
    __vo uint32_t SR1;                      /*!< I2C status register 1,                         Address offset: 0x14 */
    __vo uint32_t SR2;                      /*!< I2C status register 2,                         Address offset: 0x18 */
    __vo uint32_t CCR;                      /*!< I2C clock control register,                    Address offset: 0x1C */
    __vo uint32_t TRISE;                    /*!< I2C TRISE register,                            Address offset: 0x20 */
    __vo uint32_t FLTR;                     /*!< I2C FLTR register,                             Address offset: 0x24 */
} I2C_RegDef_t;


/*
 * Peripheral register definition structure for USART
 */
typedef struct
{
    __vo uint32_t SR;                       /*!< USART status register,                         Address offset: 0x00 */
    __vo uint32_t DR;                       /*!< USART data register,                           Address offset: 0x04 */
    __vo uint32_t BRR;                      /*!< USART baud rate register,                      Address offset: 0x08 */
    __vo uint32_t CR1;                      /*!< USART control register 1,                      Address offset: 0x0C */
    __vo uint32_t CR2;                      /*!< USART control register 2,                      Address offset: 0x10 */
    __vo uint32_t CR3;                      /*!< USART control register 3,                      Address offset: 0x14 */
    __vo uint32_t GTPR;                     /*!< USART guard time and prescaler register,       Address offset: 0x18 */
} USART_RegDef_t;


/**********************************Peripheral Definitions **********************************/
/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA                               ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                               ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                               ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                               ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE                               ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF                               ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG                               ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH                               ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC                                 ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI                                ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG                              ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1                                ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2                                ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3                                ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4                                ((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1                                ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2                                ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3                                ((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1                              ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2                              ((USART_RegDef_t*)USART2_BASEADDR)
#define USART3                              ((USART_RegDef_t*)USART3_BASEADDR)
#define UART4                               ((USART_RegDef_t*)UART4_BASEADDR)
#define UART5                               ((USART_RegDef_t*)UART5_BASEADDR)
#define USART6                              ((USART_RegDef_t*)USART6_BASEADDR)


/**********************************Clock Enable Macros **********************************/

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()                     (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()                     (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()                     (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()                     (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()                     (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()                     (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()                     (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()                     (RCC->AHB1ENR |= (1 << 7))


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()                      (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()                      (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()                      (RCC->APB1ENR |= (1 << 23))
#define FMPI2C1_PCLK_EN()                   (RCC->APB1ENR |= (1 << 24))


/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()                      (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()                      (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()                      (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()                      (RCC->APB2ENR |= (1 << 13))


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()                    (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()                    (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()                    (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()                     (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()                     (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()                    (RCC->APB2ENR |= (1 << 5))

/* Aliases for backward compatibility */
#define USART1_PCCK_EN()                    USART1_PCLK_EN()
#define USART2_PCCK_EN()                    USART2_PCLK_EN()
#define USART3_PCCK_EN()                    USART3_PCLK_EN()
#define UART4_PCCK_EN()                     UART4_PCLK_EN()
#define UART5_PCCK_EN()                     UART5_PCLK_EN()
#define USART6_PCCK_EN()                    USART6_PCLK_EN()


/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()                    (RCC->APB2ENR |= (1 << 14))


/*
 * Clock Enable Macros for CRC peripheral
 */
#define CRC_PCLK_EN()                       (RCC->AHB1ENR |= (1 << 12))


/*
 * Clock Enable Macros for DMA peripherals
 */
#define DMA1_PCLK_EN()                      (RCC->AHB1ENR |= (1 << 21))
#define DMA2_PCLK_EN()                      (RCC->AHB1ENR |= (1 << 22))


/**********************************Clock Disable Macros **********************************/

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()                     (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()                     (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()                     (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()                     (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()                     (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()                     (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()                     (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()                     (RCC->AHB1ENR &= ~(1 << 7))


/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()                      (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()                      (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()                      (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()                      (RCC->APB2ENR &= ~(1 << 13))


/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()                      (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()                      (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()                      (RCC->APB1ENR &= ~(1 << 23))
#define FMPI2C1_PCLK_DI()                   (RCC->APB1ENR &= ~(1 << 24))


/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()                    (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()                    (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()                    (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()                     (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()                     (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()                    (RCC->APB2ENR &= ~(1 << 5))


/*
 * Clock Disable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI()                    (RCC->APB2ENR &= ~(1 << 14))


/**********************************Peripheral Reset Macros **********************************/

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()                   do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()                   do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()                   do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()                   do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()                   do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()                   do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()                   do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()                   do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)


/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()                    do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()                    do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()                    do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()                    do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)


/*
 * Macros to reset I2Cx peripherals
 */
#define I2C1_REG_RESET()                    do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()                    do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()                    do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); }while(0)


/*
 * Macros to reset USARTx peripherals
 */
#define USART1_REG_RESET()                  do{ (RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4)); }while(0)
#define USART2_REG_RESET()                  do{ (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); }while(0)
#define USART3_REG_RESET()                  do{ (RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18)); }while(0)
#define UART4_REG_RESET()                   do{ (RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19)); }while(0)
#define UART5_REG_RESET()                   do{ (RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20)); }while(0)
#define USART6_REG_RESET()                  do{ (RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5)); }while(0)


/**********************************GPIO Port Code Macro **********************************/

/*
 * This macro returns a code (between 0 to 7) for a given GPIO base address (x)
 * STM32F446xx has GPIO ports A-H only
 */
#define GPIO_BASEADDR_TO_CODE(x)            ( (x == GPIOA) ? 0 :\
                                              (x == GPIOB) ? 1 :\
                                              (x == GPIOC) ? 2 :\
                                              (x == GPIOD) ? 3 :\
                                              (x == GPIOE) ? 4 :\
                                              (x == GPIOF) ? 5 :\
                                              (x == GPIOG) ? 6 :\
                                              (x == GPIOH) ? 7 : 0 )


/**********************************IRQ Numbers **********************************/

/*
 * IRQ (Interrupt Request) Numbers of STM32F446xx MCU
 * NOTE: These values are from the STM32F446xx reference manual (RM0390)
 */

#define IRQ_NO_WWDG                         0
#define IRQ_NO_PVD                          1
#define IRQ_NO_TAMP_STAMP                   2
#define IRQ_NO_RTC_WKUP                     3
#define IRQ_NO_FLASH                        4
#define IRQ_NO_RCC                          5
#define IRQ_NO_EXTI0                        6
#define IRQ_NO_EXTI1                        7
#define IRQ_NO_EXTI2                        8
#define IRQ_NO_EXTI3                        9
#define IRQ_NO_EXTI4                        10
#define IRQ_NO_DMA1_STREAM0                 11
#define IRQ_NO_DMA1_STREAM1                 12
#define IRQ_NO_DMA1_STREAM2                 13
#define IRQ_NO_DMA1_STREAM3                 14
#define IRQ_NO_DMA1_STREAM4                 15
#define IRQ_NO_DMA1_STREAM5                 16
#define IRQ_NO_DMA1_STREAM6                 17
#define IRQ_NO_ADC                          18
#define IRQ_NO_CAN1_TX                      19
#define IRQ_NO_CAN1_RX0                     20
#define IRQ_NO_CAN1_RX1                     21
#define IRQ_NO_CAN1_SCE                     22
#define IRQ_NO_EXTI9_5                      23
#define IRQ_NO_TIM1_BRK_TIM9                24
#define IRQ_NO_TIM1_UP_TIM10                25
#define IRQ_NO_TIM1_TRG_COM_TIM11           26
#define IRQ_NO_TIM1_CC                      27
#define IRQ_NO_TIM2                         28
#define IRQ_NO_TIM3                         29
#define IRQ_NO_TIM4                         30
#define IRQ_NO_I2C1_EV                      31
#define IRQ_NO_I2C1_ER                      32
#define IRQ_NO_I2C2_EV                      33
#define IRQ_NO_I2C2_ER                      34
#define IRQ_NO_SPI1                         35
#define IRQ_NO_SPI2                         36
#define IRQ_NO_USART1                       37
#define IRQ_NO_USART2                       38
#define IRQ_NO_USART3                       39
#define IRQ_NO_EXTI15_10                    40
#define IRQ_NO_RTC_ALARM                    41
#define IRQ_NO_OTG_FS_WKUP                  42
#define IRQ_NO_TIM8_BRK_TIM12               43
#define IRQ_NO_TIM8_UP_TIM13                44
#define IRQ_NO_TIM8_TRG_COM_TIM14           45
#define IRQ_NO_TIM8_CC                      46
#define IRQ_NO_DMA1_STREAM7                 47
#define IRQ_NO_FMC                          48
#define IRQ_NO_SDIO                         49
#define IRQ_NO_TIM5                         50
#define IRQ_NO_SPI3                         51
#define IRQ_NO_UART4                        52
#define IRQ_NO_UART5                        53
#define IRQ_NO_TIM6_DAC                     54
#define IRQ_NO_TIM7                         55
#define IRQ_NO_DMA2_STREAM0                 56
#define IRQ_NO_DMA2_STREAM1                 57
#define IRQ_NO_DMA2_STREAM2                 58
#define IRQ_NO_DMA2_STREAM3                 59
#define IRQ_NO_DMA2_STREAM4                 60
#define IRQ_NO_CAN2_TX                      63
#define IRQ_NO_CAN2_RX0                     64
#define IRQ_NO_CAN2_RX1                     65
#define IRQ_NO_CAN2_SCE                     66
#define IRQ_NO_OTG_FS                       67
#define IRQ_NO_DMA2_STREAM5                 68
#define IRQ_NO_DMA2_STREAM6                 69
#define IRQ_NO_DMA2_STREAM7                 70
#define IRQ_NO_USART6                       71
#define IRQ_NO_I2C3_EV                      72
#define IRQ_NO_I2C3_ER                      73
#define IRQ_NO_OTG_HS_EP1_OUT               74
#define IRQ_NO_OTG_HS_EP1_IN                75
#define IRQ_NO_OTG_HS_WKUP                  76
#define IRQ_NO_OTG_HS                       77
#define IRQ_NO_DCMI                         78
#define IRQ_NO_FPU                          81
#define IRQ_NO_SPI4                         84
#define IRQ_NO_SAI1                         87
#define IRQ_NO_SAI2                         91
#define IRQ_NO_QUADSPI                      92
#define IRQ_NO_CEC                          93
#define IRQ_NO_SPDIF_RX                     94
#define IRQ_NO_FMPI2C1_EV                   95
#define IRQ_NO_FMPI2C1_ER                   96


/**********************************NVIC Priority Macros **********************************/

/*
 * Macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0                       0
#define NVIC_IRQ_PRI1                       1
#define NVIC_IRQ_PRI2                       2
#define NVIC_IRQ_PRI3                       3
#define NVIC_IRQ_PRI4                       4
#define NVIC_IRQ_PRI5                       5
#define NVIC_IRQ_PRI6                       6
#define NVIC_IRQ_PRI7                       7
#define NVIC_IRQ_PRI8                       8
#define NVIC_IRQ_PRI9                       9
#define NVIC_IRQ_PRI10                      10
#define NVIC_IRQ_PRI11                      11
#define NVIC_IRQ_PRI12                      12
#define NVIC_IRQ_PRI13                      13
#define NVIC_IRQ_PRI14                      14
#define NVIC_IRQ_PRI15                      15


/**********************************Generic Macros **********************************/

#define ENABLE                              1
#define DISABLE                             0
#define SET                                 ENABLE
#define RESET                               DISABLE
#define GPIO_PIN_SET                        SET
#define GPIO_PIN_RESET                      RESET
#define FLAG_RESET                          RESET
#define FLAG_SET                            SET


/******************************************************************************************
 * Bit position definitions of SPI peripheral
 ******************************************************************************************/

/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA                        0
#define SPI_CR1_CPOL                        1
#define SPI_CR1_MSTR                        2
#define SPI_CR1_BR                          3
#define SPI_CR1_SPE                         6
#define SPI_CR1_LSBFIRST                    7
#define SPI_CR1_SSI                         8
#define SPI_CR1_SSM                         9
#define SPI_CR1_RXONLY                      10
#define SPI_CR1_DFF                         11
#define SPI_CR1_CRCNEXT                     12
#define SPI_CR1_CRCEN                       13
#define SPI_CR1_BIDIOE                      14
#define SPI_CR1_BIDIMODE                    15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN                     0
#define SPI_CR2_TXDMAEN                     1
#define SPI_CR2_SSOE                        2
#define SPI_CR2_FRF                         4
#define SPI_CR2_ERRIE                       5
#define SPI_CR2_RXNEIE                      6
#define SPI_CR2_TXEIE                       7

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE                         0
#define SPI_SR_TXE                          1
#define SPI_SR_CHSIDE                       2
#define SPI_SR_UDR                          3
#define SPI_SR_CRCERR                       4
#define SPI_SR_MODF                         5
#define SPI_SR_OVR                          6
#define SPI_SR_BSY                          7
#define SPI_SR_FRE                          8


/******************************************************************************************
 * Bit position definitions of I2C peripheral
 ******************************************************************************************/

/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE                          0
#define I2C_CR1_SMBUS                       1
#define I2C_CR1_SMBTYPE                     3
#define I2C_CR1_ENARP                       4
#define I2C_CR1_ENPEC                       5
#define I2C_CR1_ENGC                        6
#define I2C_CR1_NOSTRETCH                   7
#define I2C_CR1_START                       8
#define I2C_CR1_STOP                        9
#define I2C_CR1_ACK                         10
#define I2C_CR1_POS                         11
#define I2C_CR1_PEC                         12
#define I2C_CR1_ALERT                       13
#define I2C_CR1_SWRST                       15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ                        0
#define I2C_CR2_ITERREN                     8
#define I2C_CR2_ITEVTEN                     9
#define I2C_CR2_ITBUFEN                     10
#define I2C_CR2_DMAEN                       11
#define I2C_CR2_LAST                        12

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0                       0
#define I2C_OAR1_ADD71                      1
#define I2C_OAR1_ADD98                      8
#define I2C_OAR1_ADDMODE                    15

/*
 * Bit position definitions I2C_SR1
 */
#define I2C_SR1_SB                          0
#define I2C_SR1_ADDR                        1
#define I2C_SR1_BTF                         2
#define I2C_SR1_ADD10                       3
#define I2C_SR1_STOPF                       4
#define I2C_SR1_RXNE                        6
#define I2C_SR1_TXE                         7
#define I2C_SR1_BERR                        8
#define I2C_SR1_ARLO                        9
#define I2C_SR1_AF                          10
#define I2C_SR1_OVR                         11
#define I2C_SR1_PECERR                      12
#define I2C_SR1_TIMEOUT                     14
#define I2C_SR1_SMBALERT                    15

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL                         0
#define I2C_SR2_BUSY                        1
#define I2C_SR2_TRA                         2
#define I2C_SR2_GENCALL                     4
#define I2C_SR2_SMBDEFAULT                  5
#define I2C_SR2_SMBHOST                     6
#define I2C_SR2_DUALF                       7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR                         0
#define I2C_CCR_DUTY                        14
#define I2C_CCR_FS                          15


/******************************************************************************************
 * Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK                       0
#define USART_CR1_RWU                       1
#define USART_CR1_RE                        2
#define USART_CR1_TE                        3
#define USART_CR1_IDLEIE                    4
#define USART_CR1_RXNEIE                    5
#define USART_CR1_TCIE                      6
#define USART_CR1_TXEIE                     7
#define USART_CR1_PEIE                      8
#define USART_CR1_PS                        9
#define USART_CR1_PCE                       10
#define USART_CR1_WAKE                      11
#define USART_CR1_M                         12
#define USART_CR1_UE                        13
#define USART_CR1_OVER8                     15

/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD                       0
#define USART_CR2_LBDL                      5
#define USART_CR2_LBDIE                     6
#define USART_CR2_LBCL                      8
#define USART_CR2_CPHA                      9
#define USART_CR2_CPOL                      10
#define USART_CR2_CLKEN                     11
#define USART_CR2_STOP                      12
#define USART_CR2_LINEN                     14

/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE                       0
#define USART_CR3_IREN                      1
#define USART_CR3_IRLP                      2
#define USART_CR3_HDSEL                     3
#define USART_CR3_NACK                      4
#define USART_CR3_SCEN                      5
#define USART_CR3_DMAR                      6
#define USART_CR3_DMAT                      7
#define USART_CR3_RTSE                      8
#define USART_CR3_CTSE                      9
#define USART_CR3_CTSIE                     10
#define USART_CR3_ONEBIT                    11

/*
 * Bit position definitions USART_SR
 */
#define USART_SR_PE                         0
#define USART_SR_FE                         1
#define USART_SR_NE                         2
#define USART_SR_ORE                        3
#define USART_SR_IDLE                       4
#define USART_SR_RXNE                       5
#define USART_SR_TC                         6
#define USART_SR_TXE                        7
#define USART_SR_LBD                        8
#define USART_SR_CTS                        9


/******************************************************************************************
 * Bit position definitions of RCC peripheral
 ******************************************************************************************/

/*
 * Bit position definitions RCC_CR
 */
#define RCC_CR_HSION                        0
#define RCC_CR_HSIRDY                       1
#define RCC_CR_HSITRIM                      3
#define RCC_CR_HSICAL                       8
#define RCC_CR_HSEON                        16
#define RCC_CR_HSERDY                       17
#define RCC_CR_HSEBYP                       18
#define RCC_CR_CSSON                        19
#define RCC_CR_PLLON                        24
#define RCC_CR_PLLRDY                       25
#define RCC_CR_PLLI2SON                     26
#define RCC_CR_PLLI2SRDY                    27
#define RCC_CR_PLLSAION                     28
#define RCC_CR_PLLSAIRDY                    29

/*
 * Bit position definitions RCC_CFGR
 */
#define RCC_CFGR_SW                         0
#define RCC_CFGR_SWS                        2
#define RCC_CFGR_HPRE                       4
#define RCC_CFGR_PPRE1                      10
#define RCC_CFGR_PPRE2                      13
#define RCC_CFGR_RTCPRE                     16
#define RCC_CFGR_MCO1                       21
#define RCC_CFGR_MCO1PRE                    24
#define RCC_CFGR_MCO2PRE                    27
#define RCC_CFGR_MCO2                       30


#endif /* INC_STM32F446XX_H_ */


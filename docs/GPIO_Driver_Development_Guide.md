# Complete Guide to GPIO Driver Development

A step-by-step guide to understanding and developing GPIO drivers for any microcontroller.

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Prerequisites](#2-prerequisites)
3. [Understanding GPIO Fundamentals](#3-understanding-gpio-fundamentals)
4. [Reading the Reference Manual](#4-reading-the-reference-manual)
5. [Driver Architecture Overview](#5-driver-architecture-overview)
6. [Step 1: Creating the MCU Header File](#6-step-1-creating-the-mcu-header-file)
7. [Step 2: Creating the GPIO Driver Header File](#7-step-2-creating-the-gpio-driver-header-file)
8. [Step 3: Implementing the GPIO Driver Source File](#8-step-3-implementing-the-gpio-driver-source-file)
9. [Testing Your Driver](#9-testing-your-driver)
10. [Best Practices](#10-best-practices)
11. [Common Mistakes to Avoid](#11-common-mistakes-to-avoid)
12. [Summary Checklist](#12-summary-checklist)

---

## 1. Introduction

### What is a GPIO Driver?

A GPIO (General Purpose Input/Output) driver is software that provides an interface between your application code and the GPIO hardware of a microcontroller. It abstracts the low-level register manipulations and provides easy-to-use functions.

### Why Write Your Own Driver?

- **Deep Understanding**: Learn how hardware actually works
- **Optimization**: Tailor the driver for your specific needs
- **Portability**: Create consistent interfaces across different MCUs
- **No Dependencies**: Remove reliance on vendor HAL libraries
- **Educational Value**: Essential skill for embedded systems engineers

### What You'll Learn

By the end of this guide, you will be able to:
- Read and interpret MCU reference manuals
- Design driver architecture
- Create register definition structures
- Implement GPIO configuration and control functions
- Handle GPIO interrupts

---

## 2. Prerequisites

### Knowledge Required

- Basic C programming (pointers, structures, bitwise operations)
- Understanding of binary and hexadecimal numbers
- Basic electronics (what is a pin, HIGH/LOW states)

### Tools Needed

- MCU Reference Manual (datasheet)
- Text editor or IDE
- Compiler toolchain for your MCU
- Hardware for testing (development board)

### Key C Concepts You Must Know

```c
// 1. Pointers - Accessing memory addresses
uint32_t *ptr = (uint32_t *)0x40020000;  // Point to address
*ptr = 0x01;                              // Write to that address

// 2. Structures - Grouping related data
typedef struct {
    uint32_t reg1;
    uint32_t reg2;
} MyStruct_t;

// 3. Bitwise Operations
value |= (1 << 5);    // Set bit 5
value &= ~(1 << 5);   // Clear bit 5
value ^= (1 << 5);    // Toggle bit 5
value = (value >> 3) & 0x03;  // Extract bits 3-4

// 4. Volatile keyword - Prevents compiler optimization
volatile uint32_t *reg = (volatile uint32_t *)0x40020000;
```

---

## 3. Understanding GPIO Fundamentals

### What is GPIO?

GPIO pins are the physical pins on a microcontroller that can be configured as either:
- **Input**: Read external signals (buttons, sensors)
- **Output**: Control external devices (LEDs, motors)

### GPIO Pin States

```
INPUT MODE:
    External Signal → Pin → MCU reads 0 or 1

OUTPUT MODE:
    MCU writes 0 or 1 → Pin → External Device
    
    HIGH (1): Pin connected to VDD (3.3V or 5V)
    LOW (0):  Pin connected to GND (0V)
```

### Common GPIO Configurations

```
┌─────────────────────────────────────────────────────────────┐
│                     GPIO PIN MODES                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. INPUT MODE                                              │
│     ┌─────┐                                                 │
│     │ PIN │←── External Signal                              │
│     └─────┘                                                 │
│     • Floating (no pull-up/down)                            │
│     • Pull-up (internal resistor to VDD)                    │
│     • Pull-down (internal resistor to GND)                  │
│                                                             │
│  2. OUTPUT MODE                                             │
│     ┌─────┐                                                 │
│     │ PIN │──→ External Device                              │
│     └─────┘                                                 │
│     • Push-Pull (can drive HIGH and LOW)                    │
│     • Open-Drain (can only pull LOW, needs external pull-up)│
│                                                             │
│  3. ALTERNATE FUNCTION MODE                                 │
│     Pin controlled by peripheral (UART, SPI, I2C, etc.)     │
│                                                             │
│  4. ANALOG MODE                                             │
│     Pin used for ADC/DAC operations                         │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Output Types Explained

```
PUSH-PULL OUTPUT:
                VDD
                 │
              ┌──┴──┐
              │ P-FET│←── When output = 1, P-FET ON, pin = HIGH
              └──┬──┘
                 ├────→ PIN
              ┌──┴──┐
              │ N-FET│←── When output = 0, N-FET ON, pin = LOW
              └──┬──┘
                 │
                GND

OPEN-DRAIN OUTPUT:
                 
              External Pull-up
                 │
                 ├────→ PIN
              ┌──┴──┐
              │ N-FET│←── When output = 0, N-FET ON, pin = LOW
              └──┬──┘    When output = 1, N-FET OFF, pin floats (pulled HIGH by external resistor)
                 │
                GND
```

### Pull-up and Pull-down Resistors

```
PULL-UP CONFIGURATION:          PULL-DOWN CONFIGURATION:
        VDD                             VDD
         │                               │
        [R] ← Internal                   │
         │   Pull-up                     │
         ├───→ PIN                       ├───→ PIN
         │                               │
      [SWITCH]                        [SWITCH]
         │                               │
        GND                             [R] ← Internal
                                         │   Pull-down
                                        GND

• Pull-up: Pin reads HIGH when switch open, LOW when pressed
• Pull-down: Pin reads LOW when switch open, HIGH when pressed
```

---

## 4. Reading the Reference Manual

The reference manual is your bible for driver development. Here's how to extract the information you need.

### Step 4.1: Find the Memory Map

Look for the "Memory Map" section. This tells you where peripherals are located in memory.

```
Example Memory Map (STM32F446):
┌────────────────────┬─────────────────┐
│ Address Range      │ Peripheral      │
├────────────────────┼─────────────────┤
│ 0x4002 0000        │ GPIOA           │
│ 0x4002 0400        │ GPIOB           │
│ 0x4002 0800        │ GPIOC           │
│ 0x4002 0C00        │ GPIOD           │
│ ...                │ ...             │
│ 0x4002 3800        │ RCC             │
└────────────────────┴─────────────────┘
```

### Step 4.2: Find GPIO Register Descriptions

Look for the "GPIO" chapter. You'll find register descriptions like this:

```
GPIO Port Mode Register (GPIOx_MODER)
Address offset: 0x00
Reset value: 0xFFFF FFFF (for port A)

┌────┬────┬────┬────┬────┬────┬────┬────┐
│ 31 │ 30 │ 29 │ 28 │...│  3 │  2 │  1 │  0 │
├────┴────┼────┴────┼────┴────┼────┴────┤
│MODER15  │MODER14  │  ...    │ MODER0  │
└─────────┴─────────┴─────────┴─────────┘

MODERy[1:0]: Port x configuration bits (y = 0..15)
  00: Input mode (reset state)
  01: General purpose output mode
  10: Alternate function mode
  11: Analog mode
```

### Step 4.3: Document All GPIO Registers

Create a list of all registers you need:

| Register | Offset | Purpose |
|----------|--------|---------|
| MODER    | 0x00   | Pin mode (input/output/alternate/analog) |
| OTYPER   | 0x04   | Output type (push-pull/open-drain) |
| OSPEEDR  | 0x08   | Output speed |
| PUPDR    | 0x0C   | Pull-up/Pull-down |
| IDR      | 0x10   | Input data (read pin states) |
| ODR      | 0x14   | Output data (set pin states) |
| BSRR     | 0x18   | Bit set/reset (atomic operations) |
| LCKR     | 0x1C   | Configuration lock |
| AFRL     | 0x20   | Alternate function low (pins 0-7) |
| AFRH     | 0x24   | Alternate function high (pins 8-15) |

### Step 4.4: Find Clock Enable Information

GPIO peripherals need their clocks enabled before use. Find the RCC (Reset and Clock Control) chapter.

```
RCC AHB1 Peripheral Clock Enable Register (RCC_AHB1ENR)
Address: 0x4002 3830

Bit 0: GPIOAEN - GPIOA clock enable
Bit 1: GPIOBEN - GPIOB clock enable
Bit 2: GPIOCEN - GPIOC clock enable
...
```

---

## 5. Driver Architecture Overview

A well-structured GPIO driver consists of three main files:

```
┌─────────────────────────────────────────────────────────────┐
│                    DRIVER ARCHITECTURE                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │           MCU Header File (stm32f446xx.h)           │   │
│  │  • Base addresses                                    │   │
│  │  • Register definition structures                    │   │
│  │  • Peripheral pointers                               │   │
│  │  • Clock enable/disable macros                       │   │
│  │  • Bit position definitions                          │   │
│  └─────────────────────────────────────────────────────┘   │
│                           │                                 │
│                           ▼                                 │
│  ┌─────────────────────────────────────────────────────┐   │
│  │      GPIO Driver Header (stm32f446xx_gpio_driver.h) │   │
│  │  • Configuration structure                           │   │
│  │  • Handle structure                                  │   │
│  │  • Configuration macros                              │   │
│  │  • API function prototypes                           │   │
│  └─────────────────────────────────────────────────────┘   │
│                           │                                 │
│                           ▼                                 │
│  ┌─────────────────────────────────────────────────────┐   │
│  │      GPIO Driver Source (stm32f446xx_gpio_driver.c) │   │
│  │  • API function implementations                      │   │
│  │  • Clock control                                     │   │
│  │  • Initialization                                    │   │
│  │  • Read/Write operations                             │   │
│  │  • Interrupt handling                                │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 6. Step 1: Creating the MCU Header File

This file contains all hardware-specific definitions for your microcontroller.

### 6.1 Start with Include Guards and Standard Headers

```c
/*
 * stm32f446xx.h
 * Device header file for STM32F446xx MCU
 */

#ifndef STM32F446XX_H_
#define STM32F446XX_H_

#include <stdint.h>    // For uint8_t, uint32_t, etc.
#include <stddef.h>    // For NULL

// Volatile macro for register access
#define __vo volatile
```

**Why `volatile`?**
- Tells compiler not to optimize access to these variables
- Register values can change by hardware at any time
- Without `volatile`, compiler might cache values and miss changes

### 6.2 Define Processor-Specific Details (ARM Cortex-M)

```c
/*
 * ARM Cortex-M4 Processor NVIC Register Addresses
 * These are needed for interrupt configuration
 */

// Interrupt Set Enable Registers
#define NVIC_ISER0          ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1          ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2          ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3          ((__vo uint32_t*)0xE000E10C)

// Interrupt Clear Enable Registers
#define NVIC_ICER0          ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1          ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2          ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3          ((__vo uint32_t*)0xE000E18C)

// Priority Register Base Address
#define NVIC_PR_BASE_ADDR   ((__vo uint32_t*)0xE000E400)

// Number of priority bits implemented (MCU specific)
#define NO_PR_BITS_IMPLEMENTED  4
```

### 6.3 Define Memory Base Addresses

```c
/*
 * Base addresses of Flash and SRAM memories
 * Find these in the "Memory Map" section of reference manual
 */

#define FLASH_BASEADDR      0x08000000U   // Flash memory start
#define SRAM1_BASEADDR      0x20000000U   // SRAM1 start
#define SRAM2_BASEADDR      0x2001C000U   // SRAM2 start (if available)
#define ROM_BASEADDR        0x1FFF0000U   // System memory (bootloader)
#define SRAM                SRAM1_BASEADDR
```

### 6.4 Define Peripheral Bus Base Addresses

```c
/*
 * Peripheral Bus Base Addresses
 * Peripherals are grouped on different buses (APB1, APB2, AHB1, etc.)
 */

#define PERIPH_BASEADDR         0x40000000U
#define APB1PERIPH_BASEADDR     PERIPH_BASEADDR        // 0x40000000
#define APB2PERIPH_BASEADDR     0x40010000U
#define AHB1PERIPH_BASEADDR     0x40020000U
#define AHB2PERIPH_BASEADDR     0x50000000U
```

### 6.5 Define Individual Peripheral Base Addresses

```c
/*
 * GPIO Peripheral Base Addresses (on AHB1 bus)
 * Each GPIO port is 0x400 bytes apart
 */

#define GPIOA_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1C00)

// RCC (Reset and Clock Control)
#define RCC_BASEADDR        (AHB1PERIPH_BASEADDR + 0x3800)

// EXTI (External Interrupt) - for GPIO interrupts
#define EXTI_BASEADDR       (APB2PERIPH_BASEADDR + 0x3C00)

// SYSCFG (System Configuration) - for interrupt routing
#define SYSCFG_BASEADDR     (APB2PERIPH_BASEADDR + 0x3800)
```

### 6.6 Create Register Definition Structures

This is the most important part. Map the hardware registers to a C structure.

```c
/*
 * GPIO Register Definition Structure
 * 
 * The order of members MUST match the register memory layout!
 * Each member represents a 32-bit register at its offset address.
 */

typedef struct
{
    __vo uint32_t MODER;    // Mode register,              offset: 0x00
    __vo uint32_t OTYPER;   // Output type register,       offset: 0x04
    __vo uint32_t OSPEEDR;  // Output speed register,      offset: 0x08
    __vo uint32_t PUPDR;    // Pull-up/pull-down register, offset: 0x0C
    __vo uint32_t IDR;      // Input data register,        offset: 0x10
    __vo uint32_t ODR;      // Output data register,       offset: 0x14
    __vo uint32_t BSRR;     // Bit set/reset register,     offset: 0x18
    __vo uint32_t LCKR;     // Configuration lock register,offset: 0x1C
    __vo uint32_t AFR[2];   // Alternate function registers (LOW & HIGH)
                            // AFR[0] = AFRL, offset: 0x20
                            // AFR[1] = AFRH, offset: 0x24
} GPIO_RegDef_t;
```

**How does this work?**

```
Memory Layout:
┌─────────────────────────────────────────────────┐
│ GPIO Base Address (e.g., 0x40020000 for GPIOA)  │
├─────────────────────────────────────────────────┤
│ Offset 0x00: MODER   ← GPIO_RegDef_t.MODER      │
│ Offset 0x04: OTYPER  ← GPIO_RegDef_t.OTYPER     │
│ Offset 0x08: OSPEEDR ← GPIO_RegDef_t.OSPEEDR    │
│ Offset 0x0C: PUPDR   ← GPIO_RegDef_t.PUPDR      │
│ Offset 0x10: IDR     ← GPIO_RegDef_t.IDR        │
│ Offset 0x14: ODR     ← GPIO_RegDef_t.ODR        │
│ Offset 0x18: BSRR    ← GPIO_RegDef_t.BSRR       │
│ Offset 0x1C: LCKR    ← GPIO_RegDef_t.LCKR       │
│ Offset 0x20: AFRL    ← GPIO_RegDef_t.AFR[0]     │
│ Offset 0x24: AFRH    ← GPIO_RegDef_t.AFR[1]     │
└─────────────────────────────────────────────────┘
```

### 6.7 Create RCC Register Definition Structure

```c
/*
 * RCC Register Definition Structure
 * Needed for enabling peripheral clocks
 */

typedef struct
{
    __vo uint32_t CR;           // Clock control register,           offset: 0x00
    __vo uint32_t PLLCFGR;      // PLL configuration register,       offset: 0x04
    __vo uint32_t CFGR;         // Clock configuration register,     offset: 0x08
    __vo uint32_t CIR;          // Clock interrupt register,         offset: 0x0C
    __vo uint32_t AHB1RSTR;     // AHB1 peripheral reset register,   offset: 0x10
    __vo uint32_t AHB2RSTR;     // AHB2 peripheral reset register,   offset: 0x14
    __vo uint32_t AHB3RSTR;     // AHB3 peripheral reset register,   offset: 0x18
    uint32_t      RESERVED0;    // Reserved,                         offset: 0x1C
    __vo uint32_t APB1RSTR;     // APB1 peripheral reset register,   offset: 0x20
    __vo uint32_t APB2RSTR;     // APB2 peripheral reset register,   offset: 0x24
    uint32_t      RESERVED1[2]; // Reserved,                         offset: 0x28-0x2C
    __vo uint32_t AHB1ENR;      // AHB1 peripheral clock enable,     offset: 0x30
    __vo uint32_t AHB2ENR;      // AHB2 peripheral clock enable,     offset: 0x34
    __vo uint32_t AHB3ENR;      // AHB3 peripheral clock enable,     offset: 0x38
    uint32_t      RESERVED2;    // Reserved,                         offset: 0x3C
    __vo uint32_t APB1ENR;      // APB1 peripheral clock enable,     offset: 0x40
    __vo uint32_t APB2ENR;      // APB2 peripheral clock enable,     offset: 0x44
    // ... more registers as needed
} RCC_RegDef_t;
```

**Note about Reserved fields:**
- Some addresses in the register map are reserved (not used)
- You MUST include them to maintain correct offsets
- Use `uint32_t RESERVED` (without `__vo`) for unused spaces

### 6.8 Create EXTI and SYSCFG Structures (for interrupts)

```c
/*
 * EXTI Register Definition Structure
 * Controls external interrupt behavior
 */

typedef struct
{
    __vo uint32_t IMR;      // Interrupt mask register,         offset: 0x00
    __vo uint32_t EMR;      // Event mask register,             offset: 0x04
    __vo uint32_t RTSR;     // Rising trigger selection,        offset: 0x08
    __vo uint32_t FTSR;     // Falling trigger selection,       offset: 0x0C
    __vo uint32_t SWIER;    // Software interrupt event,        offset: 0x10
    __vo uint32_t PR;       // Pending register,                offset: 0x14
} EXTI_RegDef_t;

/*
 * SYSCFG Register Definition Structure
 * Used to route GPIO pins to EXTI lines
 */

typedef struct
{
    __vo uint32_t MEMRMP;       // Memory remap register,        offset: 0x00
    __vo uint32_t PMC;          // Peripheral mode config,       offset: 0x04
    __vo uint32_t EXTICR[4];    // External interrupt config,    offset: 0x08-0x14
    uint32_t      RESERVED[2];  // Reserved,                     offset: 0x18-0x1C
    __vo uint32_t CMPCR;        // Compensation cell control,    offset: 0x20
} SYSCFG_RegDef_t;
```

### 6.9 Create Peripheral Pointer Macros

```c
/*
 * Peripheral Definitions
 * These macros typecast base addresses to register structure pointers
 */

#define GPIOA   ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB   ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC   ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD   ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE   ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF   ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG   ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH   ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC     ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI    ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG  ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
```

**How to use:**
```c
// Instead of this:
*((volatile uint32_t*)(0x40020000 + 0x14)) |= (1 << 5);

// You can write this:
GPIOA->ODR |= (1 << 5);  // Much more readable!
```

### 6.10 Create Clock Enable/Disable Macros

```c
/*
 * Clock Enable Macros for GPIO Peripherals
 * These set the appropriate bit in RCC_AHB1ENR register
 */

#define GPIOA_PCLK_EN()    (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()    (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()    (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()    (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()    (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()    (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()    (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()    (RCC->AHB1ENR |= (1 << 7))

// SYSCFG clock enable (needed for interrupts)
#define SYSCFG_PCLK_EN()   (RCC->APB2ENR |= (1 << 14))

/*
 * Clock Disable Macros
 */

#define GPIOA_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()    (RCC->AHB1ENR &= ~(1 << 1))
// ... and so on for other ports
```

### 6.11 Create Peripheral Reset Macros

```c
/*
 * Macros to Reset GPIO Peripherals
 * Set reset bit, then clear it (pulse reset)
 */

#define GPIOA_REG_RESET()  do{ (RCC->AHB1RSTR |= (1 << 0)); \
                               (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()  do{ (RCC->AHB1RSTR |= (1 << 1)); \
                               (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
// ... and so on
```

**Why `do{ }while(0)`?**
- Makes macro behave like a single statement
- Safe to use with if/else without braces
- Prevents subtle bugs

### 6.12 Create Helper Macros and Generic Definitions

```c
/*
 * IRQ Numbers for GPIO EXTI lines
 * Find these in the vector table section of reference manual
 */

#define IRQ_NO_EXTI0        6
#define IRQ_NO_EXTI1        7
#define IRQ_NO_EXTI2        8
#define IRQ_NO_EXTI3        9
#define IRQ_NO_EXTI4        10
#define IRQ_NO_EXTI9_5      23
#define IRQ_NO_EXTI15_10    40

/*
 * Generic Macros
 */

#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET

/*
 * Macro to convert GPIO port to code number (for SYSCFG_EXTICR)
 */

#define GPIO_BASEADDR_TO_CODE(x)  ( (x == GPIOA) ? 0 :\
                                    (x == GPIOB) ? 1 :\
                                    (x == GPIOC) ? 2 :\
                                    (x == GPIOD) ? 3 :\
                                    (x == GPIOE) ? 4 :\
                                    (x == GPIOF) ? 5 :\
                                    (x == GPIOG) ? 6 :\
                                    (x == GPIOH) ? 7 : 0 )

#endif /* STM32F446XX_H_ */
```

---

## 7. Step 2: Creating the GPIO Driver Header File

This file defines the user-facing API for the GPIO driver.

### 7.1 File Structure

```c
/*
 * stm32f446xx_gpio_driver.h
 * GPIO Driver Header File
 */

#ifndef STM32F446XX_GPIO_DRIVER_H_
#define STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"  // Include MCU header
```

### 7.2 Create Configuration Structure

This structure holds all configurable parameters for a GPIO pin.

```c
/*
 * GPIO Pin Configuration Structure
 * 
 * This structure is filled by the user to specify how a pin should be configured.
 * Each member represents one aspect of pin configuration.
 */

typedef struct
{
    uint8_t GPIO_PinNumber;         // Pin number (0-15)
    uint8_t GPIO_PinMode;           // Pin mode: input, output, alternate, analog, or interrupt
    uint8_t GPIO_PinSpeed;          // Output speed: low, medium, fast, high
    uint8_t GPIO_PinPuPdControl;    // Pull-up/Pull-down configuration
    uint8_t GPIO_PinOPType;         // Output type: push-pull or open-drain
    uint8_t GPIO_PinAltFunMode;     // Alternate function selection (0-15)
} GPIO_PinConfig_t;
```

### 7.3 Create Handle Structure

The handle structure combines the port pointer and configuration.

```c
/*
 * GPIO Handle Structure
 * 
 * This is the main structure used to work with a GPIO pin.
 * It contains:
 *   1. Pointer to the GPIO port (which port: A, B, C, etc.)
 *   2. Configuration settings for the pin
 */

typedef struct
{
    GPIO_RegDef_t *pGPIOx;           // Pointer to GPIO port base address
    GPIO_PinConfig_t GPIO_PinConfig; // Pin configuration settings
} GPIO_Handle_t;
```

### 7.4 Define Configuration Macros

```c
/*
 * @GPIO_PIN_NUMBERS
 * Macros for pin numbers
 */

#define GPIO_PIN_NO_0       0
#define GPIO_PIN_NO_1       1
#define GPIO_PIN_NO_2       2
#define GPIO_PIN_NO_3       3
#define GPIO_PIN_NO_4       4
#define GPIO_PIN_NO_5       5
#define GPIO_PIN_NO_6       6
#define GPIO_PIN_NO_7       7
#define GPIO_PIN_NO_8       8
#define GPIO_PIN_NO_9       9
#define GPIO_PIN_NO_10      10
#define GPIO_PIN_NO_11      11
#define GPIO_PIN_NO_12      12
#define GPIO_PIN_NO_13      13
#define GPIO_PIN_NO_14      14
#define GPIO_PIN_NO_15      15

/*
 * @GPIO_PIN_MODES
 * Macros for pin modes
 * 
 * Values 0-3: Match hardware register values (from reference manual)
 * Values 4-6: Software-defined for interrupt modes
 */

#define GPIO_MODE_IN        0   // Input mode
#define GPIO_MODE_OUT       1   // General purpose output mode
#define GPIO_MODE_ALTFN     2   // Alternate function mode
#define GPIO_MODE_ANALOG    3   // Analog mode
#define GPIO_MODE_IT_FT     4   // Interrupt, falling edge trigger
#define GPIO_MODE_IT_RT     5   // Interrupt, rising edge trigger
#define GPIO_MODE_IT_RFT    6   // Interrupt, rising & falling edge trigger

/*
 * @GPIO_PIN_OP_TYPES
 * Macros for output types
 */

#define GPIO_OP_TYPE_PP     0   // Push-pull
#define GPIO_OP_TYPE_OD     1   // Open-drain

/*
 * @GPIO_PIN_SPEEDS
 * Macros for output speeds
 */

#define GPIO_SPEED_LOW      0   // Low speed
#define GPIO_SPEED_MEDIUM   1   // Medium speed
#define GPIO_SPEED_FAST     2   // Fast speed
#define GPIO_SPEED_HIGH     3   // High speed

/*
 * @GPIO_PIN_PUPD
 * Macros for pull-up/pull-down configuration
 */

#define GPIO_NO_PUPD        0   // No pull-up, no pull-down
#define GPIO_PIN_PU         1   // Pull-up
#define GPIO_PIN_PD         2   // Pull-down
```

### 7.5 Declare API Function Prototypes

```c
/******************************************************************************************
 *                              APIs Supported by this Driver
 ******************************************************************************************/

/*
 * Peripheral Clock Setup
 * Enable or disable the clock for a GPIO port
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Initialization and De-initialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);    // Initialize GPIO pin
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);       // Reset GPIO port to default

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* STM32F446XX_GPIO_DRIVER_H_ */
```

---

## 8. Step 3: Implementing the GPIO Driver Source File

Now let's implement each API function.

### 8.1 Include Header and Start Implementation

```c
/*
 * stm32f446xx_gpio_driver.c
 * GPIO Driver Implementation
 */

#include "stm32f446xx_gpio_driver.h"
```

### 8.2 Implement Clock Control Function

```c
/*********************************************************************
 * @fn          GPIO_PeriClockControl
 *
 * @brief       Enables or disables peripheral clock for GPIO port
 *
 * @param[in]   pGPIOx: GPIO port base address (GPIOA, GPIOB, etc.)
 * @param[in]   EnorDi: ENABLE or DISABLE
 *
 * @return      None
 *
 * @note        This must be called before using any GPIO functions
 *********************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        // Enable clock for the appropriate port
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
    }
    else
    {
        // Disable clock for the appropriate port
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        // ... similar for other ports
    }
}
```

### 8.3 Implement GPIO Initialization Function

This is the most complex function. Let's break it down step by step.

```c
/*********************************************************************
 * @fn          GPIO_Init
 *
 * @brief       Initializes a GPIO pin according to the configuration
 *
 * @param[in]   pGPIOHandle: Pointer to GPIO handle structure
 *
 * @return      None
 *********************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0;
    
    // 0. Enable the peripheral clock (so we don't forget!)
    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

    // 1. Configure the mode of GPIO pin
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        /*
         * Non-interrupt mode
         * MODER register: 2 bits per pin
         * Pin 0: bits [1:0], Pin 1: bits [3:2], ..., Pin 15: bits [31:30]
         */
        
        // Calculate the bit position: pin_number * 2
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode 
                << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        
        // Clear the 2 bits first (important!)
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        
        // Set the new value
        pGPIOHandle->pGPIOx->MODER |= temp;
    }
    else
    {
        /*
         * Interrupt mode
         * Need to configure: EXTI, SYSCFG, and NVIC
         */
        
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            // Configure for Falling edge trigger
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // Make sure rising trigger is disabled
            EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            // Configure for Rising edge trigger
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // Make sure falling trigger is disabled
            EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            // Configure for both Rising and Falling edge trigger
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        /*
         * Configure GPIO port selection in SYSCFG_EXTICR
         * 
         * EXTICR[0] controls pins 0-3
         * EXTICR[1] controls pins 4-7
         * EXTICR[2] controls pins 8-11
         * EXTICR[3] controls pins 12-15
         * 
         * Each pin uses 4 bits in the register
         */
        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
        uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        
        SYSCFG_PCLK_EN();  // Enable SYSCFG clock
        SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

        // Enable the EXTI interrupt delivery using IMR (Interrupt Mask Register)
        EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }

    // 2. Configure the speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed 
            << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    // 3. Configure the pull-up/pull-down settings
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl 
            << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    // 4. Configure the output type
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType 
            << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    // 5. Configure the alternate functionality (if applicable)
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        /*
         * AFR[0] (AFRL) controls pins 0-7
         * AFR[1] (AFRH) controls pins 8-15
         * Each pin uses 4 bits
         */
        uint8_t temp1, temp2;

        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;  // Which AFR register
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;  // Position within register

        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));  // Clear 4 bits
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
    }
}
```

### 8.4 Implement GPIO De-initialization Function

```c
/*********************************************************************
 * @fn          GPIO_DeInit
 *
 * @brief       Resets GPIO port to default state
 *
 * @param[in]   pGPIOx: GPIO port base address
 *
 * @return      None
 *********************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    // ... continue for all ports
}
```

### 8.5 Implement Data Read Functions

```c
/*********************************************************************
 * @fn          GPIO_ReadFromInputPin
 *
 * @brief       Reads the value of a specific GPIO pin
 *
 * @param[in]   pGPIOx: GPIO port base address
 * @param[in]   PinNumber: Pin number (0-15)
 *
 * @return      0 or 1
 *********************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;
    
    /*
     * Read from IDR (Input Data Register)
     * Shift right to move the target bit to position 0
     * Mask with 0x01 to extract only that bit
     */
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    
    return value;
}

/*********************************************************************
 * @fn          GPIO_ReadFromInputPort
 *
 * @brief       Reads the entire GPIO port
 *
 * @param[in]   pGPIOx: GPIO port base address
 *
 * @return      16-bit value representing all pins
 *********************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;
    
    value = (uint16_t)pGPIOx->IDR;
    
    return value;
}
```

### 8.6 Implement Data Write Functions

```c
/*********************************************************************
 * @fn          GPIO_WriteToOutputPin
 *
 * @brief       Writes to a specific GPIO pin
 *
 * @param[in]   pGPIOx: GPIO port base address
 * @param[in]   PinNumber: Pin number (0-15)
 * @param[in]   Value: GPIO_PIN_SET or GPIO_PIN_RESET
 *
 * @return      None
 *********************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (Value == GPIO_PIN_SET)
    {
        // Set the bit (write 1)
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else
    {
        // Clear the bit (write 0)
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

/*********************************************************************
 * @fn          GPIO_WriteToOutputPort
 *
 * @brief       Writes to entire GPIO port
 *
 * @param[in]   pGPIOx: GPIO port base address
 * @param[in]   Value: 16-bit value to write
 *
 * @return      None
 *********************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

/*********************************************************************
 * @fn          GPIO_ToggleOutputPin
 *
 * @brief       Toggles a specific GPIO pin
 *
 * @param[in]   pGPIOx: GPIO port base address
 * @param[in]   PinNumber: Pin number (0-15)
 *
 * @return      None
 *********************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    /*
     * XOR operation: toggles the bit
     * If bit is 0, XOR with 1 makes it 1
     * If bit is 1, XOR with 1 makes it 0
     */
    pGPIOx->ODR ^= (1 << PinNumber);
}
```

### 8.7 Implement Interrupt Functions

```c
/*********************************************************************
 * @fn          GPIO_IRQInterruptConfig
 *
 * @brief       Enables or disables interrupt in NVIC
 *
 * @param[in]   IRQNumber: Interrupt number
 * @param[in]   EnorDi: ENABLE or DISABLE
 *
 * @return      None
 *********************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        /*
         * NVIC has multiple ISER (Interrupt Set Enable) registers
         * ISER0: IRQ 0-31
         * ISER1: IRQ 32-63
         * ISER2: IRQ 64-95
         */
        if (IRQNumber <= 31)
        {
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
    else
    {
        // Use ICER (Interrupt Clear Enable) registers to disable
        if (IRQNumber <= 31)
        {
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
        }
    }
}

/*********************************************************************
 * @fn          GPIO_IRQPriorityConfig
 *
 * @brief       Sets the priority of an interrupt
 *
 * @param[in]   IRQNumber: Interrupt number
 * @param[in]   IRQPriority: Priority value (0-15 for 4-bit priority)
 *
 * @return      None
 *********************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    /*
     * NVIC Priority Registers (IPR0-IPR59)
     * Each register contains 4 priority fields (8 bits each)
     * Only upper 4 bits are implemented (in STM32)
     */
    
    // Find which IPR register
    uint8_t iprx = IRQNumber / 4;
    
    // Find which section within the register (0, 1, 2, or 3)
    uint8_t iprx_section = IRQNumber % 4;
    
    // Calculate shift amount
    // Priority is in upper 4 bits of each 8-bit field
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*********************************************************************
 * @fn          GPIO_IRQHandling
 *
 * @brief       Clears the pending interrupt flag
 *
 * @param[in]   PinNumber: Pin number that triggered the interrupt
 *
 * @return      None
 *
 * @note        Must be called in the ISR to clear the interrupt
 *********************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
    /*
     * Clear the EXTI Pending Register (PR)
     * Writing 1 clears the pending bit
     */
    if (EXTI->PR & (1 << PinNumber))
    {
        EXTI->PR |= (1 << PinNumber);  // Clear by writing 1
    }
}
```

---

## 9. Testing Your Driver

### 9.1 Simple LED Blink Test

```c
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

void delay(uint32_t count)
{
    for (uint32_t i = 0; i < count; i++);
}

int main(void)
{
    GPIO_Handle_t GpioLed;
    
    // Configure the LED pin (PA5 on NUCLEO-F446RE)
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    
    // Initialize the pin
    GPIO_Init(&GpioLed);
    
    while (1)
    {
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
        delay(500000);
    }
    
    return 0;
}
```

### 9.2 Button Input Test

```c
int main(void)
{
    GPIO_Handle_t GpioLed, GpioBtn;
    
    // Configure LED
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_Init(&GpioLed);
    
    // Configure Button (PC13 on NUCLEO-F446RE)
    GpioBtn.pGPIOx = GPIOC;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_Init(&GpioBtn);
    
    while (1)
    {
        // Button is active LOW on NUCLEO board
        if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == 0)
        {
            delay(50000);  // Debounce
            GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
            while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == 0);  // Wait for release
        }
    }
}
```

### 9.3 Interrupt Test

```c
int main(void)
{
    GPIO_Handle_t GpioLed, GpioBtn;
    
    // Configure LED
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_Init(&GpioLed);
    
    // Configure Button with Interrupt
    GpioBtn.pGPIOx = GPIOC;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;  // Falling edge interrupt
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_Init(&GpioBtn);
    
    // Configure NVIC
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, 15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
    
    while (1)
    {
        // Main loop does nothing, interrupt handles everything
    }
}

// Interrupt Service Routine (must match name in vector table)
void EXTI15_10_IRQHandler(void)
{
    GPIO_IRQHandling(GPIO_PIN_NO_13);  // Clear interrupt
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}
```

---

## 10. Best Practices

### 10.1 Code Organization

```
project/
├── drivers/
│   ├── inc/                    # Header files
│   │   ├── stm32f446xx.h
│   │   └── stm32f446xx_gpio_driver.h
│   └── src/                    # Source files
│       └── stm32f446xx_gpio_driver.c
├── Src/                        # Application code
│   └── main.c
└── Startup/                    # Startup files
    └── startup_stm32f446xx.s
```

### 10.2 Naming Conventions

- **Macros**: ALL_CAPS_WITH_UNDERSCORES
- **Functions**: CamelCase_WithPrefix (e.g., `GPIO_Init`)
- **Types**: CamelCase_t suffix (e.g., `GPIO_Handle_t`)
- **Variables**: camelCase or snake_case

### 10.3 Documentation

Always document your functions:

```c
/*********************************************************************
 * @fn          FunctionName
 *
 * @brief       Brief description of what the function does
 *
 * @param[in]   paramName: Description of input parameter
 * @param[out]  paramName: Description of output parameter
 *
 * @return      Description of return value
 *
 * @note        Any important notes or considerations
 *********************************************************************/
```

### 10.4 Error Handling

Consider adding parameter validation:

```c
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    // Validate parameters
    if (pGPIOHandle == NULL) return;
    if (pGPIOHandle->pGPIOx == NULL) return;
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber > 15) return;
    
    // ... rest of implementation
}
```

### 10.5 Register Access Patterns

```c
// ALWAYS: Clear bits before setting
register &= ~(MASK << position);  // Clear
register |= (value << position);  // Set

// For single-bit toggle
register ^= (1 << position);

// For reading specific bits
value = (register >> position) & MASK;
```

---

## 11. Common Mistakes to Avoid

### 11.1 Forgetting to Enable Clock

```c
// WRONG - Will not work!
GPIOA->MODER |= (1 << 10);

// CORRECT - Enable clock first
RCC->AHB1ENR |= (1 << 0);  // or GPIOA_PCLK_EN();
GPIOA->MODER |= (1 << 10);
```

### 11.2 Not Clearing Bits Before Setting

```c
// WRONG - May give unexpected results
GPIOA->MODER |= (0x01 << 10);  // If bits were already set, won't work correctly

// CORRECT - Always clear first
GPIOA->MODER &= ~(0x03 << 10);  // Clear both bits
GPIOA->MODER |= (0x01 << 10);   // Then set
```

### 11.3 Wrong Bit Positions

```c
// WRONG - Off by one in 2-bit fields
temp = mode << (pinNumber);      // This works for 1-bit fields only

// CORRECT - For 2-bit fields
temp = mode << (2 * pinNumber);  // MODER, OSPEEDR, PUPDR
```

### 11.4 Forgetting volatile

```c
// WRONG - Compiler may optimize away reads
uint32_t *reg = (uint32_t*)0x40020014;

// CORRECT - Use volatile
volatile uint32_t *reg = (volatile uint32_t*)0x40020014;
```

### 11.5 Not Clearing Interrupt Pending Flag

```c
// WRONG - Interrupt will fire repeatedly
void EXTI0_IRQHandler(void)
{
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
    // Missing: Clear pending flag
}

// CORRECT - Always clear the pending flag
void EXTI0_IRQHandler(void)
{
    GPIO_IRQHandling(GPIO_PIN_NO_0);  // Clear pending flag first!
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}
```

---

## 12. Summary Checklist

Use this checklist when developing GPIO drivers for any MCU:

### Before Starting
- [ ] Obtain the reference manual for your MCU
- [ ] Find the memory map section
- [ ] Locate the GPIO chapter
- [ ] Find the RCC (clock control) chapter
- [ ] Identify interrupt-related chapters (EXTI, NVIC)

### MCU Header File
- [ ] Define base addresses (Flash, SRAM, Peripherals)
- [ ] Define bus base addresses (APB1, APB2, AHB1, etc.)
- [ ] Define peripheral base addresses
- [ ] Create register definition structures
- [ ] Create peripheral pointer macros
- [ ] Create clock enable/disable macros
- [ ] Create reset macros
- [ ] Define IRQ numbers
- [ ] Define generic macros (ENABLE, DISABLE, etc.)

### Driver Header File
- [ ] Include MCU header
- [ ] Create configuration structure
- [ ] Create handle structure
- [ ] Define configuration value macros
- [ ] Declare API function prototypes

### Driver Source File
- [ ] Implement clock control function
- [ ] Implement initialization function
- [ ] Implement de-initialization function
- [ ] Implement read functions (pin and port)
- [ ] Implement write functions (pin and port)
- [ ] Implement toggle function
- [ ] Implement interrupt configuration functions
- [ ] Implement interrupt handling function

### Testing
- [ ] Test basic output (LED blink)
- [ ] Test input reading (button)
- [ ] Test interrupts
- [ ] Test all GPIO modes
- [ ] Test alternate functions (if applicable)

---

## Conclusion

Congratulations! You now have the knowledge to write GPIO drivers for virtually any microcontroller. The key concepts transfer across different MCU families:

1. **Study the reference manual** - It's your primary source of truth
2. **Map registers to structures** - Makes code readable and maintainable
3. **Create abstraction layers** - Handle structures and configuration structures
4. **Implement standard APIs** - Init, Read, Write, Toggle, Interrupt handling
5. **Test thoroughly** - Start simple, add complexity gradually

Remember: The GPIO driver is often the first driver you write, and mastering it gives you the foundation to write drivers for more complex peripherals like SPI, I2C, UART, and DMA.

Happy coding! 🚀

---

*This guide was created as part of the STM32F446RE_Drivers project.*
*Reference: RM0390 - STM32F446xx Reference Manual*


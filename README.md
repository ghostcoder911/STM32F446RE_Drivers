# STM32F446RE Drivers

Bare-metal peripheral drivers for the STM32F446RE microcontroller, developed from scratch using the RM0390 reference manual.

## Overview

This project provides low-level drivers for the STM32F446RE ARM Cortex-M4 microcontroller. The drivers are written in C and provide a hardware abstraction layer for various peripherals.

## Supported Peripherals

| Peripheral | Status | Description |
|------------|--------|-------------|
| GPIO | ✅ Complete | General Purpose Input/Output |
| SPI | 🚧 In Progress | Serial Peripheral Interface |
| I2C | 📋 Planned | Inter-Integrated Circuit |
| USART | 📋 Planned | Universal Synchronous/Asynchronous Receiver/Transmitter |
| RCC | 📋 Planned | Reset and Clock Control |

## Project Structure

```
STM32F446RE_Drivers/
├── drivers/
│   ├── inc/                          # Driver header files
│   │   ├── stm32f446xx.h             # MCU specific header (register definitions)
│   │   ├── stm32f446xx_gpio_driver.h # GPIO driver header
│   │   └── stm32f446xx_spi_driver.h  # SPI driver header
│   └── src/                          # Driver source files
│       └── stm32f446xx_gpio_driver.c # GPIO driver implementation
├── Src/                              # Application source files
│   ├── main.c
│   ├── led_toggle.c                  # LED toggle example
│   ├── led_button.c                  # LED with button example
│   └── led_ext_button.c              # LED with external button example
├── Startup/
│   └── startup_stm32f446retx.s       # Startup assembly file
├── STM32F446RETX_FLASH.ld            # Flash linker script
├── STM32F446RETX_RAM.ld              # RAM linker script
└── README.md
```

## Hardware

- **MCU**: STM32F446RE (ARM Cortex-M4, 180 MHz)
- **Board**: NUCLEO-F446RE (or custom board)
- **Flash**: Up to 512 KB
- **SRAM**: 128 KB (SRAM1: 112 KB, SRAM2: 16 KB)

## GPIO Driver Features

### Supported Modes
- Input mode
- Output mode (Push-pull / Open-drain)
- Alternate function mode
- Analog mode
- Interrupt mode (Rising/Falling/Both edges)

### API Functions

```c
// Clock Control
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

// Initialization
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// Data Read/Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// Interrupt Handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);
```

## Usage Example

### LED Toggle

```c
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

int main(void)
{
    GPIO_Handle_t GpioLed;

    // Configure PA5 as output (on-board LED on NUCLEO-F446RE)
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&GpioLed);

    while(1)
    {
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
        for(uint32_t i = 0; i < 500000; i++); // Simple delay
    }

    return 0;
}
```

### Button with Interrupt

```c
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

int main(void)
{
    GPIO_Handle_t GpioBtn;

    // Configure PC13 as input with falling edge interrupt (user button)
    GpioBtn.pGPIOx = GPIOC;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOC, ENABLE);
    GPIO_Init(&GpioBtn);

    // Configure NVIC
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

    while(1);

    return 0;
}

void EXTI15_10_IRQHandler(void)
{
    GPIO_IRQHandling(GPIO_PIN_NO_13);
    // Your interrupt handling code here
}
```

## Building

This project is configured for STM32CubeIDE. To build:

1. Import the project into STM32CubeIDE
2. Select the appropriate build configuration
3. Build the project (Ctrl+B)

## Flashing

Use ST-Link debugger (built into NUCLEO boards) or any compatible programmer:

```bash
# Using ST-Link
st-flash write build/project.bin 0x08000000

# Using OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/project.elf verify reset exit"
```

## Reference Documents

- [RM0390 - STM32F446xx Reference Manual](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [STM32F446RE Datasheet](https://www.st.com/resource/en/datasheet/stm32f446re.pdf)
- [PM0214 - STM32 Cortex-M4 Programming Manual](https://www.st.com/resource/en/programming_manual/pm0214-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf)

## License

This project is open source and available for educational purposes.

## Author

ghostcoder911

---

*Developed as a learning project for embedded systems programming on STM32 microcontrollers.*


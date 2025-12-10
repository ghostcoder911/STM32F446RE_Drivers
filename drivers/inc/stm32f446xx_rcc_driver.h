/*
 * stm32f446xx_rcc_driver.h
 * RCC Driver for STM32F446xx MCU
 */

#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Returns the APB1 peripheral clock value
 */
uint32_t RCC_GetPCLK1Value(void);

/*
 * Returns the APB2 peripheral clock value
 */
uint32_t RCC_GetPCLK2Value(void);

/*
 * Returns the PLL output clock value
 */
uint32_t RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */

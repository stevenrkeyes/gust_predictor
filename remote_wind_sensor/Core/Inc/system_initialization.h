#ifndef __SYSTEM_INITIALIZATION_H
#define __SYSTEM_INITIALIZATION_H

#include "main.h"

void SystemClock_Config(void);
void GPIO_Init(void);
void ADC_Init(ADC_HandleTypeDef *hadc);
void I2C1_Init(I2C_HandleTypeDef *hi2c1);
void SPI1_Init(SPI_HandleTypeDef *hspi1);
void USART2_UART_Init(UART_HandleTypeDef *huart2);

#endif //__SYSTEM_INITIALIZATION_H

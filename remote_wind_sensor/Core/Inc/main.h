#ifndef __MAIN_H
#define __MAIN_H

#include "stm32l0xx_hal.h"

void Error_Handler(void);

#define VCP_TX_Pin            GPIO_PIN_2
#define VCP_TX_GPIO_Port      GPIOA
#define DAC_Pin               GPIO_PIN_1
#define DAC_GPIO_Port         GPIOB
#define Radio_NSS_Pin         GPIO_PIN_4
#define Radio_NSS_GPIO_Port   GPIOA
#define Radio_IRQ_Pin         GPIO_PIN_8
#define Radio_IRQ_GPIO_Port   GPIOA
#define Aux_Out_1_Pin         GPIO_PIN_11
#define Aux_Out_1_GPIO_Port   GPIOA
#define Aux_Out_2_Pin         GPIO_PIN_12
#define Aux_Out_2_GPIO_Port   GPIOA
#define TMS_Pin               GPIO_PIN_13
#define TMS_GPIO_Port         GPIOA
#define TCK_Pin               GPIO_PIN_14
#define TCK_GPIO_Port         GPIOA
#define VCP_RX_Pin            GPIO_PIN_15
#define VCP_RX_GPIO_Port      GPIOA
#define LD3_Pin               GPIO_PIN_3
#define LD3_GPIO_Port         GPIOB
#define LED_Other_1_Pin       GPIO_PIN_4
#define LED_Other_1_GPIO_Port GPIOB
#define LED_Other_2_Pin       GPIO_PIN_5
#define LED_Other_2_GPIO_Port GPIOB
#define Button_1_Pin          GPIO_PIN_6
#define Button_1_GPIO_Port    GPIOB
#define Button_2_Pin          GPIO_PIN_7
#define Button_2_GPIO_Port    GPIOB

#endif /* __MAIN_H */

#ifndef __BSP_X9CXXX_H
#define __BSP_X9CXXX_H

#include "stm32f4xx_hal.h"

// 引脚配置，可以配置称适合你的硬件
#define X9C_INC_GPIO_PORT    GPIOA
#define X9C_INC_PIN         GPIO_PIN_0
#define X9C_UD_GPIO_PORT    GPIOA
#define X9C_UD_PIN          GPIO_PIN_1
#define X9C_CS_GPIO_PORT    GPIOA
#define X9C_CS_PIN          GPIO_PIN_2

// ---- 基本GPIO操作 ----
#define X9C_INC_HIGH()    HAL_GPIO_WritePin(X9C_INC_GPIO_PORT, X9C_INC_PIN, GPIO_PIN_SET)
#define X9C_INC_LOW()     HAL_GPIO_WritePin(X9C_INC_GPIO_PORT, X9C_INC_PIN, GPIO_PIN_RESET)
#define X9C_UD_HIGH()     HAL_GPIO_WritePin(X9C_UD_GPIO_PORT, X9C_UD_PIN, GPIO_PIN_SET)
#define X9C_UD_LOW()      HAL_GPIO_WritePin(X9C_UD_GPIO_PORT, X9C_UD_PIN, GPIO_PIN_RESET)
#define X9C_CS_HIGH()     HAL_GPIO_WritePin(X9C_CS_GPIO_PORT, X9C_CS_PIN, GPIO_PIN_SET)
#define X9C_CS_LOW()      HAL_GPIO_WritePin(X9C_CS_GPIO_PORT, X9C_CS_PIN, GPIO_PIN_RESET)


#endif /* __BSP_X9CXXX_H */
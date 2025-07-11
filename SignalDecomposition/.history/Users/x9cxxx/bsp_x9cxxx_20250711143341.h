#ifndef __BSP_X9CXXX_H
#define __BSP_X9CXXX_H

#include "stm32f4xx_hal.h"

// 假设用 GPIOA 的 0/1/2 引脚
#define X9C_INC_GPIO_PORT    GPIOA
#define X9C_INC_PIN         GPIO_PIN_0
#define X9C_UD_GPIO_PORT    GPIOA
#define X9C_UD_PIN          GPIO_PIN_1
#define X9C_CS_GPIO_PORT    GPIOA
#define X9C_CS_PIN          GPIO_PIN_2


#endif /* __BSP_X9CXXX_H */
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


#endif /* __BSP_X9CXXX_H */
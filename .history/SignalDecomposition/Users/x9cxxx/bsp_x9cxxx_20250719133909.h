/*
 * @Author: yyf 17786321727@163.com
 * @Date: 2025-07-14 10:41:49
 * @LastEditors: yyf 17786321727@163.com
 * @LastEditTime: 2025-07-19 13:38:18
 * @FilePath: /code/SignalDecomposition/Users/x9cxxx/bsp_x9cxxx.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __BSP_X9CXXX_H
#define __BSP_X9CXXX_H

#include "stm32f4xx_hal.h"
#include "core_cm4.h"

// 引脚配置，可以配置称适合你的硬件
#define X9C_INC_GPIO_PORT    GPIOA
#define X9C_INC_PIN         GPIO_PIN_10
#define X9C_UD_GPIO_PORT    GPIOA
#define X9C_UD_PIN          GPIO_PIN_12
#define X9C_CS_GPIO_PORT    GPIOA
#define X9C_CS_PIN          GPIO_PIN_14



// ---- 基本GPIO操作 ----
#define X9C_INC_HIGH()    HAL_GPIO_WritePin(X9C_INC_GPIO_PORT, X9C_INC_PIN, GPIO_PIN_SET)
#define X9C_INC_LOW()     HAL_GPIO_WritePin(X9C_INC_GPIO_PORT, X9C_INC_PIN, GPIO_PIN_RESET)
#define X9C_UD_HIGH()     HAL_GPIO_WritePin(X9C_UD_GPIO_PORT, X9C_UD_PIN, GPIO_PIN_SET)
#define X9C_UD_LOW()      HAL_GPIO_WritePin(X9C_UD_GPIO_PORT, X9C_UD_PIN, GPIO_PIN_RESET)
#define X9C_CS_HIGH()     HAL_GPIO_WritePin(X9C_CS_GPIO_PORT, X9C_CS_PIN, GPIO_PIN_SET)
#define X9C_CS_LOW()      HAL_GPIO_WritePin(X9C_CS_GPIO_PORT, X9C_CS_PIN, GPIO_PIN_RESET)

// ---- 芯片型号选择 ----
#define X9C103    0
#define X9C503    1

#define X9C503_TOTAL_RESISTANCE 50000.0f   // 50kΩ
#define X9C503_STEPS            99

#define X9C103_TOTAL_RESISTANCE 10000.0f  // 10kΩ
#define X9C103_STEPS            99

// 相关函数声明
void X9C_Init_103(void);
void X9C103_SetResistance_103(float resistance);
void X9C503_SetResistance_103(float resistance);

void X9C_Init_103(void);
void X9C103_SetResistance_103(float resistance);
void X9C503_SetResistance_103(float resistance);
#endif /* __BSP_X9CXXX_H */

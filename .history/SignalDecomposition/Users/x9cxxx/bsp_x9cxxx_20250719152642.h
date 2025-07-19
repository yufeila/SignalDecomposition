/*
 * @Author: yyf 17786321727@163.com
 * @Date: 2025-07-14 10:41:49
 * @LastEditors: yyf 17786321727@163.com
 * @LastEditTime: 2025-07-19 15:25:51
 * @FilePath: /code/SignalDecomposition/Users/x9cxxx/bsp_x9cxxx.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __BSP_X9CXXX_H
#define __BSP_X9CXXX_H

#include "stm32f4xx_hal.h"
#include "core_cm4.h"
 
// 引脚配置，可以配置称适合你的硬件
#define X9C103_INC_GPIO_PORT    GPIOG
#define X9C103_INC_PIN         GPIO_PIN_2
#define X9C103_UD_GPIO_PORT    GPIOG
#define X9C103_UD_PIN          GPIO_PIN_4
#define X9C103_CS_GPIO_PORT    GPIOC
#define X9C103_CS_PIN          GPIO_PIN_10

#define X9C503_INC_GPIO_PORT    GPIOA
#define X9C503_INC_PIN         GPIO_PIN_11
#define X9C503_UD_GPIO_PORT    GPIOA
#define X9C503_UD_PIN          GPIO_PIN_13
#define X9C503_CS_GPIO_PORT    GPIOA
#define X9C503_CS_PIN          GPIO_PIN_15



// ---- 基本GPIO操作 ----
#define X9C103_INC_HIGH()    HAL_GPIO_WritePin(X9C103_INC_GPIO_PORT, X9C103_INC_PIN, GPIO_PIN_SET)
#define X9C103_INC_LOW()     HAL_GPIO_WritePin(X9C103_INC_GPIO_PORT, X9C103_INC_PIN, GPIO_PIN_RESET)
#define X9C103_UD_HIGH()     HAL_GPIO_WritePin(X9C103_UD_GPIO_PORT, X9C103_UD_PIN, GPIO_PIN_SET)
#define X9C103_UD_LOW()      HAL_GPIO_WritePin(X9C103_UD_GPIO_PORT, X9C103_UD_PIN, GPIO_PIN_RESET)
#define X9C103_CS_HIGH()     HAL_GPIO_WritePin(X9C103_CS_GPIO_PORT, X9C103_CS_PIN, GPIO_PIN_SET)
#define X9C103_CS_LOW()      HAL_GPIO_WritePin(X9C103_CS_GPIO_PORT, X9C103_CS_PIN, GPIO_PIN_RESET)

#define X9C503_INC_HIGH()    HAL_GPIO_WritePin(X9C503_INC_GPIO_PORT, X9C503_INC_PIN, GPIO_PIN_SET)
#define X9C503_INC_LOW()     HAL_GPIO_WritePin(X9C503_INC_GPIO_PORT, X9C503_INC_PIN, GPIO_PIN_RESET)
#define X9C503_UD_HIGH()     HAL_GPIO_WritePin(X9C503_UD_GPIO_PORT, X9C503_UD_PIN, GPIO_PIN_SET)
#define X9C503_UD_LOW()      HAL_GPIO_WritePin(X9C503_UD_GPIO_PORT, X9C503_UD_PIN, GPIO_PIN_RESET)
#define X9C503_CS_HIGH()     HAL_GPIO_WritePin(X9C503_CS_GPIO_PORT, X9C503_CS_PIN, GPIO_PIN_SET)
#define X9C503_CS_LOW()      HAL_GPIO_WritePin(X9C503_CS_GPIO_PORT, X9C503_CS_PIN, GPIO_PIN_RESET)

// ---- 芯片型号选择 ----
#define X9C103    0
#define X9C503    1

#define X9C503_TOTAL_RESISTANCE 50000.0f   // 50kΩ
#define X9C503_STEPS            99

#define X9C103_TOTAL_RESISTANCE 10000.0f  // 10kΩ
#define X9C103_STEPS            99

// 相关函数声明
void DWT_Delay_Init(void);
void DWT_Delay_us(uint32_t us);

// X9C103相关函数
void X9C103_Init(void);
void X9C103_Select(void);
void X9C103_Deselect(void);
void X9C103_SetDirection(uint8_t up);
void X9C103_IncPulse(void);
void X9C103_SetPos(uint8_t pos);
void X9C103_Store(void);
void X9C103_SetPos_Store(uint8_t pos);
void X9C103_SetResistance(float resistance);

// X9C503相关函数
void X9C503_Init(void);
void X9C503_Select(void);
void X9C503_Deselect(void);
void X9C503_SetDirection(uint8_t up);
void X9C503_IncPulse(void);
void X9C503_SetPos(uint8_t pos);
void X9C503_Store(void);
void X9C503_SetPos_Store(uint8_t pos);
void X9C503_SetResistance(float resistance);

// 统一初始化函数
void X9C_Init(void);

#endif /* __BSP_X9CXXX_H */

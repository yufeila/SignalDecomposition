/*
 * @Author: yyf 17786321727@163.com
 * @Date: 2025-07-12 15:12:50
 * @LastEditors: yyf 17786321727@163.com
 * @LastEditTime: 2025-07-13 15:34:57
 * @FilePath: /code/SignalDecomposition/Users/key/key.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __KEY_H
#define __KEY_H

#include "main.h"
#include "stm32f4xx_hal.h"

#define KEY0_PIN GPIO_PIN_8
#define KEY0_PORT GPIOB

#define KEY1_PIN GPIO_PIN_9
#define KEY1_PORT GPIOB

#define KEY_DEBOUNCE_DELAY 10 // ms

extern uint8_t signal_decomposition_flag;

#endif /* __KEY_H */

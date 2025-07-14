#ifndef __KEY_H
#define __KEY_H

#include "main.h"
#include "stm32f4xx_hal.h"

#define KEY0_PIN GPIO_PIN_8
#define KEY0_PORT GPIOB

#define KEY1_PIN GPIO_PIN_9
#define KEY1_PORT GPIOB

#define KEY_DEBOUNCE_DELAY 10 // ms

extern volatile uint8_t signal_decomposition_flag;
void Detect_KeyPress(void);

#endif /* __KEY_H */

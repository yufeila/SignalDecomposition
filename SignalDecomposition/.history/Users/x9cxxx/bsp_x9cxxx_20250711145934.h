#ifndef __BSP_X9CXXX_H
#define __BSP_X9CXXX_H

#include "stm32f4xx_hal.h"
#include "core_cm4.h"

// �������ã��������ó��ʺ����Ӳ��
#define X9C_INC_GPIO_PORT    GPIOA
#define X9C_INC_PIN         GPIO_PIN_0
#define X9C_UD_GPIO_PORT    GPIOA
#define X9C_UD_PIN          GPIO_PIN_1
#define X9C_CS_GPIO_PORT    GPIOA
#define X9C_CS_PIN          GPIO_PIN_2

// ---- ����GPIO���� ----
#define X9C_INC_HIGH()    HAL_GPIO_WritePin(X9C_INC_GPIO_PORT, X9C_INC_PIN, GPIO_PIN_SET)
#define X9C_INC_LOW()     HAL_GPIO_WritePin(X9C_INC_GPIO_PORT, X9C_INC_PIN, GPIO_PIN_RESET)
#define X9C_UD_HIGH()     HAL_GPIO_WritePin(X9C_UD_GPIO_PORT, X9C_UD_PIN, GPIO_PIN_SET)
#define X9C_UD_LOW()      HAL_GPIO_WritePin(X9C_UD_GPIO_PORT, X9C_UD_PIN, GPIO_PIN_RESET)
#define X9C_CS_HIGH()     HAL_GPIO_WritePin(X9C_CS_GPIO_PORT, X9C_CS_PIN, GPIO_PIN_SET)
#define X9C_CS_LOW()      HAL_GPIO_WritePin(X9C_CS_GPIO_PORT, X9C_CS_PIN, GPIO_PIN_RESET)

// ---- оƬ�ͺ�ѡ�� ----
#define X9C103    0
#define X9C503    1

#define X9C503_TOTAL_RESISTANCE 50000.0f   // 50k��
#define X9C503_STEPS            99

#define X9C103_TOTAL_RESISTANCE 10000.0f  // 10k��
#define X9C103_STEPS            99

// X9C103��غ�������
void X9C103_SetResistance(float resistance);

#endif /* __BSP_X9CXXX_H */
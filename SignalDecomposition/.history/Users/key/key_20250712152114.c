#include "./key/key.h"

volitate uint8_t signal_deconposition_flag = 0; // 信号分解标志

void Detect_KeyPress(void)
{
    // 检测按键0
    if (HAL_GPIO_ReadPin(KEY0_PORT, KEY0_PIN) == GPIO_PIN_RESET) {
        // 按键0被按下
        signal_deconposition_flag = 1; // 设置信号分解标志
    }

    // 检测按键1
    if (HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN) == GPIO_PIN_RESET) {
        // 按键1被按下
        signal_deconposition_flag = 2; // 设置信号分解标志
    }
}
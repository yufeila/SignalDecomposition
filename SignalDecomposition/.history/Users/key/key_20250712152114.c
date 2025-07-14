#include "./key/key.h"

volitate uint8_t signal_deconposition_flag = 0; // �źŷֽ��־

void Detect_KeyPress(void)
{
    // ��ⰴ��0
    if (HAL_GPIO_ReadPin(KEY0_PORT, KEY0_PIN) == GPIO_PIN_RESET) {
        // ����0������
        signal_deconposition_flag = 1; // �����źŷֽ��־
    }

    // ��ⰴ��1
    if (HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN) == GPIO_PIN_RESET) {
        // ����1������
        signal_deconposition_flag = 2; // �����źŷֽ��־
    }
}
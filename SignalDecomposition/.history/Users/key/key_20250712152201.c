#include "./key/key.h"

volitate uint8_t signal_deconposition_flag = 0; // �źŷֽ��־

void Detect_KeyPress(void)
{

    uint8_t key0_state = HAL_GPIO_ReadPin(KEY0_PORT, KEY0_PIN);

    static uint32_t last_tick0 = 0;
    

}
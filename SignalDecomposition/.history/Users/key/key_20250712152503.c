#include "./key/key.h"

volatile uint8_t signal_deconposition_flag = 0; // 信号分解标志
static uint8_t key0_last_state = 1; // 默认按键未按下

void Detect_KeyPress(void)
{

    uint8_t key0_state = HAL_GPIO_ReadPin(KEY0_PORT, KEY0_PIN);

    static uint32_t last_tick0 = 0;

    // ---- KEY0 检测与消抖 ----
    if (key0_state == GPIO_PIN_RESET && key0_last_state == GPIO_PIN_SET) // 按下
    {
        if ((HAL_GetTick() - last_tick0) > KEY_DEBOUNCE_DELAY)
        {
            last_tick0 = HAL_GetTick();

                // 在其他模式下，KEY0切换到基本测量模式
                basic_measurement_flag = 1;
                sweep_freq_response_flag = 0;
                measure_r_out_flag = 0;
                current_system_state = BASIC_MEASUREMENT_STATE;
            }
        }
    }
    key0_last_state = key0_state;


}
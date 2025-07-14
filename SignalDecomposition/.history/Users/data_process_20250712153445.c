#include "data_process.h"

void SignalOutputAndDisplay(void)
{
    // 检测按键状态
    Detect_KeyPress();

    // 如果信号分解标志被设置，执行信号分解处理
    if (signal_deconposition_flag)
    {
        signal_deconposition_flag = 0; // 重置标志

        Data_Process();

        Update_LCD_Display();

        DDS_Output();
    }
}

void Data_Process(void)
{
    // 1. 启动本轮ADC+DMA+TIM采集（彻底修复启动时序）
    ADC_BufferReadyFlag = BUFFER_READY_FLAG_NONE;
        
}
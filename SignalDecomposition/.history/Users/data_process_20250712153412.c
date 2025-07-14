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
    // 读取ADC数据
    adc_zero_bias(adc_buffer, adc_zeroed, BUF_SIZE);

    // 执行FFT高通估计
    float f_est, A_est;
    fft_hann_zero_interp(adc_zeroed, &f_est, &A_est);

    // 更新AD9833输出频率和幅值
    AD9833_1_SetFrequency(0, f_est, AD9833_SINE_WAVE);
    AD9833_1_SetAmplitude(A_est);
}
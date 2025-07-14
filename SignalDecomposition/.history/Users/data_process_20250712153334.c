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

        Updata

        // 执行FFT和插值处理
        float f_est, A_est;
        fft_hann_zero_interp(adc_zeroed, &f_est, &A_est);

        // 显示结果到LCD
        char display_buf[50];
        sprintf(display_buf, "Freq: %.2f Hz, Amp: %.2f V", f_est, A_est * ADC_LSB_VOLT);
        LCD_DisplayString(display_buf);
    }
}
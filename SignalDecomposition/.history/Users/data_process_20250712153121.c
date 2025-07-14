#include "data_process.h"

void SignalOutputAndDisplay(void)
{
    // ��ⰴ��״̬
    Detect_KeyPress();

    // ����źŷֽ��־�����ã�ִ���źŷֽ⴦��
    if (signal_deconposition_flag)
    {
        signal_deconposition_flag = 0; // ���ñ�־

        // ��ȡADC���ݲ�������ƫ��У��
        float adc_zeroed[BUF_SIZE];
        adc_zero_bias(adc_buffer, adc_zeroed, BUF_SIZE);

        // ִ��FFT�Ͳ�ֵ����
        float f_est, A_est;
        fft_hann_zero_interp(adc_zeroed, &f_est, &A_est);

        // ��ʾ�����LCD
        char display_buf[50];
        sprintf(display_buf, "Freq: %.2f Hz, Amp: %.2f V", f_est, A_est * ADC_LSB_VOLT);
        LCD_DisplayString(display_buf);
    }
}
#include "data_process.h"

void SignalOutputAndDisplay(void)
{
    // ��ⰴ��״̬
    Detect_KeyPress();

    // ����źŷֽ��־�����ã�ִ���źŷֽ⴦��
    if (signal_deconposition_flag)
    {
        signal_deconposition_flag = 0; // ���ñ�־

        Data_Process();

        Update_LCD_Display();

        DDS_Output();
    }
}

void Data_Process(void)
{
    // ��ȡADC����
    adc_zero_bias(adc_buffer, adc_zeroed, BUF_SIZE);

    // ִ��FFT��ͨ����
    float f_est, A_est;
    fft_hann_zero_interp(adc_zeroed, &f_est, &A_est);

    // ����AD9833���Ƶ�ʺͷ�ֵ
    AD9833_1_SetFrequency(0, f_est, AD9833_SINE_WAVE);
    AD9833_1_SetAmplitude(A_est);
}
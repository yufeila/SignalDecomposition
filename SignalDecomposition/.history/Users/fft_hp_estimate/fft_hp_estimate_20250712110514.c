#include "fft_hp_estimate.h"

static float in_buf[N_FFT];   /* N_RAW ���� + �������� */
static float mag[N_FFT/2];    /* ��ֵƽ����ʵ����      */

// �ڶ��ź��� FFT �任ǰ��ͨ�����ÿ����������� hann(n, N) �Ľ�����Լ���Ƶ��й©�����Ƶ�׷�����׼ȷ�ԡ�
static inline float hann(float n, float N)
{
    return 0.5f * (1.0f - arm_cos_f32(2.0f * PI * n / N));
}

/* Hann ���� **coherent gain** = 0.5�����ڷ�ֵ��ԭ */
#define HANN_CG   0.5f

/*------------------------------------------------------------------
 * ���룺adc[] (int16_t * N_RAW)  ���� �ѹ�һ�� [-1,1) ��ԭʼ����
 * �����*f_est   ����Ƶ�� (Hz)
 *       *A_est   ������ֵ (��ֵ������ rms)
 *----------------------------------------------------------------*/
void fft_hann_zero_interp(const float *adc, float *f_est, float *A_est)
{
    /* 1) �� Hann �� + ���� + ���� */
    for (uint32_t n = 0; n < N_RAW; n++)
        in_buf[n] = adc[n] * hann(n, N_RAW-1);
    for (uint32_t n = N_RAW; n < N_FFT; n++)    // ����
        in_buf[n] = 0.0f;

    /* 2) ʵ FFT (CMSIS) */
    arm_rfft_fast_instance_f32 cfg;                
    arm_rfft_fast_init_f32(&cfg, N_FFT);            // ��ʼ�� FFT ����
    arm_rfft_fast_f32(&cfg, in_buf, in_buf, 0);     //  ��һ��in_buf��Ϊ�������,
                                                    //  �ڶ���in_buf��Ϊ�������, 0��ʾFFT���任

    /* 3) ��ֵƽ��������� bin  (���� DC) */
    float Pmax = 0.0f; uint32_t kmax = 1;
    for (uint32_t k = 1; k < N_FFT/2; k++)
    {
        float re = in_buf[2*k];
        float im = in_buf[2*k+1];
        float P  = re*re + im*im;
        mag[k] = P;
        if (P > Pmax)  { Pmax = P; kmax = k; }
    }

    /* 4) �����߲�ֵ�����㣩 */
    uint32_t km1 = (kmax==0) ? kmax : kmax-1;   // kmac����ߵ�
    uint32_t kp1 = (kmax==N_FFT/2-1) ? kmax : kmax+1;   // kmax���ұߵ� 

    float alpha = mag[km1];
    float beta  = mag[kmax];
    float gamma = mag[kp1];

    float delta = 0.5f*(alpha - gamma) / (alpha - 2.0f*beta + gamma);
    float k_ref = (float)kmax + delta;          // ���徫ȷλ��

    /* 5) Ƶ�� & ��ֵ�����㲻Ӱ���ֵ��ֻ����Դ����棩 */
    *f_est = k_ref * FS_HZ / N_FFT;

    /* �ò�ֵ������ֵ��|X(k)| = sqrt(beta - 0.25*(��-��)*��) */
    float pk_corr = beta - 0.25f*(alpha - gamma)*delta;
    *A_est = 2.0f * sqrtf(pk_corr) / (N_RAW * HANN_CG);   /* ��2 ȡ��-�� �� ��ֵ */
}

/* �� ADC ԭʼ���� (volatile uint16_t ����) ��ȥ��ֵ�������� 0 ƫ�ã���һ���� [-1, 1)������ float ָ�� */
void adc_zero_bias(const uint16_t *adc_raw, float* adc_zeroed, uint32_t len)
{

    float sum = 0.0f;
    for (uint32_t i = 0; i < len; i++)
        sum += (float)adc_raw[i];
    float mean = sum / (float)len;
    
    for (uint32_t i = 0; i < len; i++)
        adc_zeroed[i] = ((float)adc_raw[i] - mean) * ADC_LSB_VOLT; // ���� ADC ������ΧΪ 16 λ
    return buf;
}
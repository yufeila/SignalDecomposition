#include "./fft_hp_estimate/fft_hp_estimate.h"
#include "usart.h"
#include <stdio.h>

static float      win[N_RAW];          // Hann(n)
static float      buf[N_FFT];          // FFT ���� & �������
static float      mag[N_FFT/2];        // ���߹����ף�P = |X|^2��
static uint8_t    win_ready = 0;
static uint8_t    fft_ready = 0;
static arm_rfft_fast_instance_f32 cfg;


static inline void init_hann_fft(void)
{
    if (!win_ready) {
        for (uint32_t n = 0; n < N_RAW; ++n)
            win[n] = 0.5f * (1.0f - cosf(2.0f * PI_F * n / (N_RAW-1)));
        win_ready = 1;
    }
    if (!fft_ready) {
        arm_rfft_fast_init_f32(&cfg, N_FFT);
        fft_ready = 1;
    }
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

/**
 * @brief ��ADCԭʼ���ݽ���ȥ��ƫ����
 *
 * �˺�����������ADCԭʼ���ݵľ�ֵ������ÿ������ֵ��ȥ��ֵ�����ADC�ĵ�ѹ�ֱ��ʣ�ADC_LSB_VOLT����
 * �õ�ȥ��ƫ��ĵ�ѹֵ������洢��adc_zeroed�����С�
 *
 * @param[in]  adc_raw     ADCԭʼ��������ָ��
 * @param[out] adc_zeroed  ȥ��ƫ��ĵ�ѹֵ����ָ��
 * @param[in]  len         ���ݳ���
 */
void adc_zero_bias(const uint16_t *adc_raw, float* adc_zeroed, uint32_t len)
{
    float sum = 0.0f;
    for (uint32_t i = 0; i < len; i++)
        sum += (float)adc_raw[i];
    float mean = sum / (float)len;
    
    for (uint32_t i = 0; i < len; i++)
        adc_zeroed[i] = ((float)adc_raw[i] - mean) * ADC_LSB_VOLT;
}

/**
 * @brief �������źŽ���FFT����������ǰ����Ƶ�ʷ��������ֵ��������+�����+�����߲�ֵ����
 *
 * �˺����������ADC�������ݽ������´���
 * 1. ���Ժ���������������䣻
 * 2. ����ʵ�����ٸ���Ҷ�任��FFT����
 * 3. ����ÿ��Ƶ��bin�ķ���ƽ����Ѱ�ҷ�ֵ��������bin��
 *    - ��������������bin����Խ�С������Ѱ�ң�
 * 4. ��ÿ������������߲�ֵ����ȷ����Ƶ�ʺͷ�ֵ��
 *
 * @param[in]  adc     �����ADC�����������飬����ΪN_RAW
 * @param[out] f1_est  ��һ��Ƶ�ʷ����Ĺ���Ƶ�ʣ�Hz��
 * @param[out] A1_est  ��һ��Ƶ�ʷ����Ĺ����ֵ
 * @param[out] f2_est  �ڶ���Ƶ�ʷ����Ĺ���Ƶ�ʣ�Hz��
 * @param[out] A2_est  �ڶ���Ƶ�ʷ����Ĺ����ֵ
 *
 * @note
 * - ʹ�ú���������Ƶ��й©��HANN_CGΪ����У��ϵ����
 * - Ƶ�ʹ�����������߲�ֵ��߾��ȣ�
 * - �豣֤��������adc����ΪN_RAW�����ָ����Ч��
 */
void fft_top2_hann_zero_interp(const float *adc,
                               float *f1_est, float *A1_est,
                               float *f2_est, float *A2_est)
{
    /* ---------- ��ɺ�����ͬ��1) �Ӵ� + 0 ����2) ʵ FFT ---------- */
    for (uint32_t n = 0; n < N_RAW; n++)
        in_buf[n] = adc[n] * hann(n, N_RAW-1);
    for (uint32_t n = N_RAW; n < N_FFT; n++)
        in_buf[n] = 0.0f;

	static arm_rfft_fast_instance_f32 cfg;
	if(fft_inited)
	{	
		fft_inited = 0;
		arm_rfft_fast_init_f32(&cfg, N_FFT);
	}

    arm_rfft_fast_f32(&cfg, in_buf, in_buf, 0);

    /* ---------- 3) ����ƽ�����ҵ�һ���ڶ��� bin ---------- */
    float P1 = 0.0f, P2 = 0.0f;
    uint32_t k1 = 1, k2 = 1;

    for (uint32_t k = 1; k < N_FFT/2; k++)
    {
        float re = in_buf[2*k];
        float im = in_buf[2*k+1];
        float P  = re*re + im*im;
        mag[k] = P;

        if (P > P1) {                 /* ���µ�һ����˳�Ƶڶ��� */
            P2 = P1;  k2 = k1;
            P1 = P;   k1 = k;
        } else if (P > P2) {          /* ֻ���µڶ��� */
            P2 = P;   k2 = k;
        }
    }

    /* �������������� bin���ɸ�����Ҫ�ټ�һ���ų��� */
    if (fabsf((int32_t)k1 - (int32_t)k2) < 2) {
        /* ���������ѽ�С��������Ѱ��һ���·� */
        P2 = 0.0f; k2 = 1;
        for (uint32_t k = 1; k < N_FFT/2; k++) {
            if (k==k1) continue;
            if (mag[k] > P2) { P2 = mag[k]; k2 = k; }
        }
    }

    /* ---------- 4) ��ÿ�����������߲�ֵ ---------- */
    /* --- helper �� --- */
   /* --- helper �� --- */
    #define PARABOLA_INTERP(_k, _f_out, _A_out)                    \
    do {                                                           \
        uint32_t km1 = (_k==0)?_k:_k-1, kp1 = (_k==N_FFT/2-1)?_k:_k+1; \
        float a = logf(mag[km1]), b = logf(mag[_k]), c = logf(mag[kp1]);\
        float delta = 0.5f * (c - a) / (2.0f * b - a - c);        \
        float k_ref = (float)_k + delta;                           \
        *(_f_out) = k_ref * FS_HZ / N_FFT;                         \
        float pk_corr = expf(b - 0.25f * (c - a) * delta);         \
        *(_A_out) = 2.0f * sqrtf(pk_corr) / (N_RAW * HANN_CG);     \
		printf("k = %lu delta = %f   f = %f\r\n", k1, delta , *f1_est);\
    } while(0)

    PARABOLA_INTERP(k1, f1_est, A1_est);
    PARABOLA_INTERP(k2, f2_est, A2_est);
	
	
}

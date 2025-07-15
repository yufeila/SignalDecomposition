#include "./fft_hp_estimate/fft_hp_estimate.h"
#include "usart.h"
#include <stdio.h>

static float      win[N_RAW];          // Hann(n)
static float      buf[N_FFT];          // FFT ���� & �������
static float      mag[N_FFT/2];        // ���߹����ף�P = |X|^2��
static uint8_t    win_ready = 0;
static uint8_t    fft_ready = 0;
static arm_rfft_fast_instance_f32 cfg;

#define PI_F 3.1415926535

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


void fft_top2_hann_zero_interp(const float *adc,
                               float *f1_est, float *A1_est,
                               float *f2_est, float *A2_est)
{
    init_hann_fft();                          /* ������ */

    /* ---------- 1) �� Hann �� + 0 �� ---------- */
    for (uint32_t n = 0; n < N_RAW; ++n)
        buf[n] = adc[n] * win[n];
    memset(&buf[N_RAW], 0, (N_FFT - N_RAW) * sizeof(float));

    /* ---------- 2) ʵ�� FFT ---------- */
    arm_rfft_fast_f32(&cfg, buf, buf, 0);     /* ԭ�ظ��� */

    /* ---------- 3) ���߹����� & �� top-2 ---------- */
    float P1 = -1.0f, P2 = -1.0f;
    uint32_t k1 = 1,   k2 = 1;

    for (uint32_t k = 1; k < N_FFT/2; ++k) {  /* ���� DC */
        float re = buf[2*k];                  /* CMSIS: ʵ����ż���±�=2k */
        float im = buf[2*k+1];                /*          �鲿���棬�±�=2k+1 */
        float P  = re*re + im*im;             /* ����ƽ�� = ������ */
        mag[k] = P;                           /* ����������ں����ֵ */

        if (P > P1) {                         /* ���µ�һ����˳�Ƶڶ��� */
            P2 = P1;  k2 = k1;
            P1 = P;   k1 = k;
        } else if (P > P2) {                  /* ֻ���µڶ��� */
            P2 = P;   k2 = k;
        }
    }

    /* ���� ��̫��ʱ�����ҵڶ��壨�ɰ�����رմ˶Σ� ���� */
    if (fabsf((int32_t)k1 - (int32_t)k2) < 2) {
        P2 = -1.0f; k2 = 1;
        for (uint32_t k = 1; k < N_FFT/2; ++k) {
            if (k == k1) continue;
            if (mag[k] > P2) { P2 = mag[k]; k2 = k; }
        }
    }

//    /* ---------- 4) ���������߲�ֵ (1 bin ���� �� <0.1 bin) ---------- */
//#define PARABOLA_INTERP(_k, _f_out, _A_out)                          \
//    do {                                                             \
//        uint32_t km1 = (_k==0U)?_k : _k-1U;                          \
//        uint32_t kp1 = (_k==N_FFT/2-1U)?_k : _k+1U;                  \
//        float a = logf(mag[km1]);                                    \
//        float b = logf(mag[_k]);                                     \
//        float c = logf(mag[kp1]);                                    \
//        float delta = 0.5f * (c - a) / (2.0f*b - a - c);             \
//        float k_ref = (float)_k + delta;                             \
//        *(_f_out) = k_ref * (FS_HZ / (float)N_FFT);                  \
//        /* ��ֵ������pk_corr = exp(b - 0.25*(c-a)*delta)            */ \
//        float pk_corr = expf(b - 0.25f*(c - a)*delta);               \
//        *(_A_out) = 2.0f * sqrtf(pk_corr) / ((float)N_RAW * HANN_CG);\
//    } while(0)

//    /* �ȱ�֤��Ƶ��������� */
//    if (k1 > k2) { uint32_t kt=k1; k1=k2; k2=kt; float Pt=P1; P1=P2; P2=Pt; }

//    PARABOLA_INTERP(k1, f1_est, A1_est);
//    PARABOLA_INTERP(k2, f2_est, A2_est);

	// �ȱ�֤��Ƶ���������

	// ��֤k1ʱ��С��k2
    if (k1 > k2) {
        uint32_t kt = k1; k1 = k2; k2 = kt;
        float Pt = P1;    P1 = P2; P2 = Pt;
    }

    *f1_est = k1 * (FS_HZ / (float)N_FFT);                 // binתHz
    *A1_est = 2.0f * sqrtf(P1) / ((float)N_RAW * HANN_CG); // Hann����һ����ֵ
    *f2_est = k2 * (FS_HZ / (float)N_FFT);
    *A2_est = 2.0f * sqrtf(P2) / ((float)N_RAW * HANN_CG);
}

void fft_top5_hann_zero_nointp(const float *adc)
{
    init_hann_fft();

    // 1) �� Hann �� + 0 ��
    for (uint32_t n = 0; n < N_RAW; ++n)
        buf[n] = adc[n] * win[n];
    memset(&buf[N_RAW], 0, (N_FFT - N_RAW) * sizeof(float));

    // 2) ʵ�� FFT
    arm_rfft_fast_f32(&cfg, buf, buf, 0);

    // 3) ���߹����ף���top-5
    #define TOPN 5
    float topP[TOPN] = {-1,-1,-1,-1,-1};
    uint32_t topK[TOPN] = {1,1,1,1,1};

    for (uint32_t k = 1; k < N_FFT/2; ++k) {
        float re = buf[2*k];
        float im = buf[2*k+1];
        float P  = re*re + im*im;
        // ��������ά��top-5
        for(int i=0;i<TOPN;i++) {
            if(P > topP[i]) {
                for(int j=TOPN-1;j>i;j--) { topP[j]=topP[j-1]; topK[j]=topK[j-1]; }
                topP[i]=P; topK[i]=k;
                break;
            }
        }
    }
    // ��ӡtop-5���Ѱ���ֵ����
    printf("Top 5 Peaks (Descending):\n");
    for(int i=0;i<TOPN;i++) {
        float freq = topK[i]*(FS_HZ/(float)N_FFT);
        float amp  = 2.0f * sqrtf(topP[i]) / ((float)N_RAW * HANN_CG);
        printf("Peak %d: F = %.2f Hz,  Amp = %.4f V\n", i+1, freq, amp);
    }
}


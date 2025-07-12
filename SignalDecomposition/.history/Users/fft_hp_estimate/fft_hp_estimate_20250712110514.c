#include "fft_hp_estimate.h"

static float in_buf[N_FFT];   /* N_RAW 数据 + 余下置零 */
static float mag[N_FFT/2];    /* 幅值平方（实数）      */

// 在对信号做 FFT 变换前，通常会对每个采样点乘以 hann(n, N) 的结果，以减少频谱泄漏，提高频谱分析的准确性。
static inline float hann(float n, float N)
{
    return 0.5f * (1.0f - arm_cos_f32(2.0f * PI * n / N));
}

/* Hann 窗的 **coherent gain** = 0.5；用于幅值还原 */
#define HANN_CG   0.5f

/*------------------------------------------------------------------
 * 输入：adc[] (int16_t * N_RAW)  —— 已归一到 [-1,1) 的原始序列
 * 输出：*f_est   基波频率 (Hz)
 *       *A_est   基波幅值 (峰值，不是 rms)
 *----------------------------------------------------------------*/
void fft_hann_zero_interp(const float *adc, float *f_est, float *A_est)
{
    /* 1) 加 Hann 窗 + 拷贝 + 补零 */
    for (uint32_t n = 0; n < N_RAW; n++)
        in_buf[n] = adc[n] * hann(n, N_RAW-1);
    for (uint32_t n = N_RAW; n < N_FFT; n++)    // 补零
        in_buf[n] = 0.0f;

    /* 2) 实 FFT (CMSIS) */
    arm_rfft_fast_instance_f32 cfg;                
    arm_rfft_fast_init_f32(&cfg, N_FFT);            // 初始化 FFT 配置
    arm_rfft_fast_f32(&cfg, in_buf, in_buf, 0);     //  第一个in_buf作为输入参数,
                                                    //  第二个in_buf作为输出参数, 0表示FFT正变换

    /* 3) 幅值平方并找最大 bin  (跳过 DC) */
    float Pmax = 0.0f; uint32_t kmax = 1;
    for (uint32_t k = 1; k < N_FFT/2; k++)
    {
        float re = in_buf[2*k];
        float im = in_buf[2*k+1];
        float P  = re*re + im*im;
        mag[k] = P;
        if (P > Pmax)  { Pmax = P; kmax = k; }
    }

    /* 4) 抛物线插值（三点） */
    uint32_t km1 = (kmax==0) ? kmax : kmax-1;   // kmac的左边点
    uint32_t kp1 = (kmax==N_FFT/2-1) ? kmax : kmax+1;   // kmax的右边点 

    float alpha = mag[km1];
    float beta  = mag[kmax];
    float gamma = mag[kp1];

    float delta = 0.5f*(alpha - gamma) / (alpha - 2.0f*beta + gamma);
    float k_ref = (float)kmax + delta;          // 主峰精确位置

    /* 5) 频率 & 幅值（补零不影响幅值，只需除以窗增益） */
    *f_est = k_ref * FS_HZ / N_FFT;

    /* 用插值后的真幅值：|X(k)| = sqrt(beta - 0.25*(α-γ)*δ) */
    float pk_corr = beta - 0.25f*(alpha - gamma)*delta;
    *A_est = 2.0f * sqrtf(pk_corr) / (N_RAW * HANN_CG);   /* ×2 取峰-峰 → 峰值 */
}

/* 将 ADC 原始数据 (volatile uint16_t 数组) 减去均值，调整到 0 偏置，归一化到 [-1, 1)，返回 float 指针 */
void adc_zero_bias(const uint16_t *adc_raw, float* adc_zeroed, uint32_t len)
{

    float sum = 0.0f;
    for (uint32_t i = 0; i < len; i++)
        sum += (float)adc_raw[i];
    float mean = sum / (float)len;
    
    for (uint32_t i = 0; i < len; i++)
        adc_zeroed[i] = ((float)adc_raw[i] - mean) * ADC_LSB_VOLT; // 假设 ADC 采样范围为 16 位
    return buf;
}
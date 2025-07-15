#include "./fft_hp_estimate/fft_hp_estimate.h"
#include "usart.h"
#include <stdio.h>

static float      win[N_RAW];          // Hann(n)
static float      buf[N_FFT];          // FFT 输入 & 输出共用
static float      mag[N_FFT/2];        // 单边功率谱（P = |X|^2）
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

/**
 * @brief 对ADC原始数据进行去零偏处理
 *
 * 此函数计算输入ADC原始数据的均值，并将每个采样值减去均值后乘以ADC的电压分辨率（ADC_LSB_VOLT），
 * 得到去零偏后的电压值。结果存储在adc_zeroed数组中。
 *
 * @param[in]  adc_raw     ADC原始数据数组指针
 * @param[out] adc_zeroed  去零偏后的电压值数组指针
 * @param[in]  len         数据长度
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
 * @brief 对输入信号进行FFT分析，估算前两大频率分量及其幅值（汉宁窗+零填充+抛物线插值）。
 *
 * 此函数对输入的ADC采样数据进行如下处理：
 * 1. 乘以汉宁窗并进行零填充；
 * 2. 进行实数快速傅里叶变换（FFT）；
 * 3. 计算每个频率bin的幅度平方，寻找幅值最大的两个bin；
 *    - 若两峰落在相邻bin，则对较小峰重新寻找；
 * 4. 对每个峰进行抛物线插值，精确估算频率和幅值。
 *
 * @param[in]  adc     输入的ADC采样数据数组，长度为N_RAW
 * @param[out] f1_est  第一大频率分量的估算频率（Hz）
 * @param[out] A1_est  第一大频率分量的估算幅值
 * @param[out] f2_est  第二大频率分量的估算频率（Hz）
 * @param[out] A2_est  第二大频率分量的估算幅值
 *
 * @note
 * - 使用汉宁窗减少频谱泄漏，HANN_CG为窗的校正系数；
 * - 频率估算采用抛物线插值提高精度；
 * - 需保证输入数组adc长度为N_RAW，输出指针有效。
 */
void fft_top2_hann_zero_interp(const float *adc,
                               float *f1_est, float *A1_est,
                               float *f2_est, float *A2_est)
{
    /* ---------- 与旧函数相同：1) 加窗 + 0 补，2) 实 FFT ---------- */
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

    /* ---------- 3) 幅度平方并找第一、第二大 bin ---------- */
    float P1 = 0.0f, P2 = 0.0f;
    uint32_t k1 = 1, k2 = 1;

    for (uint32_t k = 1; k < N_FFT/2; k++)
    {
        float re = in_buf[2*k];
        float im = in_buf[2*k+1];
        float P  = re*re + im*im;
        mag[k] = P;

        if (P > P1) {                 /* 更新第一名、顺推第二名 */
            P2 = P1;  k2 = k1;
            P1 = P;   k1 = k;
        } else if (P > P2) {          /* 只更新第二名 */
            P2 = P;   k2 = k;
        }
    }

    /* 若两峰落在相邻 bin，可根据需要再加一个排除带 */
    if (fabsf((int32_t)k1 - (int32_t)k2) < 2) {
        /* 简单做法：把较小峰再向下寻找一个新峰 */
        P2 = 0.0f; k2 = 1;
        for (uint32_t k = 1; k < N_FFT/2; k++) {
            if (k==k1) continue;
            if (mag[k] > P2) { P2 = mag[k]; k2 = k; }
        }
    }

    /* ---------- 4) 对每个峰做抛物线插值 ---------- */
    /* --- helper 宏 --- */
   /* --- helper 宏 --- */
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

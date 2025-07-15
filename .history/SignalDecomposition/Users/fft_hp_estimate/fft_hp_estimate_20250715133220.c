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


void fft_top2_hann_zero_interp(const float *adc,
                               float *f1_est, float *A1_est,
                               float *f2_est, float *A2_est)
{
    init_hann_fft();                          /* 懒加载 */

    /* ---------- 1) 乘 Hann 窗 + 0 补 ---------- */
    for (uint32_t n = 0; n < N_RAW; ++n)
        buf[n] = adc[n] * win[n];
    memset(&buf[N_RAW], 0, (N_FFT - N_RAW) * sizeof(float));

    /* ---------- 2) 实数 FFT ---------- */
    arm_rfft_fast_f32(&cfg, buf, buf, 0);     /* 原地复用 */

    /* ---------- 3) 单边功率谱 & 找 top-2 ---------- */
    float P1 = -1.0f, P2 = -1.0f;
    uint32_t k1 = 1,   k2 = 1;

    for (uint32_t k = 1; k < N_FFT/2; ++k) {  /* 跳过 DC */
        float re = buf[2*k];                  /* CMSIS: 实部在偶，下标=2k */
        float im = buf[2*k+1];                /*          虚部在奇，下标=2k+1 */
        float P  = re*re + im*im;             /* 幅度平方 = 功率谱 */
        mag[k] = P;                           /* 存入数组便于后面插值 */

        if (P > P1) {                         /* 更新第一名，顺推第二名 */
            P2 = P1;  k2 = k1;
            P1 = P;   k1 = k;
        } else if (P > P2) {                  /* 只更新第二名 */
            P2 = P;   k2 = k;
        }
    }

    @ /* —— 峰太近时重新找第二峰（可按需求关闭此段） —— */
    @ if (fabsf((int32_t)k1 - (int32_t)k2) < 2) {
    @     P2 = -1.0f; k2 = 1;
    @     for (uint32_t k = 1; k < N_FFT/2; ++k) {
    @         if (k == k1) continue;
    @         if (mag[k] > P2) { P2 = mag[k]; k2 = k; }
    @     }
    @ }

    /* ---------- 4) 对数抛物线插值 (1 bin 精度 → <0.1 bin) ---------- */
#define PARABOLA_INTERP(_k, _f_out, _A_out)                          \
    do {                                                             \
        uint32_t km1 = (_k==0U)?_k : _k-1U;                          \
        uint32_t kp1 = (_k==N_FFT/2-1U)?_k : _k+1U;                  \
        float a = logf(mag[km1]);                                    \
        float b = logf(mag[_k]);                                     \
        float c = logf(mag[kp1]);                                    \
        float delta = 0.5f * (c - a) / (2.0f*b - a - c);             \
        float k_ref = (float)_k + delta;                             \
        *(_f_out) = k_ref * (FS_HZ / (float)N_FFT);                  \
        /* 幅值补偿：pk_corr = exp(b - 0.25*(c-a)*delta)            */ \
        float pk_corr = expf(b - 0.25f*(c - a)*delta);               \
        *(_A_out) = 2.0f * sqrtf(pk_corr) / ((float)N_RAW * HANN_CG);\
    } while(0)

    /* 先保证按频率升序输出 */
    if (k1 > k2) { uint32_t kt=k1; k1=k2; k2=kt; float Pt=P1; P1=P2; P2=Pt; }

    PARABOLA_INTERP(k1, f1_est, A1_est);
    PARABOLA_INTERP(k2, f2_est, A2_est);
}

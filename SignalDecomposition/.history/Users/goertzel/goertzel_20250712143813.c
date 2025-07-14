#include "goertzel.h"
#include "arm_math.h"          /* CMSIS-DSP：sin、cos、atan2、sqrt */

#define TWO_PI_F 6.283185307179586476f

/********************  初始化  ************************/
void goertzel_init(goertzel_cfg_t *cfg,
                   uint16_t        N,
                   float           target_freq,
                   float           fs)
{
    cfg->N = N;

    /* k = round(f_target * N / fs)  —— 取最近的整数点 */
    float kf = (target_freq * (float)N) / fs;
    cfg->k = (uint16_t)(kf + 0.5f);

    float omega = TWO_PI_F * cfg->k / (float)N; /* w_k =  2pi * k/N */
    cfg->coeff  = 2.0f * arm_cos_f32(omega);
    cfg->sine   = arm_sin_f32(omega);
    cfg->cosine = arm_cos_f32(omega);
}

/********************  主计算  ************************/
/**
 * @brief 处理一帧 N 点数据（int16 原始 ADC 码），返回幅值与相位
 * @param cfg   Goertzel 配置结构体指针
 * @param x     输入数据指针（int16 原始 ADC 码）
 * @param mag   输出幅值指针（float）
 * @param phase 输出相位指针（float）
 * @note 该函数假设输入数据 x 的长度为 cfg->N
 * @note 计算结果的幅值不进行 2/N 归一化处理，按需自行处理。
 *       相位的单位为弧度。
 * @note 该函数使用了 CMSIS-DSP 库中的 arm_sqrt_f32 和 arm_atan2_f32 等函数，
 *       需要在项目中包含 CMSIS-DSP 库。
 */
void goertzel_process(const goertzel_cfg_t *cfg,
                      const int16_t        *x,
                      float               *mag,
                      float               *phase)
{
    float s_prev  = 0.0f;
    float s_prev2 = 0.0f;

    for (uint16_t n = 0; n < cfg->N; ++n)
    {
        /* 伪代码中：s = x[n] + coeff*s_prev ? s_prev2 */
        float s = (float)x[n] + cfg->coeff * s_prev - s_prev2;
        s_prev2 = s_prev;
        s_prev  = s;
    }

    /* 复指数权还原 DFT 第 k 点 */
    float real_part = s_prev - cfg->cosine * s_prev2;
    float imag_part = cfg->sine * s_prev2;

    /* 幅值与相位（幅值不作 2/N 归一，按需自行处理） */
    *mag   = arm_sqrt_f32(real_part * real_part + imag_part * imag_part);
    *phase = arm_atan2_f32(imag_part, real_part);
}

#include "goertzel.h"
#include "arm_math.h"          /* CMSIS-DSP��sin��cos��atan2��sqrt */

#define TWO_PI_F 6.283185307179586476f

/********************  ��ʼ��  ************************/
void goertzel_init(goertzel_cfg_t *cfg,
                   uint16_t        N,
                   float           target_freq,
                   float           fs)
{
    cfg->N = N;

    /* k = round(f_target * N / fs)  ���� ȡ����������� */
    float kf = (target_freq * (float)N) / fs;
    cfg->k = (uint16_t)(kf + 0.5f);

    float omega = TWO_PI_F * cfg->k / (float)N; /* w_k =  2pi * k/N */
    cfg->coeff  = 2.0f * arm_cos_f32(omega);
    cfg->sine   = arm_sin_f32(omega);
    cfg->cosine = arm_cos_f32(omega);
}

/* ���� �� �� init������ round() */
static inline void goertzel_init_fom(goertzel_cfg_f��_t *cfg,
                                    uint16_t N, float target_freq, float fs)
{
    cfg->N     = N;
    cfg->omega = TWO_PI_F * target_freq / fs;
    cfg->coeff = 2.0f * arm_cos_f32(cfg->omega);
    cfg->sine  = arm_sin_f32(cfg->omega);
    cfg->cosine= arm_cos_f32(cfg->omega);
}

/********************  �����㣨float �棩  ************************/
/**
 * @brief ����һ֡ N �㡰��ʵ��ѹֵ�������ط�ֵ����λ
 * @param cfg   Goertzel ����
 * @param x     ��������ָ�루float����ȥ��ƫ & �� ADC_LSB_VOLT ���㣩
 * @note �����ֵ��δ�� 2/N ��һ��   *mag �� ��Volt �� sample�� Ϊ��λ��
 *       ����õ���ֵ Vpk = 2*|X(k)|/N���� Vrms = Vpk/��2�������к���
 */
void goertzel_process_f32(const goertzel_cfg_t *cfg,
                          const float          *x,
                          float               *mag,
                          float               *phase)
{
    float s_prev  = 0.0f;
    float s_prev2 = 0.0f;

    for (uint16_t n = 0; n < cfg->N; ++n)
    {
        float s = x[n] + cfg->coeff * s_prev - s_prev2;
        s_prev2 = s_prev;
        s_prev  = s;
    }

    /* DFT �� k ���ʵ�鲿 */
    float real_part = s_prev - cfg->cosine * s_prev2;
    float imag_part = cfg->sine * s_prev2;

    *mag   = arm_sqrt_f32(real_part * real_part + imag_part * imag_part);
    *phase = arm_atan2_f32(imag_part, real_part);
}

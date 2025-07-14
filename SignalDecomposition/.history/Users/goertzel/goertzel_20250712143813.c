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

/********************  ������  ************************/
/**
 * @brief ����һ֡ N �����ݣ�int16 ԭʼ ADC �룩�����ط�ֵ����λ
 * @param cfg   Goertzel ���ýṹ��ָ��
 * @param x     ��������ָ�루int16 ԭʼ ADC �룩
 * @param mag   �����ֵָ�루float��
 * @param phase �����λָ�루float��
 * @note �ú��������������� x �ĳ���Ϊ cfg->N
 * @note �������ķ�ֵ������ 2/N ��һ�������������д���
 *       ��λ�ĵ�λΪ���ȡ�
 * @note �ú���ʹ���� CMSIS-DSP ���е� arm_sqrt_f32 �� arm_atan2_f32 �Ⱥ�����
 *       ��Ҫ����Ŀ�а��� CMSIS-DSP �⡣
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
        /* α�����У�s = x[n] + coeff*s_prev ? s_prev2 */
        float s = (float)x[n] + cfg->coeff * s_prev - s_prev2;
        s_prev2 = s_prev;
        s_prev  = s;
    }

    /* ��ָ��Ȩ��ԭ DFT �� k �� */
    float real_part = s_prev - cfg->cosine * s_prev2;
    float imag_part = cfg->sine * s_prev2;

    /* ��ֵ����λ����ֵ���� 2/N ��һ���������д��� */
    *mag   = arm_sqrt_f32(real_part * real_part + imag_part * imag_part);
    *phase = arm_atan2_f32(imag_part, real_part);
}

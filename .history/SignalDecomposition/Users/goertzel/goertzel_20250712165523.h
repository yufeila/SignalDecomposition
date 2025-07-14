/*
 * @Author: yyf 17786321727@163.com
 * @Date: 2025-07-09 20:09:06
 * @LastEditors: yyf 17786321727@163.com
 * @LastEditTime: 2025-07-12 16:55:23
 * @FilePath: /Users/goertzel/goertzel.h
 * @Description: ����Ĭ������,������`customMade`, ��koroFileHeader�鿴���� ��������: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _GOERTZEL_H_
#define _GOERTZEL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Goertzel �㷨���ýṹ��
// N �ǲ���������һ��Ϊ��FFTЧ�ʣ�N Ӧ���� 2 ���ݴη�, ��FFT_SIZE
// k ��Ŀ��Ƶ�ʶ�Ӧ���±꣬�Զ��Ƶ��������
// coeff �� 2*cos(��)���� = 2��*k/N
// sine �� sin(��)��cosine �� cos(��)

typedef struct {
    uint16_t N;
    float    omega;    /* ֱ�ӱ��� 2��f/fs������������ k          */
    float    coeff;
    float    sine;
    float    cosine;
} goertzel_cfg_f��_t;

/* ��ʼ�� ���� target_freq (Hz) �� ADC ������ fs (Hz) ���� k */
void goertzel_init(goertzel_cfg_t *cfg,
                   uint16_t        N,
                   float           target_freq, /* Ŀ��Ƶ�� (Hz) */
                   float           fs);

/* ����һ֡ N �㡰��ƫ-У׼��ĸ����ѹ�������ط�ֵ����λ */
void goertzel_process_f32(const goertzel_cfg_t *cfg,
                          const float          *x,
                          float               *mag,
                          float               *phase);
                          
#ifdef __cplusplus
}
#endif
#endif /* _GOERTZEL_H_ */

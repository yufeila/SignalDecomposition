/*
 * @Author: yyf 17786321727@163.com
 * @Date: 2025-07-09 21:54:59
 * @LastEditors: yyf 17786321727@163.com
 * @LastEditTime: 2025-07-12 11:04:57
 * @FilePath: /SignalDecomposition/Users/fft_hp_estimate/fft_hp_estimate.h
 * @Description: ����Ĭ������,������`customMade`, ��koroFileHeader�鿴���� ��������: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __FFT_HP_ESTIMATE_H
#define __FFT_HP_ESTIMATE_H

#include "arm_math.h"
#include "tim.h"


#define N_RAW      FFT_SIZE                /* ��ʵ������               */
#define Z_FACTOR   2u                   /* ���㱶��                 */
#define N_FFT      (N_RAW * Z_FACTOR)   /* FFT ���� (����2����)     */
#define PSC         (TIM2-> PSC)
#define ARR         (TIM2-> ARR)
#define TIMCLK      84000000.0f         /* APB �� 2��ʵ��ֵ         */
#define FS_HZ      800000.0f           /* ������                  */

#define ADC_LSB_VOLT            0.0008058f  // ADC��LSB��ѹֵ (3.3V / 4096 = 0.0008058V)

/* Hann ���� **coherent gain** = 0.5�����ڷ�ֵ��ԭ */
#define HANN_CG   0.5f

void fft_hann_zero_interp(const float *adc, float *f_est, float *A_est);
void adc_zero_bias(const uint16_t *adc_raw, float* adc_zeroed, uint32_t len);
void fft_top5_hann_zero_nointp(const float *adc);
/* ����������ǿ��ֵ��Ƶ�ʣ�Hz�����ֵ���ȣ���ֵ������ rms��
 * Ҫ��f1_amp �� f2_amp
 */
void fft_top2_hann_zero_interp(const float *adc,
                               float *f1_est, float *A1_est,
                               float *f2_est, float *A2_est);
                               
#endif /* fft_hp_estimate.h */

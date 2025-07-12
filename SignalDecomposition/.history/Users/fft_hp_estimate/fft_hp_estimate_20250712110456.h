#ifndef __FFT_HP_ESTIMATE_H
#define __FFT_HP_ESTIMATE_H

#include "arm_math.h"
#include "arm_const_structs.h"


#define N_RAW      2048u                /* 真实采样点               */
#define Z_FACTOR   4u                   /* 补零倍率                 */
#define N_FFT      (N_RAW * Z_FACTOR)   /* FFT 点数 (必须2的幂)     */
#define FS_HZ      250000.0f            /* 采样率                   */

#define ADC_LSB_VOLT            0.0008058f  // ADC的LSB电压值 (3.3V / 4096 = 0.0008058V)

#endif /* fft_hp_estimate.h */

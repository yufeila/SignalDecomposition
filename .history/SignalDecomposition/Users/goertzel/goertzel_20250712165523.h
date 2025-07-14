/*
 * @Author: yyf 17786321727@163.com
 * @Date: 2025-07-09 20:09:06
 * @LastEditors: yyf 17786321727@163.com
 * @LastEditTime: 2025-07-12 16:55:23
 * @FilePath: /Users/goertzel/goertzel.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _GOERTZEL_H_
#define _GOERTZEL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Goertzel 算法配置结构体
// N 是采样点数，一般为了FFT效率，N 应该是 2 的幂次方, 即FFT_SIZE
// k 是目标频率对应的下标，自动推到最近整数
// coeff 是 2*cos(ω)，ω = 2π*k/N
// sine 是 sin(ω)，cosine 是 cos(ω)

typedef struct {
    uint16_t N;
    float    omega;    /* 直接保存 2πf/fs，而不是整数 k          */
    float    coeff;
    float    sine;
    float    cosine;
} goertzel_cfg_fω_t;

/* 初始化 —— target_freq (Hz) 与 ADC 采样率 fs (Hz) 决定 k */
void goertzel_init(goertzel_cfg_t *cfg,
                   uint16_t        N,
                   float           target_freq, /* 目标频率 (Hz) */
                   float           fs);

/* 处理一帧 N 点“零偏-校准后的浮点电压”，返回幅值与相位 */
void goertzel_process_f32(const goertzel_cfg_t *cfg,
                          const float          *x,
                          float               *mag,
                          float               *phase);
                          
#ifdef __cplusplus
}
#endif
#endif /* _GOERTZEL_H_ */

#ifndef __PHASE_CONFIG_H
#define __PHASE_CONFIG_H

#include <stdint.h>
#include "./x9cxxx/bsp_x9cxxx.h"

typedef struct {
    float phi_deg;        // 目标相位（度）
    float freq_Hz;        // 测得频率（Hz）
    float R1_ohm;         // X9C503实际阻值
    float R2_ohm;         // X9C103实际阻值
    float phi_actual_deg; // 实际实现相位
    float error_deg;      // 误差
} PhaseConfig_t;

// 主算法函数
void PhaseConfig_SetAndApply(PhaseConfig_t *cfg);

#endif

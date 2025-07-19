#include "./x9cxxx/bsp_x9cxxx.h"
#include "./phase_config/phase_config.h"
#include <math.h>
#include <stdio.h>

#define PI 3.1415926f

void PhaseConfig_SetAndApply(PhaseConfig_t *cfg)
{
    // --- 电容参数 ---
    const float C1 = 1e-9f;      // 1 nF
    const float C2 = 470e-12f;   // 470 pF

    // --- 频率和相位 ---
    float w = 2.0f * PI * cfg->freq_Hz;
    float phi_rad = cfg->phi_deg * PI / 180.0f;

    // --- 1. 闭式近似 ---
    float Req = tanf(phi_rad / 4.0f) / w;
    float R1 = Req / C1;
    float R2 = Req / C2;

    // --- 2. 限幅到数字电位器范围 ---
    if(R1 < 0) R1 = 0;
    if(R1 > X9C503_TOTAL_RESISTANCE) R1 = X9C503_TOTAL_RESISTANCE;
    if(R2 < 0) R2 = 0;
    if(R2 > X9C103_TOTAL_RESISTANCE) R2 = X9C103_TOTAL_RESISTANCE;

    // --- 3. 步进优化 ---
    // 步数计算
    int s1 = (int)roundf(R1 / X9C503_TOTAL_RESISTANCE * X9C503_STEPS);
    int s2 = (int)roundf(R2 / X9C103_TOTAL_RESISTANCE * X9C103_STEPS);
    if (s1 < 0) s1 = 0; if (s1 > X9C503_STEPS) s1 = X9C503_STEPS;
    if (s2 < 0) s2 = 0; if (s2 > X9C103_STEPS) s2 = X9C103_STEPS;

    // 局部误差优化（±2步窗口）
    float best_err = 360.0f;
    float best_phi = 0;
    int best1 = s1, best2 = s2;
    for(int i = -2; i <= 2; ++i) {
        for(int j = -2; j <= 2; ++j) {
            int t1 = s1 + i, t2 = s2 + j;
            if (t1 < 0 || t1 > X9C503_STEPS || t2 < 0 || t2 > X9C103_STEPS) continue;
            float R1t = (float)t1 / X9C503_STEPS * X9C503_TOTAL_RESISTANCE;
            float R2t = (float)t2 / X9C103_STEPS * X9C103_TOTAL_RESISTANCE;
            float phi = 2.0f * atanf(w * R1t * C1) + 2.0f * atanf(w * R2t * C2);
            float err = fabsf(phi * 180.0f / PI - cfg->phi_deg);
            if (err < best_err) {
                best_err = err;
                best1 = t1; best2 = t2;
                best_phi = phi * 180.0f / PI;
            }
        }
    }

    // 计算最终阻值
    cfg->R1_ohm = (float)best1 / X9C503_STEPS * X9C503_TOTAL_RESISTANCE;
    cfg->R2_ohm = (float)best2 / X9C103_STEPS * X9C103_TOTAL_RESISTANCE;
    cfg->phi_actual_deg = best_phi;
    cfg->error_deg = best_err;

    // --- 4. 写入数字电位器（直接调用你的驱动）---
    X9C503_SetResistance(cfg->R1_ohm);  // 设置50k电位器
    X9C103_SetResistance(cfg->R2_ohm);  // 设置10k电位器
}

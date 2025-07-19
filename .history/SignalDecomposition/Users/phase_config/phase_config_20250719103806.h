#ifndef __PHASE_CONFIG_H
#define __PHASE_CONFIG_H

#include <stdint.h>
#include "./x9cxxx/bsp_x9cxxx.h"

typedef struct {
    float phi_deg;        // Ŀ����λ���ȣ�
    float freq_Hz;        // ���Ƶ�ʣ�Hz��
    float R1_ohm;         // X9C503ʵ����ֵ
    float R2_ohm;         // X9C103ʵ����ֵ
    float phi_actual_deg; // ʵ��ʵ����λ
    float error_deg;      // ���
} PhaseConfig_t;

// ���㷨����
void PhaseConfig_SetAndApply(PhaseConfig_t *cfg);

#endif

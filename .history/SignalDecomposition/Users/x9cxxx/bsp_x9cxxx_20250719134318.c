#include "./x9cxxx/bsp_x9cxxx.h"

// ---- �Ƽ�ʹ�� DWT ��ʱ�� HAL_DelayUs ----
void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void DWT_Delay_us(uint32_t us)
{
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

// ---- 1. ѡ��/ȡ��оƬ ----
void X9C103_Select(void)   { X9C103_CS_LOW(); }
void X9C103_Deselect(void) { X9C103_CS_HIGH(); }
void X9C503_Select(void)   { X9C503_CS_LOW(); }
void X9C503_Deselect(void) { X9C503_CS_HIGH(); }

// ---- 2. ���÷��� ----
void X9C103_SetDirection(uint8_t up) {
    if(up) X9C103_UD_HIGH();
    else   X9C103_UD_LOW();
}

// ---- 3. INC ���庯������������Ч�� ----
void X9C103_IncPulse(void) {
    X9C103_INC_HIGH();  // INC�ߣ�׼����һ������
    DWT_Delay_us(1); // t_IH >= 1us
    X9C103_INC_LOW();   // INC�ͣ�����
    DWT_Delay_us(1); // t_IL >= 1us
    // ��һ��ѭ����INC�ߣ�����t_CYC >= 2us
}

// ---- 4. �������ú��� ----
void X9C_SetPos(uint8_t pos) {
    X9C103_Select();
    DWT_Delay_us(0.1); // t_CI >= 100ns, ������0.1us
    // ����ǰ����
    X9C103_SetDirection(0); // ���£����㣩
    DWT_Delay_us(3);     // t_ID >= 100ns, t_DI >= 2.9us
    for(int i=0; i<99; i++) X9C103_IncPulse();

    // ���ϲ�����Ŀ��ֵ
    X9C103_SetDirection(1);
    DWT_Delay_us(3);     // t_ID >= 100ns, t_DI >= 2.9us
    for(int i=0; i<pos; i++) X9C103_IncPulse();

    // ��ɺ󣬱���INCΪ��
    X9C103_INC_LOW();
    DWT_Delay_us(1); // t_IC >= 1us
    X9C103_Deselect();
}

// ---- 5. ����ʧ�Դ洢д�� ----
// ���裺��INC=�ߣ���CS�ӵ����ߣ��ȴ�t_CPH
void X9C_Store(void) {
    X9C_INC_HIGH();
    DWT_Delay_us(1);
    X9C_Deselect();     // CS ���ߣ��洢����
    DWT_Delay_us(20000);// �ȴ�20ms
    // ����INC�ߵ�ƽ
}

// ---- 6. ��ʼ������������whileѭ����ʼǰ��ʼ��IO�� ----
void X9C103_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = X9C103_INC_PIN | X9C103_UD_PIN | X9C103_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    X9C103_CS_HIGH();
    X9C103_INC_HIGH();
}

void X9C503_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = X9C503_INC_PIN | X9C503_UD_PIN | X9C503_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    X9C503_CS_HIGH();
    X9C503_INC_HIGH();
}

void X9C_SetPos_Store(uint8_t pos)
{
    X9C_SetPos(pos); // ���ò���
    X9C_Store();     // �洢������ʧNVM
}

/**
 * @brief   ����X9C503Ŀ����ֵ���Զ�ת��Ϊ����
 * @param   resistance  Ŀ����ֵ����λ��������Χ0~50k��
 */
void X9C503_SetResistance(float resistance)
{
    if(resistance < 0) resistance = 0;
    if(resistance > X9C503_TOTAL_RESISTANCE) resistance = X9C503_TOTAL_RESISTANCE;

    // ÿ����ֵ
    float r_step = X9C503_TOTAL_RESISTANCE / X9C503_STEPS;
    // Ŀ�경������������
    uint8_t pos = (uint8_t)((resistance / r_step) + 0.5f);

    X9C_SetPos(pos);
}

/**
 * @brief   ����X9C103Ŀ����ֵ���Զ�ת��Ϊ����
 * @param   resistance  Ŀ����ֵ����λ��������Χ0~10k��
 */
void X9C103_SetResistance(float resistance)
{
    if(resistance < 0) resistance = 0;
    if(resistance > X9C103_TOTAL_RESISTANCE) resistance = X9C103_TOTAL_RESISTANCE;

    // ÿ����ֵ
    float r_step = X9C103_TOTAL_RESISTANCE / X9C103_STEPS;
    // Ŀ�경������������
    uint8_t pos = (uint8_t)((resistance / r_step) + 0.5f);

    X9C_SetPos(pos);
}


// ---- ʹ���� ----
// X9C_Init();
// X9C_SetPos(45);   // ���õ�45���������ٲ�����
// X9C_Store();      // �洢������ʧNVM

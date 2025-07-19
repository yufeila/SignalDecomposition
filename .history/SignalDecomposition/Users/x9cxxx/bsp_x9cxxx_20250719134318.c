#include "./x9cxxx/bsp_x9cxxx.h"

// ---- 推荐使用 DWT 延时或 HAL_DelayUs ----
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

// ---- 1. 选中/取消芯片 ----
void X9C103_Select(void)   { X9C103_CS_LOW(); }
void X9C103_Deselect(void) { X9C103_CS_HIGH(); }
void X9C503_Select(void)   { X9C503_CS_LOW(); }
void X9C503_Deselect(void) { X9C503_CS_HIGH(); }

// ---- 2. 设置方向 ----
void X9C103_SetDirection(uint8_t up) {
    if(up) X9C103_UD_HIGH();
    else   X9C103_UD_LOW();
}

// ---- 3. INC 脉冲函数（负边沿有效） ----
void X9C103_IncPulse(void) {
    X9C103_INC_HIGH();  // INC高，准备下一个脉冲
    DWT_Delay_us(1); // t_IH >= 1us
    X9C103_INC_LOW();   // INC低，触发
    DWT_Delay_us(1); // t_IL >= 1us
    // 下一个循环再INC高，周期t_CYC >= 2us
}

// ---- 4. 步进设置函数 ----
void X9C_SetPos(uint8_t pos) {
    X9C103_Select();
    DWT_Delay_us(0.1); // t_CI >= 100ns, 这里用0.1us
    // 步进前归零
    X9C103_SetDirection(0); // 向下（归零）
    DWT_Delay_us(3);     // t_ID >= 100ns, t_DI >= 2.9us
    for(int i=0; i<99; i++) X9C103_IncPulse();

    // 向上步进到目标值
    X9C103_SetDirection(1);
    DWT_Delay_us(3);     // t_ID >= 100ns, t_DI >= 2.9us
    for(int i=0; i<pos; i++) X9C103_IncPulse();

    // 完成后，保持INC为低
    X9C103_INC_LOW();
    DWT_Delay_us(1); // t_IC >= 1us
    X9C103_Deselect();
}

// ---- 5. 非易失性存储写入 ----
// 步骤：先INC=高，再CS从低拉高，等待t_CPH
void X9C_Store(void) {
    X9C_INC_HIGH();
    DWT_Delay_us(1);
    X9C_Deselect();     // CS 拉高，存储触发
    DWT_Delay_us(20000);// 等待20ms
    // 保持INC高电平
}

// ---- 6. 初始化（必须先在while循环开始前初始化IO） ----
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
    X9C_SetPos(pos); // 设置步数
    X9C_Store();     // 存储到非易失NVM
}

/**
 * @brief   设置X9C503目标阻值，自动转换为步数
 * @param   resistance  目标阻值（单位：Ω，范围0~50k）
 */
void X9C503_SetResistance(float resistance)
{
    if(resistance < 0) resistance = 0;
    if(resistance > X9C503_TOTAL_RESISTANCE) resistance = X9C503_TOTAL_RESISTANCE;

    // 每步阻值
    float r_step = X9C503_TOTAL_RESISTANCE / X9C503_STEPS;
    // 目标步数，四舍五入
    uint8_t pos = (uint8_t)((resistance / r_step) + 0.5f);

    X9C_SetPos(pos);
}

/**
 * @brief   设置X9C103目标阻值，自动转换为步数
 * @param   resistance  目标阻值（单位：Ω，范围0~10k）
 */
void X9C103_SetResistance(float resistance)
{
    if(resistance < 0) resistance = 0;
    if(resistance > X9C103_TOTAL_RESISTANCE) resistance = X9C103_TOTAL_RESISTANCE;

    // 每步阻值
    float r_step = X9C103_TOTAL_RESISTANCE / X9C103_STEPS;
    // 目标步数，四舍五入
    uint8_t pos = (uint8_t)((resistance / r_step) + 0.5f);

    X9C_SetPos(pos);
}


// ---- 使用例 ----
// X9C_Init();
// X9C_SetPos(45);   // 设置第45步（归零再步进）
// X9C_Store();      // 存储到非易失NVM

/**
  ******************************************************************************
  * @file    App_Encoder.c
  * @brief   电机转速采集模块实现（TIM3 正交编码器 + 4 极磁环 + 双霍尔）
  * @note    ── 计数换算推导 ──────────────────────────────────────────────
  *          1) 磁环径向 4 极（N-S-N-S），即 2 个极对；
  *          2) 霍尔每经过一个磁极翻转一次输出，一圈 4 个磁极 → 每通道
  *             输出 2 个方波周期，即 PPR = 4/2 = 2 脉冲/转；
  *          3) TIM3 为 TI12 模式（×4 解码：A、B 两路上升沿与下降沿都计数），
  *             故 CPR = 4 × PPR = 8 计数/转，分辨率 360/8 = 45° 机械角；
  *          4) M 法测速，窗口 T = 100ms 内计数增量为 ΔC：
  *               n[rpm] = ΔC × 60 / (CPR × i × T)
  *                      = ΔC × 60 / (8 × 1 × 0.1s) = ΔC × 75
  *             （i 为磁环所在轴到输出轴的传动比，直连取 1）。
  *          ── 精度说明 ──────────────────────────────────────────────────
  *          ±1 计数的量化误差在 100ms 窗口下折合 75rpm：
  *            3000rpm（ΔC≈40）→ 约 2.5%；5400rpm（ΔC≈72）→ 约 1.4%；
  *            600rpm （ΔC≈8） → 约 12%（低速时误差偏大，属低线数磁环固有
  *            限制，需要更高精度可加大 ENC_SAMPLE_MS 或改用测周法）。
  *          故叠加 1/4 系数的一阶 IIR 平滑，显示与闭环用滤波值，
  *          同时保留原始值用于观察量化台阶。
  *          仅正转应用：转速取绝对值，不对外输出方向符号。
  ******************************************************************************
  */
#include "App_Encoder.h"
#include "App_SysTem.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

/*------------------------------ 私有宏定义 ---------------------------------*/
#define ENC_RING_POLES          4u    /* 磁环极数（径向 4 极 N-S-N-S） */
#define ENC_PULSES_PER_REV      (ENC_RING_POLES / 2u)     /* 每转每通道脉冲数 = 2 */
#define ENC_COUNTS_PER_REV      (4u * ENC_PULSES_PER_REV) /* ×4 解码计数/转 = 8 */
#define ENC_GEAR_RATIO          1.0f  /* 磁环轴 → 输出轴 传动比（同轴直连=1） */

#define ENC_SAMPLE_MS           100u  /* 采样窗口/任务周期（ms） */
#define ENC_DEADBAND_COUNT      1     /* 计数死区（±1 计数视为静止，抗抖动） */
#define ENC_FILTER_SHIFT        2u    /* IIR 平滑：alpha = 1/4（滞后约 4 个窗口） */

/* 每 1 个计数/窗口 对应的转速：60000 / (CPR × i × T_ms) = 75 rpm */
#define ENC_RPM_PER_COUNT                                            \
        ((60.0f * 1000.0f) / ((float)ENC_COUNTS_PER_REV *            \
                              ENC_GEAR_RATIO * (float)ENC_SAMPLE_MS))

#define ENC_TASK_STACK          128u  /* 采样任务栈深度（字） */
#define ENC_TASK_PRIO           2u    /* 采样任务优先级（与风扇任务同级） */

/*------------------------------ 私有变量 ---------------------------------*/
static volatile float   s_speedRpm   = 0.0f;  /* 滤波后转速（rpm，仅非负） */
static volatile float   s_rawRpm     = 0.0f;  /* 单窗口原始转速（rpm） */
static volatile int32_t s_totalCount = 0;     /* 累计计数（带符号，正=正转） */

/*------------------------------ 私有函数 ---------------------------------*/
static void EncoderTask(void *argument);

/**
  * @brief  编码器模块初始化：清零计数、启动 TIM3 编码器接口、创建采样任务
  * @note   须在 vTaskStartScheduler() 之前、MX_TIM3_Init() 之后调用。
  *         上电计数器本来为 0，此处显式清零避免复位之外的复用场景残留。
  * @retval 1=成功，0=失败（任务创建失败）
  */
uint8_t Encoder_Init(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0u);
    (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    s_speedRpm   = 0.0f;
    s_rawRpm     = 0.0f;
    s_totalCount = 0;
    SysTem_SetCurrentRPM(0.0f);   /* 调度器启动前无并发，可直接写 */

    if (xTaskCreate(EncoderTask, "Encoder", ENC_TASK_STACK, NULL,
                    ENC_TASK_PRIO, NULL) != pdPASS)
    {
        return 0u;                /* 任务创建失败 */
    }

    return 1u;
}

/**
  * @brief  读取滤波后的转速
  * @retval 转速（rpm，非负）
  */
float Encoder_GetSpeedRpm(void)
{
    float rpm;

    taskENTER_CRITICAL();
    rpm = s_speedRpm;
    taskEXIT_CRITICAL();

    return rpm;
}

/**
  * @brief  读取单窗口原始转速（未滤波）
  * @retval 转速（rpm，非负）
  */
float Encoder_GetRawSpeedRpm(void)
{
    float rpm;

    taskENTER_CRITICAL();
    rpm = s_rawRpm;
    taskEXIT_CRITICAL();

    return rpm;
}

/**
  * @brief  读取累计计数（8 计数 = 1 转）
  * @retval 累计计数（带符号，正=正转方向）
  */
int32_t Encoder_GetTotalCount(void)
{
    int32_t count;

    taskENTER_CRITICAL();
    count = s_totalCount;
    taskEXIT_CRITICAL();

    return count;
}

/**
  * @brief  转速采样任务：固定 100ms 窗口读取 TIM3 计数增量并换算转速
  * @note   1) 用 (int16_t)(now - last) 求增量，天然兼容 16 位计数器回绕；
  *         2) 增量取绝对值（本机只正转，惯性反转也不输出负转速）；
  *         3) 死区 ±1 计数抑制静止时的抖动；
  *         4) 一阶 IIR 滤波后写入 SysTem 的 CurrentRPM（本任务为唯一写者）。
  *         vTaskDelayUntil 保证窗口周期恒定，避免任务抖动引入换算误差。
  * @param  argument 未使用
  * @retval 无（任务永不返回）
  */
static void EncoderTask(void *argument)
{
    TickType_t lastWake = xTaskGetTickCount();
    uint16_t   lastCnt  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    uint16_t   nowCnt;
    int16_t    delta;
    int16_t    mag;
    float      rawRpm;
    float      filtRpm;

    (void)argument;

    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(ENC_SAMPLE_MS));

        nowCnt  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        delta   = (int16_t)(nowCnt - lastCnt);   /* 16 位回绕安全 */
        lastCnt = nowCnt;

        s_totalCount += (int32_t)delta;

        mag = (delta < 0) ? (int16_t)(-delta) : delta;
        if (mag <= ENC_DEADBAND_COUNT)
        {
            rawRpm = 0.0f;                        /* 静止/抖动 → 0 */
        }
        else
        {
            rawRpm = (float)mag * ENC_RPM_PER_COUNT;
        }

        filtRpm = s_speedRpm;
        filtRpm += (rawRpm - filtRpm) / (float)(1u << ENC_FILTER_SHIFT);
        s_rawRpm   = rawRpm;
        s_speedRpm = filtRpm;

        SysTem_SetCurrentRPM(filtRpm);            /* 供 UI/主控/闭环读取 */
    }
}

/**
  ******************************************************************************
  * @file    App_ADC.c
  * @brief   温度采集模块实现（PB0 板载 NTC + PB1 液体 NTC，ADC1 轮询采集）
  * @note    ── 换算链推导 ──────────────────────────────────────────────────
  *          1) 分压（已按原理图核实：固定电阻 10k 接 VREF，NTC 10k 接地）：
  *               V = VREF × R_NTC / (R_FIX + R_NTC)   →   R_NTC = R_FIX×V/(VREF−V)
  *             25°C 时 R_NTC=10k → V ≈ 1.65V → ADC ≈ 2048（等分压）；
  *             25°C 附近灵敏度 ≈ 45 个计数/°C。上电自检：室温 25°C 时
  *             原始值应落在 2000~2100；若只有几百，说明接法与本假设相反，
  *             把 ADC_NTC_TO_GND 改为 0 即可（换算公式已做好两种接法）。
  *          2) B 值公式（NTC 出厂参数 R0=10k@25°C，B=3950）：
  *               1/T = 1/T0 + (1/B)×ln(R_NTC/R0)，T0 = 298.15K
  *               温度(℃) = T − 273.15
  *          ── 采集策略 ──────────────────────────────────────────────────
  *          ADC1 为单转换序列（NbrOfConversion=1），任务里对目标通道临时
  *          重配 Rank1 后连续转换 16 次取平均（软件过采样，等效 +2bit）；
  *          采样时间 239.5 周期（MX 已配），10k 分压源阻抗下充电时间
  *          约 20µs，采样窗口充裕，无需担忧精度。
  *          温度经 1/4 系数 IIR 平滑；水温（过程变量）写入 SysTem
  *          CurrentTemp，板温仅供监测。开路/短路判据：原始值 ≤2 / ≥4093。
  ******************************************************************************
  */
#include "App_ADC.h"
#include "App_SysTem.h"
#include "bsp_pins.h"
#include "adc.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <math.h>

/*------------------------------ 私有宏定义 ---------------------------------*/
/* NTC 与分压网络参数（按原理图核实：10k NTC，B=3950 ±1%，固定电阻 10kΩ） */
#define NTC_R0                  10000.0f  /* 25°C 标称阻值（Ω） */
#define NTC_B                   3950.0f   /* B 值（K） */
#define NTC_T0_K                298.15f   /* 标称温度 25°C（K） */
#define NTC_SERIES_R            10000.0f  /* 分压固定电阻（Ω），已核实为 10k */

#define ADC_VREF                3.3f      /* 参考电压 = VDDA（V） */
#define ADC_FULL_SCALE          4095.0f   /* 12 位满量程 */
#define ADC_OVERSAMPLE_CNT      16u       /* 每通道过采样次数（16 次均值） */
#define ADC_RAW_OPEN_TH         2u        /* 开路判据：原始值 ≤ 2 */
#define ADC_RAW_SHORT_TH        4093u     /* 短路判据：原始值 ≥ 4093 */

/* 分压接法：1 = 固定电阻接 VREF、NTC 接地（默认假设，见文件头自检方法）；
 *          0 = NTC 接 VREF、固定电阻接地 */
#define ADC_NTC_TO_GND          1

#define ADC_TASK_PERIOD_MS      200u      /* 采集任务周期（ms） */
#define ADC_FILTER_SHIFT        2u        /* 温度 IIR 平滑系数：alpha = 1/4 */
#define ADC_FAULT_SENTINEL_LO   (-274.0f) /* 开路哨兵值（预留调试观察用） */
#define ADC_FAULT_SENTINEL_HI   (-273.0f) /* 短路哨兵值 */

#define ADC_TASK_STACK          192u      /* 采集任务栈深度（字，含 float 运算） */
#define ADC_TASK_PRIO           2u        /* 采集任务优先级（与风扇/编码器任务同级） */

/*------------------------------ 私有变量 ---------------------------------*/
static volatile float    s_boardTempC = 0.0f;  /* 板载温度（℃，滤波后） */
static volatile float    s_waterTempC = 0.0f;  /* 水温（℃，滤波后） */
static volatile uint16_t s_boardRaw   = 0u;    /* 板载通道过采样平均值（原始） */
static volatile uint16_t s_waterRaw   = 0u;    /* 水温通道过采样平均值（原始） */
static volatile uint8_t  s_boardFault = 0u;    /* 板载 NTC 故障：1=开路/短路 */
static volatile uint8_t  s_waterFault = 0u;    /* 水温 NTC 故障：1=开路/短路 */

/*------------------------------ 私有函数 ---------------------------------*/
static void     AdcTask(void *argument);
static uint16_t Adc_ReadChannelAvg(uint32_t channel);
static float    Adc_RawToTempC(uint16_t raw);

/**
  * @brief  温度采集模块初始化：ADC 自校准并创建采集任务
  * @note   须在 vTaskStartScheduler() 之前、MX_ADC1_Init() 之后调用。
  * @retval 1=成功，0=失败（任务创建失败）
  */
uint8_t Adc_Init(void)
{
    /* ADC 上电自校准（F1 系列提高转换精度，只需执行一次） */
    (void)HAL_ADCEx_Calibration_Start(&hadc1);

    s_boardTempC = 0.0f;
    s_waterTempC = 0.0f;
    s_boardFault = 0u;
    s_waterFault = 0u;

    if (xTaskCreate(AdcTask, "Adc", ADC_TASK_STACK, NULL,
                    ADC_TASK_PRIO, NULL) != pdPASS)
    {
        return 0u;                /* 任务创建失败（FreeRTOS 堆不足） */
    }

    return 1u;
}

/**
  * @brief  读取板载温度（滤波后）
  * @retval 温度（℃），传感器故障时保持最后有效值
  */
float Adc_GetBoardTempC(void)
{
    float t;

    taskENTER_CRITICAL();
    t = s_boardTempC;
    taskEXIT_CRITICAL();

    return t;
}

/**
  * @brief  读取水温（滤波后，与 SysTem CurrentTemp 同源）
  * @retval 温度（℃），传感器故障时保持最后有效值
  */
float Adc_GetWaterTempC(void)
{
    float t;

    taskENTER_CRITICAL();
    t = s_waterTempC;
    taskEXIT_CRITICAL();

    return t;
}

/**
  * @brief  读取板载通道过采样平均原始值（换算诊断/标定用）
  * @retval ADC 原始值（0~4095，16 次平均）
  */
uint16_t Adc_GetBoardRaw(void)
{
    uint16_t raw;

    taskENTER_CRITICAL();
    raw = s_boardRaw;
    taskEXIT_CRITICAL();

    return raw;
}

/**
  * @brief  读取水温通道过采样平均原始值（换算诊断/标定用）
  * @retval ADC 原始值（0~4095，16 次平均）
  */
uint16_t Adc_GetWaterRaw(void)
{
    uint16_t raw;

    taskENTER_CRITICAL();
    raw = s_waterRaw;
    taskEXIT_CRITICAL();

    return raw;
}

/**
  * @brief  查询板载温度传感器故障状态
  * @retval 1=故障（开路/短路），0=正常
  */
uint8_t Adc_GetBoardFault(void)
{
    uint8_t f;

    taskENTER_CRITICAL();
    f = s_boardFault;
    taskEXIT_CRITICAL();

    return f;
}

/**
  * @brief  查询水温传感器故障状态
  * @retval 1=故障（开路/短路），0=正常
  */
uint8_t Adc_GetWaterFault(void)
{
    uint8_t f;

    taskENTER_CRITICAL();
    f = s_waterFault;
    taskEXIT_CRITICAL();

    return f;
}

/**
  * @brief  温度采集任务：周期采集两路 NTC，换算温度并写入系统状态
  * @note   1) 每周期对 CH8（板温）/CH9（水温）各过采样 16 次取平均；
  *         2) 原始值开路/短路检测：故障时温度保持最后有效值并打印一次；
  *         3) 温度经 1/4 IIR 平滑后更新内部值；
  *         4) 水温写入 SysTem 的 CurrentTemp（本任务为唯一写者）。
  * @param  argument 未使用
  * @retval 无（任务永不返回）
  */
static void AdcTask(void *argument)
{
    TickType_t lastWake = xTaskGetTickCount();
    uint16_t rawBoard;
    uint16_t rawWater;
    float    temp;
    uint8_t  fault;
    uint8_t  lastBoardFault = 0xFFu;   /* 初值非法，保证首轮必执行日志判定 */
    uint8_t  lastWaterFault = 0xFFu;

    (void)argument;

    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(ADC_TASK_PERIOD_MS));

        /* ---------- 板载温度（PB0，ADC1_IN8） ---------- */
        rawBoard = Adc_ReadChannelAvg(ADC_CHANNEL_8);
        s_boardRaw = rawBoard;
        fault    = ((rawBoard <= ADC_RAW_OPEN_TH) || (rawBoard >= ADC_RAW_SHORT_TH)) ? 1u : 0u;
        if (fault == 0u)
        {
            temp = s_boardTempC;
            temp += (Adc_RawToTempC(rawBoard) - temp) / (float)(1u << ADC_FILTER_SHIFT);
            s_boardTempC = temp;
        }
        s_boardFault = fault;
        if (fault != lastBoardFault)
        {
            printf("[Adc] board NTC %s (raw=%u)\r\n",
                   (fault != 0u) ? "FAULT" : "OK", (unsigned int)rawBoard);
            lastBoardFault = fault;
        }

        /* ---------- 水温（PB1，ADC1_IN9） ---------- */
        rawWater = Adc_ReadChannelAvg(ADC_CHANNEL_9);
        s_waterRaw = rawWater;
        fault    = ((rawWater <= ADC_RAW_OPEN_TH) || (rawWater >= ADC_RAW_SHORT_TH)) ? 1u : 0u;
        if (fault == 0u)
        {
            temp = s_waterTempC;
            temp += (Adc_RawToTempC(rawWater) - temp) / (float)(1u << ADC_FILTER_SHIFT);
            s_waterTempC = temp;
            SysTem_SetCurrentTemp(temp);   /* 过程变量写入系统状态（唯一写者） */
        }
        s_waterFault = fault;
        if (fault != lastWaterFault)
        {
            printf("[Adc] water NTC %s (raw=%u)\r\n",
                   (fault != 0u) ? "FAULT" : "OK", (unsigned int)rawWater);
            lastWaterFault = fault;
        }
    }
}

/**
  * @brief  采集指定 ADC 通道并做过采样平均
  * @note   ADC1 为单转换序列，这里对目标通道临时重配到 Rank1 后连续
  *         转换 ADC_OVERSAMPLE_CNT 次（ContinuousConvMode 已使能），
  *         取算术平均；采样时间用 MX 配置的 239.5 周期，源阻抗无忧。
  * @param  channel ADC 通道号（ADC_CHANNEL_x）
  * @retval 16 次采样的平均原始值（0~4095）
  */
static uint16_t Adc_ReadChannelAvg(uint32_t channel)
{
    ADC_ChannelConfTypeDef cfg = {0};
    uint32_t sum = 0u;
    uint8_t  i;

    /* 把目标通道重配到转换序列 Rank1（覆盖上一次的通道配置） */
    cfg.Channel      = channel;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    (void)HAL_ADC_ConfigChannel(&hadc1, &cfg);

    (void)HAL_ADC_Start(&hadc1);
    for (i = 0u; i < ADC_OVERSAMPLE_CNT; i++)
    {
        if (HAL_ADC_PollForConversion(&hadc1, 10u) == HAL_OK)
        {
            sum += HAL_ADC_GetValue(&hadc1);
        }
    }
    (void)HAL_ADC_Stop(&hadc1);

    return (uint16_t)(sum / ADC_OVERSAMPLE_CNT);
}

/**
  * @brief  ADC 平均原始值 → 温度（B 值公式）
  * @note   原始值 → 分压电压 → NTC 阻值 → B 值公式 → 摄氏度；
  *         两种分压接法由 ADC_NTC_TO_GND 编译期选择（见文件头）。
  * @param  raw 过采样平均后的 ADC 原始值
  * @retval 温度（℃）
  */
static float Adc_RawToTempC(uint16_t raw)
{
    float v;      /* 分压点电压 */
    float rntc;   /* NTC 当前阻值 */
    float tk;     /* 开尔文温度 */

    v = (float)raw * ADC_VREF / ADC_FULL_SCALE;

#if ADC_NTC_TO_GND
    /* 固定电阻接 VREF、NTC 接地：V = VREF×R_NTC/(R_FIX+R_NTC) */
    rntc = NTC_SERIES_R * v / (ADC_VREF - v);
#else
    /* NTC 接 VREF、固定电阻接地：V = VREF×R_FIX/(R_FIX+R_NTC) */
    rntc = NTC_SERIES_R * (ADC_VREF - v) / v;
#endif

    tk = 1.0f / ((1.0f / NTC_T0_K) + (1.0f / NTC_B) * logf(rntc / NTC_R0));

    return tk - 273.15f;
}

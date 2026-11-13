/**
  ******************************************************************************
  * @file    App_ADC.h
  * @brief   温度采集模块接口（PB0 板载 NTC + PB1 液体 NTC，ADC1 轮询采集）
  * @note    硬件连接（原理图网络名取自 bsp_pins.h）：
  *            BOARD_ADC（PB0，ADC1_IN8）→ 板载 10kΩ NTC（B=3950，±1%）；
  *            LIQUID_ADC（PB1，ADC1_IN9）→ 液体 10kΩ NTC（B=3950，±1%）；
  *          两路均为 NTC + 固定电阻（10kΩ，接 VREF；NTC 接地）分压网络，
  *          由 ADC1 分时轮询采集。
  *
  *          换算链（详见 .c 顶部推导）：
  *            ADC 原始值 → 分压电压 → NTC 阻值 → B 值公式 → 摄氏温度
  *
  *          刷新方式：Adc_Init() 创建 "Adc" 任务，每 200ms 对两路各采样
  *            16 次取平均（软件过采样），经一阶 IIR 平滑后更新内部值；
  *            水温（过程变量）写入 SysTem 的 CurrentTemp，供 KEY4 启动
  *            条件判断、OLED 显示与控温闭环使用；板温仅供监测/过热保护，
  *            通过 Adc_GetBoardTempC() 读取。
  *
  *          传感器故障检测：ADC 原始值接近 0（断线/开路）或接近满量程
  *            （短路）时置故障标志，温度值保持最后有效值，避免跳变。
  ******************************************************************************
  */
#ifndef __App_ADC_H__
#define __App_ADC_H__

#include <stdint.h>

/**
  * @brief  温度采集模块初始化：ADC 校准并创建采集任务
  * @note   须在 vTaskStartScheduler() 之前调用，且 MX_ADC1_Init() 已完成。
  *         动作：执行 ADC 自校准（上电一次，提高精度）；创建 "Adc" 任务，
  *         每 200ms 采集板温/水温各 16 次平均换算；水温自动写入
  *         SysTem 的 CurrentTemp（本任务为该成员唯一写者）。
  * @retval 1=成功，0=失败（采集任务创建失败）
  */
uint8_t Adc_Init(void);

/**
  * @brief  读取板载温度（PB0 NTC，滤波后）
  * @retval 温度（℃）；传感器故障时保持最后有效值，配合 Adc_GetBoardFault()
  */
float Adc_GetBoardTempC(void);

/**
  * @brief  读取水温（PB1 NTC，滤波后，与 SysTem CurrentTemp 同源）
  * @retval 温度（℃）；传感器故障时保持最后有效值，配合 Adc_GetWaterFault()
  */
float Adc_GetWaterTempC(void);

/**
  * @brief  读取板载通道过采样平均原始值（换算诊断/标定用）
  * @retval ADC 原始值（0~4095，16 次平均）
  */
uint16_t Adc_GetBoardRaw(void);

/**
  * @brief  读取水温通道过采样平均原始值（换算诊断/标定用）
  * @retval ADC 原始值（0~4095，16 次平均）
  */
uint16_t Adc_GetWaterRaw(void);

/**
  * @brief  查询板载温度传感器故障状态
  * @retval 1=故障（开路/短路），0=正常
  */
uint8_t Adc_GetBoardFault(void);

/**
  * @brief  查询水温传感器故障状态
  * @retval 1=故障（开路/短路），0=正常
  */
uint8_t Adc_GetWaterFault(void);

#endif /* __App_ADC_H__ */

/**
  ******************************************************************************
  * @file    App_Encoder.h
  * @brief   电机转速采集模块接口（TIM3 正交编码器 + 4 极磁环 + 双霍尔）
  * @note    硬件连接（原理图网络名取自 bsp_pins.h）：
  *            M1_ENC_A（PB4）→ TIM3_CH1，内部上拉，兼容霍尔开漏输出；
  *            M1_ENC_B（PB5）→ TIM3_CH2，内部上拉；
  *          两只霍尔（MT1401A-EN）在磁环圆周上机械相差 45°，对应电气 90°，
  *          因此两路输出构成标准正交信号，可判方向；TIM3 已由 CubeMX 配成
  *          TIM_ENCODERMODE_TI12（×4 解码：两路上升沿+下降沿均计数），
  *          ARR=65535 自由计数，预分频 0（计数时钟 72MHz，计数本身不分频）。
  *
  *          机械/磁性参数与换算（详见 .c 文件顶部推导）：
  *            磁环径向 4 极（N-S-N-S）→ 2 个极对；
  *            每转每通道脉冲数 PPR = 4 / 2 = 2；
  *            ×4 解码 → 每转计数 CPR = 4 × 2 = 8（分辨率 45° 机械角/计数）。
  *
  *          测速方法：M 法（固定窗口计数），窗口 ENC_SAMPLE_MS = 100ms，
  *            n[rpm] = ΔCount × 60 / (CPR × i × T) = ΔCount × 75（i=1 直连）；
  *            配 1/4 系数的一阶 IIR 平滑，抑制 ±1 计数量化抖动。
  *
  *          刷新方式：Encoder_Init() 创建 "Encoder" 任务，每 100ms 采样一次，
  *            把结果写入 SysTem 的 CurrentRPM（本任务为该成员唯一写者），
  *            UI/主控/闭环直接读 SysTem 或调用 Encoder_GetSpeedRpm()。
  ******************************************************************************
  */
#ifndef __App_Encoder_H__
#define __App_Encoder_H__

#include <stdint.h>

/**
  * @brief  编码器模块初始化：清零计数、启动 TIM3 编码器接口、创建采样任务
  * @note   须在 vTaskStartScheduler() 之前调用，且 MX_TIM3_Init() 已完成。
  *         上电把 CurrentRPM 置 0（此时无并发，可直接写），保证 UI 初值正确。
  * @retval 1=成功，0=失败（采样任务创建失败）
  */
uint8_t Encoder_Init(void);

/**
  * @brief  读取电机转速（滤波后，推荐用于显示与闭环）
  * @note   单位 rpm，恒为非负值（本机只正转，不区分旋转方向）；
  *         100ms 采样 + 1/4 IIR 平滑，等效滞后约 400ms，
  *         需要快速响应时可改用 Encoder_GetRawSpeedRpm()。
  * @retval 转速（rpm）
  */
float Encoder_GetSpeedRpm(void);

/**
  * @brief  读取电机转速（未滤波的单窗口原始值）
  * @note   一次窗口（100ms）内的直接换算结果，量化步进 75rpm
  *         （= 1 计数/100ms），用于标定量化误差与滤波器效果。
  * @retval 转速（rpm）
  */
float Encoder_GetRawSpeedRpm(void);

/**
  * @brief  读取累计计数（带符号，正=正转方向）
  * @note   32 位累加，已在内部处理 TIM3 的 16 位回绕；
  *         换算圈数：revolutions = Encoder_GetTotalCount() / 8.0f。
  * @retval 累计计数（计数单位，8 计数 = 1 转）
  */
int32_t Encoder_GetTotalCount(void);

#endif /* __App_Encoder_H__ */

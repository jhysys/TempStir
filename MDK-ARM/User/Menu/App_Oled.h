/**
  ******************************************************************************
  * @file    App_Oled.h
  * @brief   OLED 显示模块接口（SSD1315 控制器，I2C1 总线，128x64 点阵）
  * @note    本模块基于 LibDriver 的 driver_ssd1315 驱动，在初始化时把
  *          驱动的 IIC 接口回调挂接到 MDK-ARM/User/Int 的 bsp_i2c1
  *          阻塞式封装上，业务层只调用本文件暴露的接口，不直接
  *          操作驱动句柄与 I2C 外设。
  ******************************************************************************
  */
#ifndef APP_OLED_H
#define APP_OLED_H

#include <stdint.h>
#include "App_SysTem.h"   /* SysFocus_t：聚焦点类型 */

/**
  * @brief  OLED 模块初始化：完成芯片配置并创建 UI 刷新任务
  * @note   完成以下动作：链接驱动回调 → SSD1315 芯片初始化与寄存器配置 →
  *         清屏 → 创建 UI 任务（优先级 1）。UI 任务周期 200ms，按行差分
  *         无感刷新显示系统状态布局（设备 ID/三组目标与当前参数/系统
  *         状态/聚焦标记），并执行运行态监控规则。内部含阻塞式 I2C
  *         读写与毫秒级延时，须在任务上下文或 vTaskStartScheduler()
  *         之前的启动阶段调用，禁止在 ISR 调用。
  * @retval 1=成功，0=失败（I2C 不在线/芯片初始化失败/任务创建失败）
  */
uint8_t Oled_Init(void);

/**
  * @brief  设置 UI 参数聚焦点（供将来的按键任务调用）
  * @note   聚焦点决定 UI 中 '>' 标记的位置，在目标温度/目标转速/
  *         目标时间三行之间切换；内部通过 SysTem_SetFocus 写入全局
  *         状态（保持"按键任务唯一写者"纪律），UI 任务下一周期自动
  *         重绘标记，无需额外刷新调用。非法值不生效。
  * @param  focus 聚焦点（SYS_FOCUS_TEMP/SYS_FOCUS_RPM/SYS_FOCUS_TIME）
  * @retval 无
  */
void Oled_SetFocus(SysFocus_t focus);

#endif /* APP_OLED_H */

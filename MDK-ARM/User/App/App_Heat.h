/**
  ******************************************************************************
  * @file    App_Heat.h
  * @brief   加热控制模块接口（PA0 TIM2_CH1 PWM + UCC27517 门极驱动 + PID 闭环）
  * @note    硬件信号链（原理图网络名取自 bsp_pins.h）：
  *            HEAT_PWM（PA0，TIM2_CH1，1kHz PWM）→ UCC27517DBVR 的 IN+；
  *            UCC27517 的 IN− ← 74LVC1G74（D 触发器）的 Q；
  *            FAULT_RST_N（PC2，推挽输出）→ 74LVC1G74 的 R#D（低有效复位）；
  *            HEAT_FAULT_N（PC3，输入，低有效）→ U21 硬件故障锁存状态。
  *
  *          驱动真值（UCC27517：OUT = IN+ 与非 IN−）：
  *            IN− = 0（触发器复位态）→ OUT 跟随 IN+ → PWM 正常加热；
  *            IN− = 1（触发器置位态）→ OUT 强制低 → 硬件切断加热。
  *          软件写 FAULT_RST_N=低 脉冲可清除硬件锁存（Q 归 0），恢复受控。
  *
  *          PID 温度闭环（HeatCtrlTask，500ms 周期）：
  *            系统状态联动：RUN → 闭环加热；IDLE / FAULT → 停止输出；
  *            反馈：水温（SysTem CurrentTemp，App_ADC 采集任务唯一写者）；
  *            控制律：u = Kp×e + ∫Ki×e（位置式 PI，积分限幅 + 抗饱和），
  *            加热对象时间常数为分钟级且只单向输出（不能制冷），无需
  *            前馈与微分；到达目标后由积分维持保温功率。
  *          安全保护（任一触发即强制 0% 输出）：
  *            1) 水温传感器故障（探头断线/短路，加热无反馈严禁输出）；
  *            2) 水温绝对上限 HEAT_ABS_MAX_C；
  *            3) 板温上限 HEAT_BOARD_MAX_C（主板过热保护）；
  *            4) 硬件故障锁存（HEAT_FAULT_N 有效）。
  *          RUN 态下本任务是加热功率的唯一写者，手动 SetPower 会被覆盖。
  ******************************************************************************
  */
#ifndef __App_Heat_H__
#define __App_Heat_H__

#include <stdint.h>

/**
  * @brief  加热模块初始化：清除硬件故障锁存、启动 PWM（0%）并创建控制任务
  * @note   须在 vTaskStartScheduler() 之前调用，且 MX_TIM2_Init() 已完成。
  *         动作：FAULT_RST_N 低脉冲复位 74LVC1G74（Q→0，驱动使能）后释放；
  *         TIM2_CH1 启动 PWM 输出且比较值=0（IN+=0 → 不加热）；随后创建
  *         "HeatCtrl" 任务（500ms 周期）：系统 RUN → 按目标水温 PID 闭环，
  *         IDLE / FAULT → 停止输出；内置四重安全保护（见文件头）。
  * @retval 1=成功，0=失败（控制任务创建失败）
  */
uint8_t Heat_Init(void);

/**
  * @brief  加热开始（手动接口）：按当前设定功率输出 PWM
  * @note   RUN 态下会被 PID 任务覆盖，主要用于调试与非运行态点动。
  * @retval 无
  */
void Heat_Start(void);

/**
  * @brief  加热停止（手动接口）：比较值清零 → 驱动输出低 → 停止加热
  * @note   设定功率保留，下次 Heat_Start 原功率恢复。
  * @retval 无
  */
void Heat_Stop(void);

/**
  * @brief  更新加热功率（手动接口）
  * @param  percent 功率占空比（0~100，>100 饱和到 100）
  * @retval 无
  */
void Heat_SetPower(uint8_t percent);

/**
  * @brief  读取当前设定加热功率
  * @retval 功率占空比（%）
  */
uint8_t Heat_GetPower(void);

/**
  * @brief  查询是否正在加热
  * @retval 1=已启动且功率≥1%，0=停止
  */
uint8_t Heat_IsRunning(void);

#endif /* __App_Heat_H__ */

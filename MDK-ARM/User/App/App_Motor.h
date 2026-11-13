/**
  ******************************************************************************
  * @file    App_Motor.h
  * @brief   电机应用层接口（DRV8870DDAR，单向正转 + PWM 调速）
  * @note    硬件连接（原理图网络名取自 bsp_pins.h）：
  *            MOTOR1（PB8）→ DRV8870 IN1，复用为 TIM4_CH3 PWM 输出，调速；
  *            MOTOR2（PB9）→ DRV8870 IN2，普通推挽输出，固定低电平。
  *          方向固定为正转：IN1 = PWM，IN2 = 0 → 正转，占空比即平均电压；
  *          IN1 = 0 且 IN2 = 0 → 滑行（coast，电机自由减速停止）。
  *          PWM 时基：TIM4 挂 APB1（72MHz），ARR=3599 → 20kHz，超出人耳
  *          可听范围且满足 DRV8870 的开关频率要求，电机运行无啸叫。
  *          引脚电气属性由 Core/Src/gpio.c 与 tim.c 的 MSP 初始化统一配置
  *          （PB8 在 HAL_TIM_MspPostInit 中配为复用推挽，PB9 在 MX_GPIO_Init
  *          中配为推挽输出），本模块只写电平/比较值，不重复初始化外设。
  *          接口均为非阻塞操作，可在任务上下文直接调用；内部用临界区保护
  *          "运行标志 + 速度设定"的读-改-写，避免多任务并发错乱。
  ******************************************************************************
  */
#ifndef __App_Motor_H__
#define __App_Motor_H__

#include <stdint.h>

/**
  * @brief  电机模块初始化：固定正转方向、置停止状态并创建 PID 控制任务
  * @note   须在 vTaskStartScheduler() 之前调用，且 MX_TIM4_Init() 已完成。
  *         动作：IN2（PB9）写固定低电平（方向=正转）；TIM4_CH3 启动 PWM
  *         输出并将比较值置 0（IN1 恒低 → 电机不转，与被控设备安全状态
  *         一致）；内部运行标志与速度设定清零；随后创建 "MotorPid" 控制
  *         任务（100ms 周期）：系统 RUN → 按目标转速 PID 闭环调速
  *         （前馈 + PI，反馈取编码器实测转速）；IDLE / FAULT → 自动停机。
  *         RUN 态下 PID 任务是电机速度的唯一写者，手动 SetSpeed 会被覆盖。
  * @retval 1=成功，0=失败（控制任务创建失败）
  */
uint8_t Motor_Init(void);

/**
  * @brief  启动电机（开始）：按当前设定速度输出 PWM
  * @note   幂等，重复调用无副作用；若尚未设定过速度（设定值为 0），
  *         则按 MOTOR_DEFAULT_SPEED_PCT（默认 50%）启动，保证"开始"
  *         一定有可见动作；设定速度非 0 时沿用上次设定值。
  * @retval 无
  */
void Motor_Start(void);

/**
  * @brief  暂停电机（停止）：占空比清零并关闭 PWM 输出
  * @note   幂等；IN1/IN2 同时为低 → DRV8870 进入滑行（coast），电机
  *         按惯性自由减速，无反向制动冲击。速度设定值保留，下次
  *         Motor_Start() 按原速度恢复。
  * @retval 无
  */
void Motor_Stop(void);

/**
  * @brief  更新电机速度（占空比百分比）
  * @note   运行中立即生效（改写 TIM4_CH3 比较值）；停止状态下仅记录，
  *         下次启动时生效。入参 > 100 按 100 处理（饱和限幅），
  *         0 等同于"停转"——运行中设为 0 时电机滑行停止，但仍保持
  *         "运行"标志，之后设非 0 值可立即恢复转动。
  * @param  percent 占空比 0~100（%）
  * @retval 无
  */
void Motor_SetSpeed(uint8_t percent);

/**
  * @brief  查询电机当前设定速度
  * @retval 占空比 0~100（%）
  */
uint8_t Motor_GetSpeed(void);

/**
  * @brief  查询电机是否处于运行状态
  * @retval 1=运行（已启动且速度非 0），0=停止
  */
uint8_t Motor_IsRunning(void);

#endif /* __App_Motor_H__ */

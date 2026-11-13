/**
  ******************************************************************************
  * @file    App_Buzzer.h
  * @brief   蜂鸣器应用层接口（专用任务 + 命令队列方案）
  * @note    蜂鸣器为无源类型，由 PA8（TIM1_CH1）输出 PWM 方波驱动。
  *          业务代码不要直接操作 TIM1 或 GPIO，统一通过 Buzzer_Beep() 投递
  *          鸣叫命令，由蜂鸣器任务排队执行，避免多任务争抢。
  ******************************************************************************
  */
#ifndef __APP_BUZZER_H__
#define __APP_BUZZER_H__

#include <stdint.h>

/* 鸣叫命令类型：每种命令对应一种声效 */
typedef enum
{
  BEEP_SHORT = 0,   /* 短鸣：响 100ms */
  BEEP_LONG  = 1,   /* 长鸣：响 1000ms */
} BeepCmd_t;

/**
  * @brief  蜂鸣器模块初始化：创建鸣叫命令队列和蜂鸣器任务
  * @note   必须在 vTaskStartScheduler() 之前调用（创建队列/任务均合法），
  *         蜂鸣器任务在调度器启动后自动运行等待命令。
  * @retval 1=成功，0=失败（队列或任务创建失败）
  */
uint8_t Buzzer_Init(void);

/**
  * @brief  向蜂鸣器任务投递一次鸣叫命令（非阻塞）
  * @note   仅可在任务上下文或允许调用 RTOS API 的中断中调用（调度器启动后）。
  *         命令进入队列排队执行；队列满时投递失败直接返回，不阻塞调用者。
  * @param  cmd 鸣叫命令：BEEP_SHORT 短鸣 100ms，BEEP_LONG 长鸣 1000ms
  * @retval 1=投递成功，0=投递失败（队列满）
  */
uint8_t Buzzer_Beep(BeepCmd_t cmd);

#endif /* __APP_BUZZER_H__ */

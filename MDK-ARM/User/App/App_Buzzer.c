/**
  ******************************************************************************
  * @file    App_Buzzer.c
  * @brief   蜂鸣器应用层实现（专用任务 + 命令队列方案）
  * @note    硬件连接：无源蜂鸣器 <- PA8（TIM1_CH1 PWM，CubeMX 已配置
  *          4kHz 方波、Pulse=0 上电静音）。
  *          时基说明：TIM1 挂在 APB2（72MHz），PWM 频率 = 72MHz/(17999+1) = 4kHz，
  *          开声时将比较值设为 9000（约 50% 占空比）获得最大声压。
  ******************************************************************************
  */
#include "App_Buzzer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "tim.h"

/*------------------------------ 私有宏定义 -------------------------------*/
#define BEEP_TASK_STACK_SIZE   128u   /* 蜂鸣器任务栈深度（单位：字，128字=512字节） */
#define BEEP_TASK_PRIORITY     2u     /* 蜂鸣器任务优先级（低于关键控制任务） */
#define BEEP_QUEUE_LENGTH      8u     /* 鸣叫命令队列深度（最多缓存 8 次鸣叫请求） */
#define BEEP_SHORT_MS          100u   /* 短鸣持续时间（ms） */
#define BEEP_LONG_MS           1000u  /* 长鸣持续时间（ms） */
#define BEEP_GAP_MS            100u   /* 连续两次鸣叫之间的静音间隔（ms） */
#define BEEP_PWM_COMPARE_ON    9000u  /* 开声比较值：(17999+1)/2，约 50% 占空比 */

/*------------------------------ 私有变量 ---------------------------------*/
static QueueHandle_t s_beepQueue = NULL;  /* 鸣叫命令队列句柄 */

/*------------------------------ 私有函数 ---------------------------------*/
static void Buzzer_Task(void *argument);
static void Buzzer_SoundOn(void);
static void Buzzer_SoundOff(void);

/**
  * @brief  蜂鸣器模块初始化：创建鸣叫命令队列和蜂鸣器任务
  * @note   在 vTaskStartScheduler() 之前调用；调用前 MX_TIM1_Init() 必须已完成。
  * @retval 1=成功，0=失败（队列或任务创建失败）
  */
uint8_t Buzzer_Init(void)
{
  /* 创建鸣叫命令队列：缓存各任务/中断投递的声效请求 */
  s_beepQueue = xQueueCreate(BEEP_QUEUE_LENGTH, sizeof(BeepCmd_t));
  if (s_beepQueue == NULL)
  {
    return 0;   /* 队列创建失败（FreeRTOS 堆不足） */
  }

  /* 创建蜂鸣器任务：循环取出命令并按声效时间表鸣叫 */
  if (xTaskCreate(Buzzer_Task, "Buzzer", BEEP_TASK_STACK_SIZE, NULL,
                  BEEP_TASK_PRIORITY, NULL) != pdPASS)
  {
    return 0;   /* 任务创建失败 */
  }

  return 1;     /* 初始化成功 */
}

/**
  * @brief  向蜂鸣器任务投递一次鸣叫命令（非阻塞）
  * @param  cmd 鸣叫命令：BEEP_SHORT 短鸣 100ms，BEEP_LONG 长鸣 1000ms
  * @retval 1=投递成功，0=投递失败（队列满）
  */
uint8_t Buzzer_Beep(BeepCmd_t cmd)
{
  if (s_beepQueue == NULL)
  {
    return 0;   /* 模块未初始化 */
  }

  /* 超时为 0：队列满时不阻塞调用者，直接报失败 */
  if (xQueueSend(s_beepQueue, &cmd, 0) != pdPASS)
  {
    return 0;
  }
  return 1;     /* 投递成功 */
}

/**
  * @brief  蜂鸣器任务：等待命令队列，按声效时间表执行鸣叫
  * @note   执行流程：取命令 -> 开声 -> 延时对应时长 -> 关声 ->
  *         若队列还有后续命令则插入静音间隔，区分连续两次鸣叫。
  * @param  argument 未使用
  * @retval 无（任务永不返回）
  */
static void Buzzer_Task(void *argument)
{
  (void)argument;
  BeepCmd_t cmd;

  for (;;)
  {
    /* 阻塞等待鸣叫命令，无命令时任务挂起不占用 CPU */
    if (xQueueReceive(s_beepQueue, &cmd, portMAX_DELAY) == pdPASS)
    {
      /* 开声并保持对应时长 */
      Buzzer_SoundOn();

      switch (cmd)
      {
        case BEEP_SHORT:
          vTaskDelay(pdMS_TO_TICKS(BEEP_SHORT_MS));
          break;

        case BEEP_LONG:
          vTaskDelay(pdMS_TO_TICKS(BEEP_LONG_MS));
          break;

        default:
          /* 未知命令按短鸣处理，保证必关声 */
          vTaskDelay(pdMS_TO_TICKS(BEEP_SHORT_MS));
          break;
      }

      /* 关声，结束本次鸣叫 */
      Buzzer_SoundOff();

      /* 队列中还有后续命令时插入静音间隔，避免两次鸣叫连成一片 */
      if (uxQueueMessagesWaiting(s_beepQueue) > 0u)
      {
        vTaskDelay(pdMS_TO_TICKS(BEEP_GAP_MS));
      }
    }
  }
}

/**
  * @brief  开启蜂鸣器发声：启动 TIM1_CH1 PWM 输出并将比较值设为 50% 占空比
  * @note   仅由蜂鸣器任务调用；PWM 频率 4kHz 由 CubeMX 时基配置决定。
  * @retval 无
  */
static void Buzzer_SoundOn(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BEEP_PWM_COMPARE_ON);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

/**
  * @brief  关闭蜂鸣器发声：停止 TIM1_CH1 PWM 输出并将比较值清零
  * @note   仅由蜂鸣器任务调用；停止输出后 PA8 不再驱动方波，蜂鸣器静音。
  * @retval 无
  */
static void Buzzer_SoundOff(void)
{
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
}

/**
  ******************************************************************************
  * @file    App_Fan.c
  * @brief   风扇控制模块实现（PA1 = 原理图网络名 MOTOR_ON，高电平有效）
  * @note    只做"使能脚电平"与"软件状态标志"的成对维护：
  *            启动 → PA1 = MOTOR_ON_ACTIVE_STATE（高），s_running = 1；
  *            停止 → PA1 = MOTOR_ON_INACTIVE_STATE（低），s_running = 0。
  *          引脚网络名与有效电平统一取自 bsp_pins.h，不硬编码端口/电平；
  *          临界区仅包裹"状态标志读-改-写"，内部无阻塞调用，开销极小。
  *          自动跟随：Fan_Init() 创建"风扇控制任务"，每 200ms 读取系统
  *          状态并联动风扇——RUN → 启动；IDLE / FAULT → 停止。状态只在
  *          跳变时写一次 GPIO，稳态下零总线/零额外开销；系统上电即空闲，
  *          风扇保持停止。
  ******************************************************************************
  */
#include "App_Fan.h"
#include "App_SysTem.h"
#include "bsp_pins.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

/*------------------------------ 私有宏定义 ---------------------------------*/
#define FAN_CTRL_PERIOD_MS   200u   /* 状态联动轮询周期（ms） */

/*------------------------------ 私有变量 ---------------------------------*/
static uint8_t s_running = 0u;   /* 风扇运行状态标志：0=停止，1=运行 */

/*------------------------------ 私有函数 ---------------------------------*/
static void FanCtrlTask(void *argument);

/**
  * @brief  风扇状态联动任务：按系统状态自动启停风扇
  * @note   周期 200ms 读取 SysTem 状态：
  *           SYS_STATE_RUN   → Fan_Start()（运行态吹风散热）；
  *           SYS_STATE_IDLE  → Fan_Stop()（空闲停止）；
  *           SYS_STATE_FAULT → Fan_Stop()（故障安全停机）。
  *         启停接口本身幂等，仅状态跳变时才真正写 GPIO；读状态走
  *         SysTem_GetState()（内部临界区），与主控任务状态写入无竞争。
  * @param  argument 未使用
  * @retval 无
  */
static void FanCtrlTask(void *argument)
{
    (void)argument;
    SysRunState_t state;
    uint8_t       lastRunning = 0xFFu;   /* 初值取非法值，保证首轮必执行一次 */

    for (;;)
    {
        state = SysTem_GetState();

        if (state == SYS_STATE_RUN)
        {
            if (lastRunning != 1u)
            {
                Fan_Start();                 /* 运行态 → 启动风扇 */
                lastRunning = 1u;
                printf("[Fan] system RUN -> fan on\r\n");
            }
        }
        else                                 /* IDLE 或 FAULT → 停止风扇 */
        {
            if (lastRunning != 0u)
            {
                Fan_Stop();                  /* 非运行态 → 停止风扇 */
                lastRunning = 0u;
                printf("[Fan] system %s -> fan off\r\n",
                       (state == SYS_STATE_FAULT) ? "FAULT" : "IDLE");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(FAN_CTRL_PERIOD_MS));
    }
}

/**
  * @brief  风扇模块初始化：置为停止状态并创建状态联动任务
  * @note   显式拉低使能脚并清零状态标志，保证软件状态与硬件一致；
  *         随后创建"FanCtrl"任务（优先级 2、栈 128 字）实现
  *         "系统运行 → 风扇启动，系统空闲 → 风扇停止"的自动跟随。
  *         须在 vTaskStartScheduler() 之前调用（创建任务合法），
  *         任务在调度器启动后自动运行。
  * @retval 1=成功，0=失败（任务创建失败）
  */
uint8_t Fan_Init(void)
{
    HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, MOTOR_ON_INACTIVE_STATE);
    s_running = 0u;

    if (xTaskCreate(FanCtrlTask, "FanCtrl", 128u, NULL, 2u, NULL) != pdPASS)
    {
        printf("[Fan] ctrl task create failed!\r\n");
        return 0u;
    }
    return 1u;
}

/**
  * @brief  启动风扇：使能脚输出有效电平
  * @note   幂等：已在运行时直接返回，不重复写引脚。
  * @retval 无
  */
void Fan_Start(void)
{
    taskENTER_CRITICAL();
    if (s_running == 0u)
    {
        HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, MOTOR_ON_ACTIVE_STATE);
        s_running = 1u;
    }
    taskEXIT_CRITICAL();
}

/**
  * @brief  停止/暂停风扇：使能脚输出无效电平
  * @note   幂等：已停止时直接返回，不重复写引脚。
  * @retval 无
  */
void Fan_Stop(void)
{
    taskENTER_CRITICAL();
    if (s_running != 0u)
    {
        HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, MOTOR_ON_INACTIVE_STATE);
        s_running = 0u;
    }
    taskEXIT_CRITICAL();
}

/**
  * @brief  切换风扇状态（运行 ↔ 停止）
  * @note   读-改-写在临界区内完成，避免多任务并发切换时状态标志与
  *         GPIO 电平不一致。
  * @retval 无
  */
void Fan_Toggle(void)
{
    taskENTER_CRITICAL();
    if (s_running == 0u)
    {
        HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, MOTOR_ON_ACTIVE_STATE);
        s_running = 1u;
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, MOTOR_ON_INACTIVE_STATE);
        s_running = 0u;
    }
    taskEXIT_CRITICAL();
}

/**
  * @brief  查询风扇是否运行
  * @retval 1=运行中，0=已停止
  */
uint8_t Fan_IsRunning(void)
{
    return s_running;
}

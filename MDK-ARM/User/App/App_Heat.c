/**
  ******************************************************************************
  * @file    App_Heat.c
  * @brief   加热控制模块实现（PA0 TIM2_CH1 PWM + UCC27517 门极驱动 + PID 闭环）
  * @note    ── 硬件真值 ────────────────────────────────────────────────────
  *          UCC27517 为双输入低边门极驱动器：OUT = IN+ 与非 IN−，
  *            IN− = 0 → OUT 跟随 IN+（PA0 的 1kHz PWM）→ 正常加热；
  *            IN− = 1 → OUT 强制低 → 硬件切断（U21 过流锁存触发时）。
  *          74LVC1G74 的 R#D 接 MCU 的 FAULT_RST_N（PC2）：写低复位触发器
  *          （Q→0，驱动恢复使能），写高释放（外部上拉维持高电平）。
  *          HEAT_FAULT_N（PC3）为 U21 锁存状态回读：低 = 硬件故障已锁存。
  *          ── PWM 时基 ────────────────────────────────────────────────
  *          TIM2：72MHz/72 = 1MHz 计数，ARR = 999 → PWM 频率 1kHz，
  *          分辨率 1000 级 → 1% 占空比 = 10 个计数（CCR = percent × 10）。
  *          1kHz 对加热丝/发热膜而言等效直流，无机械噪声问题。
  *          ── PID 设计 ────────────────────────────────────────────────
  *          控制律：u = Kp×e + ∫Ki×e（e = 目标水温 − 实测水温）。
  *          加热对象特点：单向输出（只能加热不能制冷）、时间常数分钟级、
  *          滞后大 → 用 PI 即可：Kp 提供主要驱动功率，Ki 消除保温稳态
  *          误差；无前馈（保温功率随环境变化，交给积分自适应）；微分
  *          关闭（温度信号平滑但对象慢，微分易引入噪声）。积分限幅
  *          + 条件积分抗饱和（输出顶格且误差同向时停止积分）。
  *          ── 安全保护 ────────────────────────────────────────────────
  *          任一触发 → 强制 0% 输出（见 .h 文件头）：水温探头故障、
  *          水温绝对上限、板温上限、硬件故障锁存。锁存故障时软件周期
  *          性尝试 FAULT_RST_N 复位（2s 一次），真故障会立即再次锁存，
  *          输出保持 0；偶发误锁存则自动恢复，无需人工干预。
  ******************************************************************************
  */
#include "App_Heat.h"
#include "App_SysTem.h"
#include "App_ADC.h"
#include "bsp_pins.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

/*------------------------------ 私有宏定义 ---------------------------------*/
#define HEAT_PWM_CNT_PER_PCT    10u     /* 占空比换算：1% = 10 个计数（ARR=999） */
#define HEAT_PWM_PERIOD_CNT     1000u   /* PWM 一个周期计数（ARR+1） */

#define HEAT_DEFAULT_POWER_PCT  50u     /* 未预设功率时"开始"的默认占空比（%） */
#define HEAT_MIN_POWER_PCT      1u      /* 有效功率下限（避免 0 与"停止"语义混淆） */

/*------------------------------ PID 闭环参数 -------------------------------*/
#define HEAT_CTRL_PERIOD_MS     500u    /* 控制周期（ms）：热惯性大，500ms 足够 */
#define HEAT_TARGET_MAX         99.0f   /* 目标水温上限（°C），超限饱和（沸点以下留裕量） */
#define HEAT_TARGET_MIN         0.1f    /* 目标水温下限：低于此值视为"要求停止加热" */
#define HEAT_PID_KP             6.0f    /* 比例系数（%/°C）：误差 17°C 即满功率 */
#define HEAT_PID_KI             0.05f   /* 积分系数（%/°C/周期）：误差 10°C →
                                         * 每周期 +0.5%，保温段自适应稳态功率 */
#define HEAT_PID_KD             0.0f    /* 微分系数：默认关闭（对象慢、滞后大） */
#define HEAT_PID_INTEG_MAX      60.0f   /* 积分项限幅（%），防积分饱和 */
#define HEAT_DEADBAND_C         0.3f    /* 保温死区（°C）：误差小于此值输出 0，
                                         * 避免热惯性导致的来回小振荡 */

#define HEAT_ABS_MAX_C          95.0f   /* 水温绝对上限（°C）：超温强制停止 */
#define HEAT_BOARD_MAX_C        75.0f   /* 板温上限（°C）：主板过热保护 */
#define HEAT_FAULT_RETRY_CYCLES 4u      /* 硬件锁存故障复位重试间隔（×500ms=2s） */

#define HEAT_TASK_STACK         128u    /* PID 控制任务栈深度（字） */
#define HEAT_TASK_PRIO          2u      /* PID 控制任务优先级（与电机/编码器任务同级） */

/*------------------------------ 私有变量 ---------------------------------*/
static uint8_t s_running  = 0u;   /* 运行标志：0=停止，1=加热 */
static uint8_t s_powerPct = 0u;   /* 当前设定功率（占空比 0~100%） */

/* PID 控制器内部状态（仅 HeatCtrlTask 访问，无需加锁） */
static float   s_pidInteg   = 0.0f;  /* 积分累加值（占空比 %） */
static float   s_pidPrevErr = 0.0f;  /* 上一周期误差（°C，供微分项） */
static uint8_t s_pidActive  = 0u;    /* 闭环激活标志：用于状态跳变日志与复位 */
static uint8_t s_hwFault    = 0u;    /* 硬件故障锁存状态：1=锁存中 */
static uint8_t s_faultRetry = 0u;    /* 锁存故障复位重试计数 */

/*------------------------------ 私有函数 ---------------------------------*/
static void Heat_SetCompare(uint8_t percent);
static void HeatCtrlTask(void *argument);

/**
  * @brief  加热模块初始化：清除硬件锁存、启动 PWM（0%）并创建控制任务
  * @note   1) FAULT_RST_N 写低脉冲（>1µs 即满足 74LVC1G74 复位脉宽）复位
  *            触发器使 Q=0（UCC27517 的 IN−=0，驱动使能），再释放回高；
  *         2) TIM2_CH1 启动 PWM 输出且比较值=0（IN+=0 → 电机... 加热静止）；
  *         3) 运行标志与功率设定清零；
  *         4) 创建 "HeatCtrl" 控制任务：系统 RUN → PID 闭环加热，
  *            IDLE / FAULT → 停止输出（上电默认空闲，加热关闭）。
  *         须在 vTaskStartScheduler() 之前、MX_TIM2_Init() 之后调用。
  * @retval 1=成功，0=失败（控制任务创建失败）
  */
uint8_t Heat_Init(void)
{
    /* 清除 U21 硬件故障锁存：复位脉冲后释放（外部上拉维持高电平） */
    HAL_GPIO_WritePin(FAULT_RST_N_GPIO_Port, FAULT_RST_N_Pin, FAULT_RST_ASSERT_STATE);
    HAL_GPIO_WritePin(FAULT_RST_N_GPIO_Port, FAULT_RST_N_Pin, FAULT_RST_RELEASE_STATE);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0u);
    (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    taskENTER_CRITICAL();
    s_running    = 0u;
    s_powerPct   = 0u;
    s_pidInteg   = 0.0f;
    s_pidPrevErr = 0.0f;
    s_pidActive  = 0u;
    s_hwFault    = (uint8_t)((HAL_GPIO_ReadPin(HEAT_FAULT_N_GPIO_Port,
                                               HEAT_FAULT_N_Pin) == HEAT_FAULT_ACTIVE_STATE) ? 1u : 0u);
    s_faultRetry = 0u;
    taskEXIT_CRITICAL();

    if (xTaskCreate(HeatCtrlTask, "HeatCtrl", HEAT_TASK_STACK, NULL,
                    HEAT_TASK_PRIO, NULL) != pdPASS)
    {
        return 0u;                /* 任务创建失败（FreeRTOS 堆不足） */
    }

    return 1u;
}

/**
  * @brief  加热开始（手动接口）：按当前设定功率输出 PWM
  * @note   RUN 态下 PID 任务会按目标水温覆盖功率，本接口主要用于调试。
  * @retval 无
  */
void Heat_Start(void)
{
    uint8_t power;

    taskENTER_CRITICAL();
    power = (s_powerPct < HEAT_MIN_POWER_PCT) ? (uint8_t)HEAT_DEFAULT_POWER_PCT
                                              : s_powerPct;
    s_powerPct = power;
    s_running  = 1u;
    taskEXIT_CRITICAL();

    Heat_SetCompare(power);
    (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   /* 恢复输出（Stop 时已关闭） */
}

/**
  * @brief  加热停止（手动接口）：比较值清零并关闭 PWM 输出
  * @note   设定功率保留，下次 Heat_Start 原功率恢复。
  * @retval 无
  */
void Heat_Stop(void)
{
    taskENTER_CRITICAL();
    s_running = 0u;
    taskEXIT_CRITICAL();

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0u);
    (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

/**
  * @brief  更新加热功率（手动接口）
  * @note   运行中立即改写比较值生效；停止中仅记录（下次 Start 生效）。
  * @param  percent 功率占空比（0~100，>100 饱和到 100）
  * @retval 无
  */
void Heat_SetPower(uint8_t percent)
{
    if (percent > 100u)
    {
        percent = 100u;
    }

    taskENTER_CRITICAL();
    s_powerPct = percent;
    taskEXIT_CRITICAL();

    if (s_running != 0u)
    {
        Heat_SetCompare(percent);
    }
}

/**
  * @brief  读取当前设定加热功率
  * @retval 功率占空比（%）
  */
uint8_t Heat_GetPower(void)
{
    uint8_t power;

    taskENTER_CRITICAL();
    power = s_powerPct;
    taskEXIT_CRITICAL();

    return power;
}

/**
  * @brief  查询是否正在加热
  * @retval 1=已启动且功率≥1%，0=停止
  */
uint8_t Heat_IsRunning(void)
{
    uint8_t running;

    taskENTER_CRITICAL();
    running = ((s_running == 1u) && (s_powerPct >= HEAT_MIN_POWER_PCT)) ? 1u : 0u;
    taskEXIT_CRITICAL();

    return running;
}

/**
  * @brief  写 PWM 比较值（内部）：占空比% → TIM2 计数
  * @note   CCR = percent × 10，1kHz PWM，带周期上限兜底。
  * @param  percent 功率占空比（%）
  * @retval 无
  */
static void Heat_SetCompare(uint8_t percent)
{
    uint32_t compare = (uint32_t)percent * HEAT_PWM_CNT_PER_PCT;

    if (compare > HEAT_PWM_PERIOD_CNT)
    {
        compare = HEAT_PWM_PERIOD_CNT;           /* 兜底：不超过一个周期 */
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, compare);
}

/**
  * @brief  PID 温度控制任务：目标水温 → PI → 加热占空比输出
  * @note   控制律：u = Kp×e + ∫Ki×e，e = 目标水温 − 实测水温；
  *         保温死区 ±0.3°C：误差小于死区输出 0，防止热惯性来回振荡。
  *         状态联动：
  *           SYS_STATE_RUN   → 闭环加热（目标 > 0 才输出）；
  *           SYS_STATE_IDLE  → 停止 + 控制器复位；
  *           SYS_STATE_FAULT → 停止 + 控制器复位（故障安全）。
  *         安全保护（强制 0% 输出，恢复后自动继续）：
  *           水温探头故障 / 水温 ≥ HEAT_ABS_MAX_C / 板温 ≥ HEAT_BOARD_MAX_C；
  *         硬件故障锁存（HEAT_FAULT_N 低有效）：强制 0% 输出，每 2s 尝试
  *         FAULT_RST_N 复位一次；真故障立即再锁存（输出保持 0），误锁存
  *         则自动恢复。
  *         RUN 态下本任务是加热功率的唯一写者：每周期无条件 SetCompare，
  *         手动 Start/SetPower 的效果会被覆盖。
  * @param  argument 未使用
  * @retval 无（任务永不返回）
  */
static void HeatCtrlTask(void *argument)
{
    TickType_t    lastWake = xTaskGetTickCount();
    SysRunState_t state;
    float target;
    float current;
    float board;
    float err;
    float uOut;

    (void)argument;

    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(HEAT_CTRL_PERIOD_MS));

        /* ---------- 状态联动：非运行态一律停止并复位控制器 ---------- */
        state = SysTem_GetState();
        if (state != SYS_STATE_RUN)
        {
            if (s_pidActive != 0u)
            {
                Heat_Stop();
                s_pidInteg   = 0.0f;
                s_pidPrevErr = 0.0f;
                s_pidActive  = 0u;
                printf("[Heat] system %s -> heat off\r\n",
                       (state == SYS_STATE_FAULT) ? "FAULT" : "IDLE");
            }
            continue;
        }

        /* ---------- 目标水温处理：0 视为停止加热，超限饱和 ---------- */
        target = SysTem_GetTargetTemp();
        if (target < HEAT_TARGET_MIN)
        {
            if (s_pidActive != 0u)
            {
                Heat_Stop();
                s_pidInteg   = 0.0f;
                s_pidPrevErr = 0.0f;
                s_pidActive  = 0u;
                printf("[Heat] target=0 -> heat off\r\n");
            }
            continue;
        }
        if (target > HEAT_TARGET_MAX)
        {
            target = HEAT_TARGET_MAX;             /* 超出上限按上限处理 */
        }

        /* ---------- 安全保护检查：任一触发即强制 0% ---------- */
        current = SysTem_GetCurrentTemp();        /* 水温（ADC 任务唯一写者） */
        board   = Adc_GetBoardTempC();

        if ((Adc_GetWaterFault() != 0u) ||
            (current >= HEAT_ABS_MAX_C) ||
            (board   >= HEAT_BOARD_MAX_C))
        {
            if (s_pidActive != 0u)
            {
                s_pidInteg   = 0.0f;
                s_pidPrevErr = 0.0f;
                s_pidActive  = 0u;
                Heat_Stop();
                printf("[Heat] safety stop: waterFault=%u water=%.1f board=%.1f\r\n",
                       (unsigned int)Adc_GetWaterFault(),
                       (double)current, (double)board);
            }
            else
            {
                Heat_SetCompare(0u);              /* 保护持续期间保持 0% */
            }
            continue;
        }

        /* ---------- 硬件故障锁存：强制 0% + 周期性复位重试 ---------- */
        s_hwFault = (uint8_t)((HAL_GPIO_ReadPin(HEAT_FAULT_N_GPIO_Port,
                                                HEAT_FAULT_N_Pin) == HEAT_FAULT_ACTIVE_STATE) ? 1u : 0u);
        if (s_hwFault != 0u)
        {
            Heat_SetCompare(0u);
            s_pidInteg   = 0.0f;
            s_pidPrevErr = 0.0f;
            s_pidActive  = 0u;

            s_faultRetry++;
            if (s_faultRetry >= HEAT_FAULT_RETRY_CYCLES)
            {
                s_faultRetry = 0u;
                /* 低脉冲复位 U21 锁存：真故障会立即再次锁存（输出仍为 0），
                 * 偶发误锁存则自动恢复，下个周期重新进入闭环 */
                HAL_GPIO_WritePin(FAULT_RST_N_GPIO_Port, FAULT_RST_N_Pin, FAULT_RST_ASSERT_STATE);
                HAL_GPIO_WritePin(FAULT_RST_N_GPIO_Port, FAULT_RST_N_Pin, FAULT_RST_RELEASE_STATE);
                printf("[Heat] hw latch fault, reset attempted\r\n");
            }
            continue;
        }
        s_faultRetry = 0u;

        /* ---------- PI 计算：比例 + 积分（抗饱和） ---------- */
        err = target - current;
        if (err < HEAT_DEADBAND_C)
        {
            /* 误差在死区内（到达/略过冲）：输出 0 靠自然散热，
             * 积分清零避免保温段热惯性导致的来回小振荡 */
            s_pidInteg   *= 0.5f;                 /* 积分半衰，防止深饱和 */
            s_pidPrevErr  = err;
            uOut          = 0.0f;
        }
        else
        {
            /* 输出饱和且误差继续同向时停止积分，防止积分深饱和 */
            float uNoI = HEAT_PID_KP * err + HEAT_PID_KD * (err - s_pidPrevErr);

            if (!(((uNoI + s_pidInteg) > 100.0f) && (err > 0.0f)))
            {
                s_pidInteg += HEAT_PID_KI * err;
                if (s_pidInteg > HEAT_PID_INTEG_MAX)
                {
                    s_pidInteg = HEAT_PID_INTEG_MAX;
                }
                else if (s_pidInteg < 0.0f)
                {
                    s_pidInteg = 0.0f;            /* 加热单向输出，积分无负值 */
                }
            }
            s_pidPrevErr = err;

            uOut = uNoI + s_pidInteg;
            if (uOut > 100.0f)
            {
                uOut = 100.0f;
            }
            else if (uOut < 0.0f)
            {
                uOut = 0.0f;
            }
        }

        /* ---------- 输出：未启动先启动，再更新占空比 ---------- */
        if (s_pidActive == 0u)
        {
            s_pidActive = 1u;
            Heat_Start();                         /* 首次进入闭环：启动输出 */
            printf("[Heat] PID start: target=%.1fC\r\n", (double)target);
        }
        else if (Heat_IsRunning() == 0u)
        {
            Heat_Start();                         /* 输出通道被外部关闭时恢复 */
        }
        Heat_SetCompare((uint8_t)uOut);
    }
}

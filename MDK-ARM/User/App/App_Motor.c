/**
 ******************************************************************************
 * @file    App_Motor.c
 * @brief   电机应用层实现（DRV8870DDAR，单向正转 + PWM 调速）
 * @note    只做"PWM 比较值 + 运行标志"的成对维护：
 *            启动 → TIM4_CH3 输出设定占空比，s_running = 1；
 *            停止 → 比较值清零并关闭输出，s_running = 0；
 *            调速 → 比较值 = 百分比 × 每档计数，运行中立即生效。
 *          方向固定正转：IN2（PB9）在上电初始化时即为低电平，本模块
 *          初始化再显式写一次，运行过程中不再改动（只调速不换向）。
 *          引脚网络名取自 bsp_pins.h，比较值换算基于 tim.c 的 ARR 配置，
 *          不硬编码端口引脚；临界区仅包裹"标志读-改-写"，无阻塞调用。
 *
 *          PID 转速闭环（MotorCtrlTask，100ms 周期）：
 *            系统状态联动：RUN → 闭环运行；IDLE / FAULT → 停机；
 *            前馈（FF）：u_ff = 死区占空比 + (100−死区)×目标/3400。
 *                        直流电机带负载存在摩擦死区（低于约 30% 不转），
 *                        纯线性前馈在低速目标下严重偏小，起动后要靠积分
 *                        慢慢磨到真实工作点（表现为几十秒才达标）；引入
 *                        死区补偿后，输出直接落在真实工作点附近。
 *            反馈  (PI)：目标转速 - 编码器实测转速 → 误差，
 *                        u += Kp×e + Ki×∑e（位置式，积分限幅 + 抗饱和）；
 *                        Kd 默认 0（编码器量化步进 75rpm，微分放大噪声）；
 *            输出限幅 [0,100]%，低于最小有效占空比按 0 处理；
 *            静摩擦补偿：实测转速低于起动阈值时强制输出起动 Kick 占空比，
 *            转起来后交还 PID（同时起动阶段冻结积分，防起动过冲）。
 *            RUN 态下本任务是电机速度的唯一写者，Motor_SetSpeed 的
 *            手动调用会被下个控制周期覆盖。
 ******************************************************************************
 */
#include "App_Motor.h"
#include "App_SysTem.h"
#include "bsp_pins.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

/*------------------------------ 私有宏定义 ---------------------------------*/
/* TIM4 时基：72MHz/(3599+1) = 20kHz，比较值 0~3599 对应 0~100% 占空比，
 * 故每 1% 对应 36 个计数（3600/100）。 */
#define MOTOR_PWM_PERIOD_CNT 3600u /* ARR + 1（自动重装周期总计数） */
#define MOTOR_PWM_CNT_PER_PCT 36u  /* 每 1% 占空比对应的比较值 */

#define MOTOR_DEFAULT_SPEED_PCT 50u /* 未预设速度时"开始"的默认占空比（%） */
#define MOTOR_MIN_SPEED_PCT 1u      /* 有效速度下限（避免 0 与"停止"语义混淆） */

/*------------------------------ PID 闭环参数 -------------------------------*/
#define MOTOR_CTRL_PERIOD_MS 100u /* 控制周期（ms），与编码器采样窗口对齐 */
#define MOTOR_RPM_MAX 3400.0f     /* 负载实测可达最高转速（对应 100% 占空比） */
#define MOTOR_RPM_TARGET_MIN 1.0f /* 目标转速下限：低于此值视为"要求停转" */
#define MOTOR_DUTY_DEADZONE 30.0f /* 摩擦死区占空比（%）：低于此值电机不转。 \
                                   * 标定方法：稳定运行时看调试打印的 u 值，  \
                                   * 死区 = u − (100−30)×目标/3400 ≈ u − 6         \
                                   * （例：300rpm 稳态 u=36% → 死区≈30） */
#define MOTOR_PID_KP 0.01f        /* 比例系数（占空比%/rpm）：误差 500rpm → +5%。 \
                                   * 前馈已含死区补偿，比例只需修正残差 */
#define MOTOR_PID_KI 0.001f       /* 积分系数（占空比%/rpm/周期）：误差 100rpm \
                                   * → 每周期 +0.1%，约 10s 消除稳态误差 */
#define MOTOR_PID_KD 0.0f         /* 微分系数：默认关闭（反馈量化步进 75rpm） */
#define MOTOR_PID_INTEG_MAX 30.0f /* 积分项限幅（占空比%），防积分饱和 */
#define MOTOR_DUTY_MIN 2u         /* 最小有效占空比：低于此值电机不转，输出按 0 处理 */

#define MOTOR_STALL_RPM 50.0f    /* 起动判定阈值：实测低于此转速视为"未转起来" */
#define MOTOR_DUTY_BREAKAWAY 20u /* 起动 Kick 基准占空比（%） */
#define MOTOR_STALL_RAMP_PCT 1u  /* 每个未起动周期 Kick 爬坡增量（%）： \
                                  * 20% 起步、每 100ms +1%，1s 后 30%，   \
                                  * 兜底突破更高的静摩擦阈值 */
#define MOTOR_DUTY_KICK_MAX 60u  /* Kick 爬坡上限（%），防止无限加压 */

#define MOTOR_DEBUG_EN 1 /* 1=每秒打印一次 PID 内部状态（调试用， \
                          * 调试完成后改 0 关闭打印） */

#define MOTOR_TASK_STACK 128u /* PID 控制任务栈深度（字） */
#define MOTOR_TASK_PRIO 2u    /* PID 控制任务优先级（与风扇/编码器任务同级） */

/*------------------------------ 私有变量 ---------------------------------*/
static uint8_t s_running = 0u;  /* 运行标志：0=停止，1=运行 */
static uint8_t s_speedPct = 0u; /* 当前设定速度（占空比 0~100%） */

/* PID 控制器内部状态（仅 MotorCtrlTask 访问，无需加锁） */
static float s_pidInteg = 0.0f;     /* 积分累加值（占空比 %） */
static float s_pidPrevErr = 0.0f;   /* 上一周期误差（rpm，供微分项） */
static uint8_t s_pidActive = 0u;    /* 闭环激活标志：用于状态跳变日志与复位 */
static uint16_t s_stallCycles = 0u; /* 连续未转起来的周期数（Kick 爬坡用） */

/*------------------------------ 私有函数 ---------------------------------*/
static void Motor_SetCompare(uint8_t percent);
static void MotorCtrlTask(void *argument);

/**
 * @brief  电机模块初始化：固定方向、置停止状态并创建 PID 控制任务
 * @note   1) IN2（PB9）写固定低电平，锁定"正转"方向（IN1=PWM/IN2=0）；
 *         2) TIM4_CH3 启动 PWM 输出，比较值先置 0（IN1 恒低 → 电机静止）；
 *         3) 运行标志与速度设定清零；
 *         4) 创建 "MotorPid" 控制任务：系统 RUN → PID 闭环调速，
 *            IDLE / FAULT → 自动停机（上电默认空闲，电机保持静止）。
 *         须在 vTaskStartScheduler() 之前、MX_TIM4_Init() 之后调用。
 * @retval 1=成功，0=失败（控制任务创建失败）
 */
uint8_t Motor_Init(void)
{
    HAL_GPIO_WritePin(MOTOR2_GPIO_Port, MOTOR2_Pin, MOTOR2_FIXED_STATE);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0u);
    (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

    taskENTER_CRITICAL();
    s_running = 0u;
    s_speedPct = 0u;
    taskEXIT_CRITICAL();

    if (xTaskCreate(MotorCtrlTask, "MotorPid", MOTOR_TASK_STACK, NULL,
                    MOTOR_TASK_PRIO, NULL) != pdPASS)
    {
        return 0u; /* 任务创建失败（FreeRTOS 堆不足） */
    }

    return 1u;
}

/**
 * @brief  启动电机：按当前设定速度输出 PWM
 * @note   幂等。设定速度为 0（从未调速）时取默认 50% 启动，保证"开始"
 *         有明确动作；其余情况沿用上次设定速度。
 * @retval 无
 */
void Motor_Start(void)
{
    uint8_t speed;

    taskENTER_CRITICAL();
    if (s_speedPct < MOTOR_MIN_SPEED_PCT)
    {
        s_speedPct = MOTOR_DEFAULT_SPEED_PCT; /* 未设定速度 → 用默认值启动 */
    }
    speed = s_speedPct;
    taskEXIT_CRITICAL();

    Motor_SetCompare(speed);
    (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); /* 恢复输出（Stop 时已关闭） */

    taskENTER_CRITICAL();
    s_running = 1u;
    taskEXIT_CRITICAL();
}

/**
 * @brief  暂停电机：占空比清零并关闭 PWM 输出（滑行停止）
 * @note   幂等。IN1=0 与 IN2=0 同时成立时 DRV8870 为 coast 状态，电机
 *         惯性自由减速；速度设定值保留，便于下次按原速恢复。
 * @retval 无
 */
void Motor_Stop(void)
{
    taskENTER_CRITICAL();
    s_running = 0u;
    taskEXIT_CRITICAL();

    (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0u);
}

/**
 * @brief  更新电机速度（占空比百分比）
 * @note   入参饱和限幅到 100%。运行中立即改写比较值生效；停止状态下
 *         仅记录设定值。设 0 时运行中会滑行停转但保留运行标志，
 *         随后设非 0 可直接恢复转动。
 * @param  percent 占空比 0~100（%）
 * @retval 无
 */
void Motor_SetSpeed(uint8_t percent)
{
    uint8_t running;

    if (percent > 100u)
    {
        percent = 100u; /* 饱和限幅 */
    }

    taskENTER_CRITICAL();
    s_speedPct = percent;
    running = s_running;
    taskEXIT_CRITICAL();

    if (running == 1u)
    {
        Motor_SetCompare(percent); /* 运行中：调速立即生效 */
    }
}

/**
 * @brief  查询电机当前设定速度
 * @retval 占空比 0~100（%）
 */
uint8_t Motor_GetSpeed(void)
{
    uint8_t speed;

    taskENTER_CRITICAL();
    speed = s_speedPct;
    taskEXIT_CRITICAL();

    return speed;
}

/**
 * @brief  查询电机是否处于运行状态
 * @retval 1=运行（已启动且速度非 0），0=停止
 */
uint8_t Motor_IsRunning(void)
{
    uint8_t running;

    taskENTER_CRITICAL();
    running = ((s_running == 1u) && (s_speedPct >= MOTOR_MIN_SPEED_PCT)) ? 1u : 0u;
    taskEXIT_CRITICAL();

    return running;
}

/**
 * @brief  按占空比百分比写 TIM4_CH3 比较值
 * @note   比较值 = 百分比 × 36（3600 计数对应 100%），写 0 即输出恒低。
 *         仅由本模块内部调用，不做入参校验（调用方已限幅）。
 * @param  percent 占空比 0~100（%）
 * @retval 无
 */
static void Motor_SetCompare(uint8_t percent)
{
    uint32_t compare = (uint32_t)percent * MOTOR_PWM_CNT_PER_PCT;

    if (compare > MOTOR_PWM_PERIOD_CNT)
    {
        compare = MOTOR_PWM_PERIOD_CNT; /* 兜底：不超过一个周期 */
    }

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, compare);
}

/**
 * @brief  PID 转速控制任务：目标转速 → 前馈 + PI → 占空比输出
 * @note   控制律（位置式，前馈承担主要输出，PI 修正偏差）：
 *             u = K_ff × Target + Kp × e + ∫Ki × e dt + Kd × de
 *           其中 K_ff = 100/3400（负载实测极限折算的线性预估），
 *           e = 目标转速 - 编码器实测转速（取自 SysTem CurrentRPM）。
 *         状态联动：
 *           SYS_STATE_RUN   → 闭环运行（目标 > 0 才输出）；
 *           SYS_STATE_IDLE  → 停机 + 控制器复位；
 *           SYS_STATE_FAULT → 停机 + 控制器复位（故障安全）。
 *         抗积分饱和：输出已达上限且误差仍为正（或已达下限且误差为负）
 *         时停止积分累加；积分项另设 ±30% 限幅。
 *         RUN 态下本任务是速度唯一写者：每周期无条件 Motor_SetSpeed，
 *         手动 Start/SetSpeed 的效果会被覆盖。
 *         日志仅在状态跳变时打印一次，稳态零输出开销。
 * @param  argument 未使用
 * @retval 无（任务永不返回）
 */
static void MotorCtrlTask(void *argument)
{
    TickType_t lastWake = xTaskGetTickCount();
    SysRunState_t state;
    float target;
    float current;
    float err;
    float uNoI; /* 不含积分项的输出（用于抗饱和判断） */
    float uOut;
#if MOTOR_DEBUG_EN
    static uint8_t s_dbgDiv = 0u; /* 调试打印 1s 分频计数 */
#endif

    (void)argument;

    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_CTRL_PERIOD_MS));

        /* ---------- 状态联动：非运行态一律停机并复位控制器 ---------- */
        state = SysTem_GetState();
        if (state != SYS_STATE_RUN)
        {
            if (s_pidActive != 0u)
            {
                Motor_Stop();
                s_pidInteg = 0.0f;
                s_pidPrevErr = 0.0f;
                s_pidActive = 0u;
                s_stallCycles = 0u;
                printf("[Motor] system %s -> motor stop\r\n",
                       (state == SYS_STATE_FAULT) ? "FAULT" : "IDLE");
            }
            continue;
        }

        /* ---------- 目标转速处理：0 视为停转，超限饱和 ---------- */
        target = SysTem_GetTargetRPM();
        if (target < MOTOR_RPM_TARGET_MIN)
        {
            if (s_pidActive != 0u)
            {
                Motor_Stop();
                s_pidInteg = 0.0f;
                s_pidPrevErr = 0.0f;
                s_pidActive = 0u;
                s_stallCycles = 0u;
                printf("[Motor] target=0 -> motor stop\r\n");
            }
            continue;
        }
        if (target > MOTOR_RPM_MAX)
        {
            target = MOTOR_RPM_MAX; /* 超出负载极限按极限处理 */
        }

        /* ---------- PID 计算：前馈 + 比例 + 积分（抗饱和） ---------- */
        current = SysTem_GetCurrentRPM(); /* 编码器任务 100ms 更新 */
        err = target - current;

        uNoI = MOTOR_DUTY_DEADZONE + /* 前馈：死区 + 线性段模型
                                      * （低于死区的输出根本不转，
                                      *  纯线性前馈在低速目标下严重
                                      *  偏小，导致起动后长时间拉锯） */
               (100.0f - MOTOR_DUTY_DEADZONE) * target / MOTOR_RPM_MAX +
               MOTOR_PID_KP * err +
               MOTOR_PID_KD * (err - s_pidPrevErr);

        /* 输出饱和且误差继续同向时停止积分，防止积分深饱和；
         * 起动阶段（未转起来）冻结积分：此时误差恒为正，若继续积分会在
         * 起动瞬间积出大幅过冲（这正是"10s 才起动"且一起动就猛冲的根源） */
        if ((current >= MOTOR_STALL_RPM) &&
            !(((uNoI + s_pidInteg) > 100.0f) && (err > 0.0f)) &&
            !(((uNoI + s_pidInteg) < 0.0f) && (err < 0.0f)))
        {
            s_pidInteg += MOTOR_PID_KI * err;
            if (s_pidInteg > MOTOR_PID_INTEG_MAX)
            {
                s_pidInteg = MOTOR_PID_INTEG_MAX;
            }
            else if (s_pidInteg < -MOTOR_PID_INTEG_MAX)
            {
                s_pidInteg = -MOTOR_PID_INTEG_MAX;
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
        if (uOut < (float)MOTOR_DUTY_MIN)
        {
            uOut = 0.0f; /* 低于最小有效占空比 → 不转 */
        }

        /* ---------- 静摩擦起动补偿：未转起来时输出爬坡式起动 Kick ---------- */
        /* 目标 > 0 但实测转速仍低于起动判定阈值（静摩擦/卡滞）时，PID 算出
         * 的占空比往往不足以突破静摩擦。Kick 若为固定值有卡死风险：实际
         * 起动占空比高于 Kick 时，输出被锁死在 Kick 上永远起不来（积分又
         * 已冻结）。故改为"基准 Kick + 每周期爬坡"：
         *   Kick = BREAKAWAY + stallCycles × RAMP，封顶 KICK_MAX；
         * 转起来（≥MOTOR_STALL_RPM）后计数清零、交还 PID 正常控制。 */
        if (current < MOTOR_STALL_RPM)
        {
            float kick;

            s_stallCycles++;
            kick = (float)MOTOR_DUTY_BREAKAWAY +
                   (float)s_stallCycles * (float)MOTOR_STALL_RAMP_PCT;
            if (kick > (float)MOTOR_DUTY_KICK_MAX)
            {
                kick = (float)MOTOR_DUTY_KICK_MAX;
            }
            if (uOut < kick)
            {
                uOut = kick;
            }
            if (s_stallCycles == 1u)
            {
                printf("[Motor] stall detected -> starting kick\r\n");
            }
        }
        else
        {
            s_stallCycles = 0u; /* 已转起来：斜坡复位 */
        }

        /* ---------- 输出：未启动先启动，再更新占空比 ---------- */
        if (s_pidActive == 0u)
        {
            s_pidActive = 1u;
            Motor_Start(); /* 首次进入闭环：启动输出 */
            printf("[Motor] PID start: target=%d rpm\r\n", (int)target);
        }
        else if (Motor_IsRunning() == 0u)
        {
            Motor_Start(); /* 输出通道被外部关闭时恢复 */
        }
        Motor_SetSpeed((uint8_t)uOut);

#if MOTOR_DEBUG_EN
        /* 调试打印：每 10 个周期（1s）输出一次控制器内部状态 */
        s_dbgDiv++;
        if (s_dbgDiv >= 10u)
        {
            s_dbgDiv = 0u;
            printf("[Motor] tgt=%d cur=%d err=%d I=%d u=%d%% stall=%u\r\n",
                   (int)target, (int)current, (int)err, (int)s_pidInteg,
                   (int)uOut, (unsigned int)s_stallCycles);
        }
#endif
    }
}

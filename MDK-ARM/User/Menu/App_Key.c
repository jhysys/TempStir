/**
 ******************************************************************************
 * @file    App_Key.c
 * @brief   按键模块实现（KEY1=PC6 / KEY2=PC7 / KEY3=PC8 / KEY4=PC9，低电平有效，扫描任务方案）
 * @note    采用 10ms 周期轮询 + 双次采样确认的统一消抖检测框架，
 *          一次扫描任务同时处理四个按键，每个按键由独立的上下文
 *          （KeyCtx_t）维护状态，检测层统一产出按键事件，业务层
 *          只做"事件 → 动作"映射：
 *            KEY1：确认按下瞬间 → 聚焦点在温度/转速/时间间循环切换 +
 *                  蜂鸣器短鸣；
 *            KEY2：短按（松开前持续 < 1s）→ 聚焦参数 +1 + 短鸣；
 *                  长按（按住 ≥ 1s）→ 聚焦参数 +10，之后每 300ms
 *                  重复 +10（连发），整个长按过程不鸣叫；
 *            KEY3：与 KEY2 完全对称，方向为减——短按 -1 + 短鸣；
 *                  长按 -10 每 300ms 连发，不鸣叫。
 *            KEY4：短按 → 系统状态切换（空闲 ↔ 运行）。启动（空闲→
 *                  运行）须满足三个运行条件：目标温度 > 当前温度、
 *                  目标转速 > 当前转速、目标时间 > 当前时间；条件不
 *                  满足则拒绝启动并长鸣提示。停止（运行→空闲）无条件
 *                  执行，启动成功与手动停止均短鸣反馈；
 *                  长按（按住 ≥ 1s）→ 全部数据归零复位：目标温度/当前
 *                  温度/目标转速/当前转速/目标时间/当前时间六项清零，
 *                  系统状态强制转空闲，短鸣一声反馈，仅触发一次。
 *          KEY2/KEY3/KEY4 事件语义采用"长按到点立即触发"变体：长按
 *          在按满 1s 时刻执行（响应及时），短按在松开瞬间执行；
 *          一旦长按已触发，释放不再补发短按事件，两种事件互斥。
 *          减操作带下限保护：目标温度/转速不小于 0.0、目标时间不小于 0。
 *          聚焦点切换通过 Oled_SetFocus()（内部走 SysTem_SetFocus）
 *          完成，保持"按键任务 = 聚焦点唯一写者"纪律；参数修改通过
 *          SysTem 对应 setter 完成（自动触发 3s 延时持久化）；蜂鸣
 *          反馈通过 Buzzer_Beep() 命令队列投递，均不阻塞扫描任务。
 *          按键电气属性（输入上拉）由 gpio.c 统一配置，本模块只读。
 ******************************************************************************
 */
#include "App_Key.h"
#include "App_SysTem.h"
#include "App_Oled.h"
#include "App_Buzzer.h"
#include "bsp_pins.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

/*------------------------------ 私有宏定义 ---------------------------------*/
#define KEY_SCAN_PERIOD_MS 10u /* 按键扫描周期（ms） */
#define KEY_DEBOUNCE_TICKS 2u  /* 消抖确认周期数：连续 2 次采样一致才生效（20ms） */
#define KEY_LONG_TICKS 100u    /* 长按阈值：100 个周期 = 1s（KEY2/KEY3 共用） */
#define KEY_REPEAT_TICKS 30u   /* 长按连发间隔：30 个周期 = 300ms（KEY2/KEY3 共用） */

/*------------------------------ 私有类型定义 -------------------------------*/
/* 按键事件：由检测层产出，业务层映射为具体动作 */
typedef enum
{
    KEY_EVT_NONE = 0,   /* 本周期无事件 */
    KEY_EVT_DOWN = 1,   /* 确认按下瞬间（消抖通过） */
    KEY_EVT_SHORT = 2,  /* 短按：释放且按下持续 < 长按阈值 */
    KEY_EVT_LONG = 3,   /* 长按：按住达到阈值瞬间（仅触发一次） */
    KEY_EVT_REPEAT = 4, /* 长按连发：达到阈值后每隔连发间隔触发 */
} KeyEvent_t;

/* 按键独立上下文：每个物理按键一份，互不干扰 */
typedef struct
{
    GPIO_TypeDef *port;  /* GPIO 端口（bsp_pins.h 网络名映射） */
    uint16_t pin;        /* GPIO 引脚 */
    uint8_t lowCnt;      /* 连续读到"按下"电平的周期数 */
    uint8_t highCnt;     /* 连续读到"释放"电平的周期数 */
    uint8_t isPressed;   /* 已确认按下且未释放标志 */
    uint16_t pressTicks; /* 确认按下后经过的扫描周期数（长按计时） */
} KeyCtx_t;

/*------------------------------ 私有变量 ---------------------------------*/
/* 三个按键的独立上下文（引脚取自 bsp_pins.h，低电平有效） */
static KeyCtx_t s_key1 = {KEY1_GPIO_Port, KEY1_Pin, 0u, 0u, 0u, 0u};
static KeyCtx_t s_key2 = {KEY2_GPIO_Port, KEY2_Pin, 0u, 0u, 0u, 0u};
static KeyCtx_t s_key3 = {KEY3_GPIO_Port, KEY3_Pin, 0u, 0u, 0u, 0u};
static KeyCtx_t s_key4 = {KEY4_GPIO_Port, KEY4_Pin, 0u, 0u, 0u, 0u};

/*------------------------------ 私有函数 ---------------------------------*/
static void KeyScanTask(void *argument);
static KeyEvent_t Key_Detect(KeyCtx_t *key);
static void Key1_HandleEvent(KeyEvent_t evt);
static void Key_Value_HandleEvent(KeyEvent_t evt, int8_t dir);
static void Key_Value_ApplyStep(int8_t step);
static void Key4_HandleEvent(KeyEvent_t evt);

/**
 * @brief  按键检测：消抖状态机 + 事件产出（对所有按键统一）
 * @note   事件时序（KEY2 为例，阈值 1s、连发间隔 300ms）：
 *           按下 → 20ms 消抖 → KEY_EVT_DOWN（1 次）
 *           持续按住到 1s   → KEY_EVT_LONG（仅 1 次）
 *           继续按住每 300ms → KEY_EVT_REPEAT（连发）
 *           松开（<1s）      → KEY_EVT_SHORT
 *           松开（≥1s）      → 无事件（长按已触发，短按被抑制）
 *         释放同样需 20ms 消抖确认，防止抖动误判为松开。
 * @param  key 按键上下文
 * @retval 本周期产生的按键事件
 */
static KeyEvent_t Key_Detect(KeyCtx_t *key)
{
    KeyEvent_t evt = KEY_EVT_NONE;

    if (HAL_GPIO_ReadPin(key->port, key->pin) == KEY_PRESSED_STATE)
    {
        /* 低电平：累计按下计数，清零释放计数 */
        if (key->lowCnt < KEY_DEBOUNCE_TICKS)
        {
            key->lowCnt++;
        }
        key->highCnt = 0u;

        if (key->isPressed == 0u)
        {
            /* 消抖通过 → 确认按下，清零计时并产出 DOWN 事件 */
            if (key->lowCnt >= KEY_DEBOUNCE_TICKS)
            {
                key->isPressed = 1u;
                key->pressTicks = 0u;
                evt = KEY_EVT_DOWN;
            }
        }
        else
        {
            /* 按住中：累计按压时长，产出长按/连发事件 */
            key->pressTicks++;
            if (key->pressTicks == KEY_LONG_TICKS)
            {
                evt = KEY_EVT_LONG; /* 到点立即触发 */
            }
            else if ((key->pressTicks > KEY_LONG_TICKS) &&
                     ((key->pressTicks - KEY_LONG_TICKS) % KEY_REPEAT_TICKS) == 0u)
            {
                evt = KEY_EVT_REPEAT; /* 每 300ms 连发 */
            }
        }
    }
    else
    {
        /* 高电平：累计释放计数，清零按下计数 */
        if (key->highCnt < KEY_DEBOUNCE_TICKS)
        {
            key->highCnt++;
        }
        key->lowCnt = 0u;

        if (key->isPressed != 0u)
        {
            /* 消抖通过 → 确认释放；仅当按压时长不足长按阈值时报短按 */
            if (key->highCnt >= KEY_DEBOUNCE_TICKS)
            {
                key->isPressed = 0u;
                if (key->pressTicks < KEY_LONG_TICKS)
                {
                    evt = KEY_EVT_SHORT;
                }
            }
        }
    }

    return evt;
}

/**
 * @brief  KEY1 事件处理：聚焦点循环切换 + 蜂鸣器短鸣
 * @note   仅响应 DOWN 事件（按住不重复触发）；聚焦点顺序
 *         温度 → 转速 → 时间 → 温度。
 * @param  evt 按键事件
 * @retval 无
 */
static void Key1_HandleEvent(KeyEvent_t evt)
{
    SysFocus_t cur;
    SysFocus_t next;

    if (evt != KEY_EVT_DOWN)
    {
        return;
    }

    /* 聚焦点循环切换：温度 → 转速 → 时间 → 温度 */
    cur = SysTem_GetFocus();
    next = (cur == SYS_FOCUS_TEMP) ? SYS_FOCUS_RPM : (cur == SYS_FOCUS_RPM) ? SYS_FOCUS_TIME
                                                                            : SYS_FOCUS_TEMP;
    Oled_SetFocus(next);

    /* 蜂鸣器短鸣一声（命令队列投递，非阻塞，队列满则丢弃不卡按键） */
    (void)Buzzer_Beep(BEEP_SHORT);
}

/**
 * @brief  数值调整类按键（KEY2/KEY3）共享事件处理
 * @note   KEY2（dir=+1）与 KEY3（dir=-1）共用本映射，仅方向不同：
 *           短按 → 聚焦参数 dir*1 + 蜂鸣器短鸣；
 *           长按到点/连发 → 聚焦参数 dir*10，不鸣叫。
 * @param  evt 按键事件
 * @param  dir 方向：+1=KEY2 加，-1=KEY3 减
 * @retval 无
 */
static void Key_Value_HandleEvent(KeyEvent_t evt, int8_t dir)
{
    switch (evt)
    {
    case KEY_EVT_SHORT:
        Key_Value_ApplyStep((int8_t)(dir * 1)); /* 短按：步进 1 */
        (void)Buzzer_Beep(BEEP_SHORT);          /* 短鸣反馈 */
        break;

    case KEY_EVT_LONG:   /* 长按到点：步进 10，不鸣叫 */
    case KEY_EVT_REPEAT: /* 连发：步进 10，不鸣叫 */
        Key_Value_ApplyStep((int8_t)(dir * 10));
        break;

    default:
        break;
    }
}

/**
 * @brief  按聚焦点对目标参数施加步进（可正可负）
 * @note   通过 SysTem 对应 setter 修改，值变化时自动调度 3s 延时
 *         EEPROM 持久化；连续快速调整只会在停止调整 3s 后落盘一次。
 *         下限保护：目标温度/转速不低于 0.0、目标时间不低于 0（KEY3
 *         减操作不会出现负值/下溢回绕）；上限暂不限制，如需量程
 *         限幅在此补齐。
 * @param  step 步进量（+1/+10/-1/-10）
 * @retval 无
 */
static void Key_Value_ApplyStep(int8_t step)
{
    switch (SysTem_GetFocus())
    {
    case SYS_FOCUS_TEMP:
    {
        float v = SysTem_GetTargetTemp() + (float)step;

        if (v < 0.0f)
        {
            v = 0.0f; /* 温度不允许为负 */
        }
        SysTem_SetTargetTemp(v);
        break;
    }

    case SYS_FOCUS_RPM:
    {
        float v = SysTem_GetTargetRPM() + (float)step;

        if (v < 0.0f)
        {
            v = 0.0f; /* 转速不允许为负 */
        }
        SysTem_SetTargetRPM(v);
        break;
    }

    case SYS_FOCUS_TIME:
    {
        uint32_t v = SysTem_GetTargetTime();
        uint32_t s = (uint32_t)((step < 0) ? -step : step);

        v = (step < 0) ? ((v >= s) ? (v - s) : 0u) /* 时间不允许为负（防下溢回绕） */
                       : (v + s);
        SysTem_SetTargetTime(v);
        break;
    }

    default:
        break;
    }
}

/**
 * @brief  KEY4 事件处理：短按启停切换 / 长按全部参数归零
 * @note   短按（松开前持续 < 1s）→ 系统状态切换（空闲 ↔ 运行）：
 *           空闲 → 运行：须同时满足三个运行条件——目标温度 > 当前
 *           温度、目标转速 > 当前转速、目标时间 > 当前时间；满足则
 *           启动并短鸣反馈；任一不满足则拒绝启动，长鸣一声提示。
 *           运行 → 空闲：无条件停止，短鸣反馈。当前时间保持停止
 *           时刻值，下次启动时由运行监控自动归零重新计时。
 *         长按（按住 ≥ 1s）→ 全部数据归零复位：目标温度/当前温度/
 *           目标转速/当前转速/目标时间/当前时间六项全部清零，系统
 *           状态强制转空闲，短鸣一声反馈；仅触发一次（按住不重复）。
 * @param  evt 按键事件
 * @retval 无
 */
static void Key4_HandleEvent(KeyEvent_t evt)
{
    SysState_t view;

    switch (evt)
    {
        case KEY_EVT_SHORT:
            SysTem_GetSnapshot(&view);

            if (view.State == SYS_STATE_IDLE)
            {
                /* 运行条件校验：目标必须大于当前（温度/转速/时间三者同时满足） */
                if ((view.TargetTemp > view.CurrentTemp) &&
                    (view.TargetRPM > view.CurrentRPM) &&
                    (view.TargetTime > view.CurrentTime))
                {
                    SysTem_SetState(SYS_STATE_RUN);   /* 启动：空闲 → 运行 */
                    (void)Buzzer_Beep(BEEP_SHORT);    /* 启动成功短鸣 */
                }
                else
                {
                    (void)Buzzer_Beep(BEEP_LONG);     /* 条件不满足，长鸣提示拒绝 */
                }
            }
            else
            {
                SysTem_SetState(SYS_STATE_IDLE);      /* 手动停止：运行 → 空闲 */
                (void)Buzzer_Beep(BEEP_SHORT);        /* 停止短鸣 */
            }
            break;

        case KEY_EVT_LONG:
            /* 长按复位：六项数据全部归零 + 强制转空闲 + 短鸣反馈 */
            SysTem_SetTargetTemp(0.0f);
            SysTem_SetCurrentTemp(0.0f);
            SysTem_SetTargetRPM(0.0f);
            SysTem_SetCurrentRPM(0.0f);
            SysTem_SetCurrentTime(0u);                /* 先清当前时间 */
            SysTem_SetTargetTime(0u);
            SysTem_SetState(SYS_STATE_IDLE);          /* 强制转空闲 */
            (void)Buzzer_Beep(BEEP_SHORT);            /* 复位完成反馈 */
            break;

        default:                                      /* DOWN/REPEAT/NONE 不响应 */
            break;
    }
}

/**
 * @brief  按键扫描任务：10ms 周期统一扫描 KEY1/KEY2/KEY3/KEY4
 * @note   每周期对每个按键独立跑一遍"检测 → 事件 → 动作映射"，
 *          各按键上下文互不干扰。
 * @param  argument 未使用
 * @retval 无
 */
static void KeyScanTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(KEY_SCAN_PERIOD_MS));

        Key1_HandleEvent(Key_Detect(&s_key1));
        Key_Value_HandleEvent(Key_Detect(&s_key2), 1);  /* KEY2：加方向 */
        Key_Value_HandleEvent(Key_Detect(&s_key3), -1); /* KEY3：减方向 */
        Key4_HandleEvent(Key_Detect(&s_key4));
    }
}

/**
 * @brief  按键模块初始化：创建按键扫描任务
 * @note   必须在 vTaskStartScheduler() 之前调用；扫描任务优先级 2、
 *         栈 256 字（含长按计时与业务映射逻辑）。
 * @retval 1=成功，0=失败（任务创建失败）
 */
uint8_t Key_Init(void)
{
    if (xTaskCreate(KeyScanTask, "KeyScan", 256u, NULL, 2u, NULL) != pdPASS)
    {
        printf("[Key] scan task create failed!\r\n");
        return 0;
    }
    return 1;
}

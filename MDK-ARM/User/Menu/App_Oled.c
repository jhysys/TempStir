/**
  ******************************************************************************
  * @file    App_Oled.c
  * @brief   OLED 显示模块实现（SSD1315 控制器，I2C1 总线，128x64 点阵）
  * @note    LibDriver 的 SSD1315 驱动采用回调注入设计，本文件负责：
  *          1. 把驱动的 iic_init/iic_write 回调挂接到 bsp_i2c1 阻塞封装；
  *          2. 为驱动的 SPI/GPIO 回调提供空桩（ssd1315_init 无条件检查
  *             这些指针，即使纯 IIC 模式也必须非空，硬件上并未使用）；
  *          3. 提供 debug_print 与 delay_ms 回调；
  *          4. UI 任务：按 128x64 分区式布局显示设备 ID、系统状态、
  *             三组目标/当前参数与时间进度条，并执行运行态监控规则
  *             （约束 2~4）。
  *          布局（自上而下）：
  *             y0~11   反白标题栏：设备 ID（左）+ 系统状态（右对齐）；
  *             y12     1px 全宽分隔线；
  *             y13~28  TEMP 行（8x16 大字，聚焦行整行反白高亮）；
  *             y29~44  RPM  行（同上）；
  *             y45~60  TIME 行（同上）；
  *             y61~63  时间进度条（1px 外框 + 进度填充，仅运行态推进）。
  *          无感刷新策略：UI 任务只在"行内容发生变化"时重绘该行，再把
  *          该行占用的显存页通过页寻址 + 批量数据写推送到屏幕（一次
  *          I2C 事务写 128 字节），像素内容不变的部分完全不动，因此
  *          视觉上无闪烁、无整屏擦除感；I2C 总线占用也降到最低。
  *          OLED 的跨任务互斥由 bsp_i2c1 内部互斥量统一处理。
  ******************************************************************************
  */
#include "App_Oled.h"
#include <stdio.h>
#include <string.h>
#include "driver_ssd1315.h"
#include "bsp_i2c1.h"
#include "App_SysTem.h"
#include "FreeRTOS.h"
#include "task.h"

/*------------------------------ 私有宏定义 ---------------------------------*/
#define OLED_I2C_ADDR          SSD1315_ADDR_SA0_0  /* SSD1315 8 位 I2C 地址 0x78（SA0 接地） */
#define OLED_I2C_TIMEOUT_MS    100u                /* I2C 事务超时（ms） */

/* UI 布局几何参数：128x64 分区式布局（详见文件头说明） */
#define OLED_HEADER_Y0        0u                 /* 反白标题栏起始 y 坐标 */
#define OLED_HEADER_Y1        11u                /* 反白标题栏结束 y 坐标（高 12px，6x12 字体） */
#define OLED_SEP_Y            12u                /* 标题栏下 1px 分隔线 y 坐标 */
#define OLED_ROW_Y0           13u                /* 参数区首行 y 坐标（行高 16px，8x16 字体） */
#define OLED_ROW_H            16u                /* 参数行行高（像素） */
#define OLED_HEADER_COLS      21u                /* 标题栏字符数（6px/字 x 21 = 126 <= 128） */
#define OLED_ROW_COLS         16u                /* 参数行字符数（8px/字 x 16 = 128） */
#define OLED_BAR_Y0           61u                /* 进度条起始 y 坐标 */
#define OLED_BAR_Y1           63u                /* 进度条结束 y 坐标（总高 3px） */
#define OLED_BAR_INNER_W      125u               /* 进度条内腔可填充宽度（x1~x126） */
#define OLED_UI_REGIONS       5u                 /* UI 区域数：标题栏 + 3 参数行 + 进度条 */
#define OLED_UI_PERIOD_MS     200u               /* UI 任务刷新周期（ms） */
#define OLED_TICKS_PER_SEC    (1000u / OLED_UI_PERIOD_MS)  /* 1 秒对应的任务周期数 */

/* UI 区域编号：与签名缓存数组下标一一对应 */
typedef enum
{
    OLED_REG_HEADER = 0,                         /* 区域 0：反白标题栏 */
    OLED_REG_ROW0,                               /* 区域 1：TEMP 行 */
    OLED_REG_ROW1,                               /* 区域 2：RPM 行 */
    OLED_REG_ROW2,                               /* 区域 3：TIME 行 */
    OLED_REG_BAR,                                /* 区域 4：时间进度条 */
} OledRegion_t;

/*------------------------------ 私有变量 ---------------------------------*/
/* SSD1315 驱动句柄：全模块唯一实例，仅供本文件内部使用 */
static ssd1315_handle_t s_oledHandle;

/* UI 区域签名缓存：每区域一份签名字符串（含反白标记/内容/进度宽度），
 * 与上次上屏签名比对，只重绘变化的区域（无感刷新核心） */
static char s_sig[OLED_UI_REGIONS][24u];

/* 运行态监控辅助变量 */
static SysRunState_t s_prevState = SYS_STATE_IDLE;  /* 上一周期系统状态（检测 IDLE→RUN 清零计时） */
static uint8_t       s_tickCnt   = 0u;              /* 1 秒节拍计数器 */

/*------------------------------ 私有函数（驱动回调） -----------------------*/
static uint8_t Oled_IicInit(void);
static uint8_t Oled_IicDeinit(void);
static uint8_t Oled_IicWrite(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
static uint8_t Oled_SpiInitStub(void);
static uint8_t Oled_SpiDeinitStub(void);
static uint8_t Oled_SpiWriteCmdStub(uint8_t *buf, uint16_t len);
static uint8_t Oled_SpiGpioInitStub(void);
static uint8_t Oled_SpiGpioDeinitStub(void);
static uint8_t Oled_SpiGpioWriteStub(uint8_t value);
static uint8_t Oled_ResetGpioInitStub(void);
static uint8_t Oled_ResetGpioDeinitStub(void);
static uint8_t Oled_ResetGpioWriteStub(uint8_t value);
static void    Oled_DebugPrint(const char *const fmt, ...);
static void    Oled_DelayMs(uint32_t ms);

/*------------------------------ 私有函数（UI） ----------------------------*/
static const char *Oled_StateStr(SysRunState_t state);
static void    Oled_FmtTenth(float val, char *out);
static unsigned long Oled_Round(float val);
static void    Oled_PadSpaces(char *buf, uint8_t width);
static unsigned long Oled_BarWidth(const SysState_t *view);
static void    Oled_BuildHeader(const SysState_t *view, char *buf);
static void    Oled_DrawHeader(const SysState_t *view);
static void    Oled_BuildRow(uint8_t idx, const SysState_t *view, uint8_t invert, char *sig);
static void    Oled_DrawRow(uint8_t idx, const char *text, uint8_t invert);
static void    Oled_BuildBar(const SysState_t *view, char *sig);
static void    Oled_DrawBar(const SysState_t *view);
static uint8_t Oled_FlushPage(uint8_t page);
static uint8_t Oled_FlushRange(uint8_t y0, uint8_t y1);
static void    OledUiTask(void *argument);

/**
  * @brief  IIC 初始化回调：创建 I2C1 互斥量并确认 OLED 在线
  * @note   驱动 ssd1315_init() 内部会调用本回调；返回 0 表示成功，
  *         非 0 表示失败（驱动会中止初始化）。
  * @retval 0=成功，1=OLED 器件未应答
  */
static uint8_t Oled_IicInit(void)
{
    /* 创建 I2C1 跨任务互斥量；调度器启动前创建互斥量合法（内部自动跳过取锁） */
    Bsp_I2c1Init();

    /* 探测 OLED 是否应答，失败时让驱动中止初始化，便于上电定位接线问题 */
    if (Bsp_I2c1IsDeviceReady(OLED_I2C_ADDR, 10u, OLED_I2C_TIMEOUT_MS) != HAL_OK)
    {
        printf("[Oled] device 0x%02X not ready!\r\n", OLED_I2C_ADDR);
        return 1u;
    }
    return 0u;
}

/**
  * @brief  IIC 反初始化回调（本工程不反初始化 I2C1，直接返回成功）
  * @retval 0=成功
  */
static uint8_t Oled_IicDeinit(void)
{
    return 0u;
}

/**
  * @brief  IIC 写回调：把驱动的一次写事务转发到 bsp_i2c1
  * @note   LibDriver 的 IIC 约定：reg 即 SSD1315 的控制字节——0x00 为命令流、
  *         0x40 为显存数据流，与 Bsp_I2c1MemWrite 的"存储地址"参数语义一致，
  *         可直接透传。
  * @param  addr 8 位器件地址（0x78）
  * @param  reg  控制字节（0x00 命令 / 0x40 数据）
  * @param  buf  待写数据缓冲
  * @param  len  待写字节数
  * @retval 0=成功，1=I2C 事务失败
  */
static uint8_t Oled_IicWrite(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (Bsp_I2c1MemWrite(addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, OLED_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return 1u;
    }
    return 0u;
}

/**
  * @brief  SPI 初始化空桩（本工程 OLED 走 I2C，SPI 回调仅为满足驱动指针检查）
  * @retval 0=成功
  */
static uint8_t Oled_SpiInitStub(void)
{
    return 0u;
}

/**
  * @brief  SPI 反初始化空桩
  * @retval 0=成功
  */
static uint8_t Oled_SpiDeinitStub(void)
{
    return 0u;
}

/**
  * @brief  SPI 命令写空桩
  * @param  buf 数据缓冲（未使用）
  * @param  len 长度（未使用）
  * @retval 0=成功
  */
static uint8_t Oled_SpiWriteCmdStub(uint8_t *buf, uint16_t len)
{
    (void)buf;
    (void)len;
    return 0u;
}

/**
  * @brief  SPI 命令/数据 GPIO 初始化空桩
  * @retval 0=成功
  */
static uint8_t Oled_SpiGpioInitStub(void)
{
    return 0u;
}

/**
  * @brief  SPI 命令/数据 GPIO 反初始化空桩
  * @retval 0=成功
  */
static uint8_t Oled_SpiGpioDeinitStub(void)
{
    return 0u;
}

/**
  * @brief  SPI 命令/数据 GPIO 写空桩
  * @param  value 电平值（未使用）
  * @retval 0=成功
  */
static uint8_t Oled_SpiGpioWriteStub(uint8_t value)
{
    (void)value;
    return 0u;
}

/**
  * @brief  复位 GPIO 初始化空桩（本硬件 OLED 无独立复位脚）
  * @retval 0=成功
  */
static uint8_t Oled_ResetGpioInitStub(void)
{
    return 0u;
}

/**
  * @brief  复位 GPIO 反初始化空桩
  * @retval 0=成功
  */
static uint8_t Oled_ResetGpioDeinitStub(void)
{
    return 0u;
}

/**
  * @brief  复位 GPIO 写空桩（驱动复位时序会被调用，本硬件无此脚，直接成功）
  * @param  value 电平值（未使用）
  * @retval 0=成功
  */
static uint8_t Oled_ResetGpioWriteStub(uint8_t value)
{
    (void)value;
    return 0u;
}

/**
  * @brief  驱动调试打印回调：转发到 USART1 调试串口
  * @param  fmt 格式化字符串（可变参数）
  * @retval 无
  */
static void Oled_DebugPrint(const char *const fmt, ...)
{
    (void)fmt;   /* 初始化阶段保持安静，不刷屏；需要排错时可改为 vprintf 输出 */
}

/**
  * @brief  驱动毫秒延时回调（双态：调度器运行后用任务延时，启动阶段用 HAL 延时）
  * @note   与 drv_m24c02 的延时策略一致，保证调度器前后都能正确工作。
  * @param  ms 延时毫秒数
  * @retval 无
  */
static void Oled_DelayMs(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
    else
    {
        HAL_Delay(ms);
    }
}

/**
  * @brief  系统状态转显示文本（等宽 6x12/8x16 字体均覆盖 ASCII 字符）
  * @param  state 系统运行状态
  * @retval 状态字符串（最长 "FAULT" 5 字符，常量存储区）
  */
static const char *Oled_StateStr(SysRunState_t state)
{
    return (state == SYS_STATE_RUN)   ? "RUN"   :
           (state == SYS_STATE_FAULT) ? "FAULT" : "IDLE";
}

/**
  * @brief  把温度格式化为 0.1 摄氏度分辨率的定点文本（如 "25.3"、"-5.0"）
  * @note   本工程启用 MicroLIB（不支持 %f 浮点格式化），故统一采用
  *         定点放大 10 倍的整数格式化方案；负温度也按绝对值处理。
  * @param  val 温度值（摄氏度）
  * @param  out 输出缓冲（至少 8 字节，含结束符）
  * @retval 无
  */
static void Oled_FmtTenth(float val, char *out)
{
    int32_t  t   = (val >= 0.0f) ? (int32_t)(val * 10.0f + 0.5f)     /* 四舍五入到 0.1 度 */
                                 : (int32_t)(val * 10.0f - 0.5f);
    uint8_t  neg = (t < 0) ? 1u : 0u;                                 /* 符号标志 */
    uint32_t a   = neg ? (uint32_t)(-t) : (uint32_t)t;                /* 绝对值（0.1 度） */

    (void)snprintf(out, 8u, "%s%lu.%1lu", neg ? "-" : "",
                   (unsigned long)(a / 10u), (unsigned long)(a % 10u));
}

/**
  * @brief  浮点数四舍五入取整（转速等整数值显示用，负值归零）
  * @param  val 浮点值
  * @retval 四舍五入后的非负整数
  */
static unsigned long Oled_Round(float val)
{
    return (val >= 0.0f) ? (unsigned long)(val + 0.5f) : 0uL;
}

/**
  * @brief  把文本补空格到固定宽度（保证位数变化时旧内容被完整覆盖）
  * @param  buf   文本缓冲（宽度须大于当前长度）
  * @param  width 目标字符宽度
  * @retval 无
  */
static void Oled_PadSpaces(char *buf, uint8_t width)
{
    uint8_t len = (uint8_t)strlen(buf);
    uint8_t i;

    for (i = len; i < width; i++)
    {
        buf[i] = ' ';
    }
    buf[width] = '\0';
}

/**
  * @brief  计算时间进度条填充宽度（像素）
  * @note   仅运行态且目标时间有效时推进：宽度 = 当前时间/目标时间 x 内腔宽度；
  *         空闲/故障/目标时间为 0（不设限）时宽度为 0（空条）。
  * @param  view 系统状态快照
  * @retval 填充宽度（0~OLED_BAR_INNER_W 像素）
  */
static unsigned long Oled_BarWidth(const SysState_t *view)
{
    unsigned long w = 0uL;

    if ((view->State == SYS_STATE_RUN) && (view->TargetTime > 0u))
    {
        w = (unsigned long)view->CurrentTime * OLED_BAR_INNER_W /
            (unsigned long)view->TargetTime;
        if (w > (unsigned long)OLED_BAR_INNER_W)
        {
            w = OLED_BAR_INNER_W;
        }
    }
    return w;
}

/**
  * @brief  构建反白标题栏签名文本（21 字符，设备 ID 左、状态右对齐）
  * @note   形如 " ID:12345678        RUN"：ID 占前 12 列，状态右对齐到行尾，
  *         中间补空格；状态变化（IDLE/RUN/FAULT）也会反映在签名里。
  * @param  view 系统状态快照
  * @param  buf  输出缓冲（至少 22 字节，含结束符）
  * @retval 无
  */
static void Oled_BuildHeader(const SysState_t *view, char *buf)
{
    const char *stateStr = Oled_StateStr(view->State);
    uint8_t    n;
    uint8_t    i;
    uint8_t    stLen;

    n     = (uint8_t)snprintf(buf, 13u, " ID:%08lX", (unsigned long)view->DeviceID);
    stLen = (uint8_t)strlen(stateStr);
    for (i = n; i < (uint8_t)(OLED_HEADER_COLS - stLen); i++)   /* 中段补空格 */
    {
        buf[i] = ' ';
    }
    (void)strcpy(&buf[OLED_HEADER_COLS - stLen], stateStr);     /* 状态右对齐 */
    buf[OLED_HEADER_COLS] = '\0';
}

/**
  * @brief  绘制反白标题栏 + 分隔线到驱动显存
  * @note   先整块填白再以黑色（color 0）写字符，得到白底黑字效果；
  *         字符绘制会同时写前景/背景像素，覆盖完整。
  * @param  view 系统状态快照
  * @retval 无
  */
static void Oled_DrawHeader(const SysState_t *view)
{
    char text[OLED_HEADER_COLS + 1u];

    Oled_BuildHeader(view, text);
    (void)ssd1315_gram_fill_rect(&s_oledHandle, 0u, OLED_HEADER_Y0,
                                 127u, OLED_HEADER_Y1, 1u);     /* 反白底（整块填白） */
    (void)ssd1315_gram_write_string(&s_oledHandle, 0u, OLED_HEADER_Y0,
                                    text, OLED_HEADER_COLS, 0u, SSD1315_FONT_12);
    (void)ssd1315_gram_fill_rect(&s_oledHandle, 0u, OLED_SEP_Y,
                                 127u, OLED_SEP_Y, 1u);         /* 标题栏下 1px 分隔线 */
}

/**
  * @brief  构建参数行签名（1 字符反白标记 + 16 字符行文本）
  * @note   反白标记进签名：聚焦切换时新旧两行签名都变化，均会重绘。
  *         行文本格式（MicroLIB 无 %f，一律定点整数格式化）：
  *           行 0：TEMP 50.0/24.3    目标/当前温度（0.1 度分辨率）
  *           行 1：RPM  3700/1180    目标/当前转速（取整）
  *           行 2：TIME  300s/  45s  目标/已运行时间（秒）
  *         每行固定 16 字符（不足补空格），完整覆盖整行 128 像素。
  * @param  idx    行号（0=TEMP，1=RPM，2=TIME）
  * @param  view   系统状态快照
  * @param  invert 是否反白高亮（1=聚焦行）
  * @param  sig    输出签名缓冲（至少 18 字节，含结束符）
  * @retval 无
  */
static void Oled_BuildRow(uint8_t idx, const SysState_t *view, uint8_t invert, char *sig)
{
    char a[8];
    char b[8];

    sig[0] = invert ? 'I' : 'N';                                /* 反白标记进签名 */
    switch (idx)
    {
        case 0u:   /* 行 0：目标/当前温度 */
            Oled_FmtTenth(view->TargetTemp, a);
            Oled_FmtTenth(view->CurrentTemp, b);
            (void)snprintf(&sig[1], OLED_ROW_COLS + 1u, "TEMP %s/%s", a, b);
            break;

        case 1u:   /* 行 1：目标/当前转速 */
            (void)snprintf(&sig[1], OLED_ROW_COLS + 1u, "RPM  %4lu/%4lu",
                           Oled_Round(view->TargetRPM),
                           Oled_Round(view->CurrentRPM));
            break;

        default:   /* 行 2：目标/已运行时间 */
        case 2u:
            (void)snprintf(&sig[1], OLED_ROW_COLS + 1u, "TIME %4lus/%4lus",
                           (unsigned long)view->TargetTime,
                           (unsigned long)view->CurrentTime);
            break;
    }
    Oled_PadSpaces(&sig[1], OLED_ROW_COLS);                     /* 补满 16 字符 */
}

/**
  * @brief  绘制参数行到驱动显存（正常白字黑底 / 聚焦反白黑字白底）
  * @param  idx    行号（0~2）
  * @param  text   行文本（须为 OLED_ROW_COLS 字符）
  * @param  invert 是否反白高亮
  * @retval 无
  */
static void Oled_DrawRow(uint8_t idx, const char *text, uint8_t invert)
{
    uint8_t y     = (uint8_t)(OLED_ROW_Y0 + idx * OLED_ROW_H);  /* 行首 y 坐标 */
    uint8_t color = invert ? 0u : 1u;                           /* 反白=黑字(白底自动覆盖)，正常=白字 */

    (void)ssd1315_gram_write_string(&s_oledHandle, 0u, y, (char *)text,
                                    OLED_ROW_COLS, color, SSD1315_FONT_16);
}

/**
  * @brief  构建时间进度条签名（填充宽度像素值字符串）
  * @param  view 系统状态快照
  * @param  sig  输出签名缓冲（至少 8 字节，含结束符）
  * @retval 无
  */
static void Oled_BuildBar(const SysState_t *view, char *sig)
{
    (void)snprintf(sig, 8u, "%lu", Oled_BarWidth(view));
}

/**
  * @brief  绘制底部时间进度条到驱动显存（1px 外框 + 进度填充）
  * @note   空闲时为空框，运行态随已运行时间从左向右填充，满格即到达
  *         目标时间（随后系统转空闲，条自动清空）。
  * @param  view 系统状态快照
  * @retval 无
  */
static void Oled_DrawBar(const SysState_t *view)
{
    unsigned long w = Oled_BarWidth(view);

    (void)ssd1315_gram_fill_rect(&s_oledHandle, 0u, OLED_BAR_Y0,
                                 127u, OLED_BAR_Y1, 0u);        /* 清空条区 */
    (void)ssd1315_gram_fill_rect(&s_oledHandle, 0u, OLED_BAR_Y0,
                                 127u, OLED_BAR_Y0, 1u);        /* 上边框 */
    (void)ssd1315_gram_fill_rect(&s_oledHandle, 0u, OLED_BAR_Y1,
                                 127u, OLED_BAR_Y1, 1u);        /* 下边框 */
    (void)ssd1315_gram_fill_rect(&s_oledHandle, 0u, OLED_BAR_Y0,
                                 0u, OLED_BAR_Y1, 1u);          /* 左端点 */
    (void)ssd1315_gram_fill_rect(&s_oledHandle, 127u, OLED_BAR_Y0,
                                 127u, OLED_BAR_Y1, 1u);        /* 右端点 */
    if (w > 0uL)
    {
        (void)ssd1315_gram_fill_rect(&s_oledHandle, 1u, (uint8_t)(OLED_BAR_Y0 + 1u),
                                     (uint8_t)(1u + w), (uint8_t)(OLED_BAR_Y1 - 1u), 1u);
    }
}

/**
  * @brief  把驱动显存的一页（128 字节）推送到屏幕
  * @note   用页寻址 + 一次 I2C 事务写 128 字节显存数据，比驱动自带的
  *         gram_update（逐字节事务）快约一个数量级，是无感刷新的
  *         关键实现；页数据取自驱动句柄内的 gram 缓冲。
  * @param  page 页号（0~7）
  * @retval 0=成功，1=任一 I2C 命令/数据写失败
  */
static uint8_t Oled_FlushPage(uint8_t page)
{
    uint8_t buf[128];
    uint8_t n;

    for (n = 0u; n < 128u; n++)                                   /* 从驱动 gram 取整页数据 */
    {
        buf[n] = s_oledHandle.gram[n][page];
    }

    if ((ssd1315_set_page_address(&s_oledHandle, page) != 0u) ||  /* 定位页 */
        (ssd1315_set_low_column_start_address(&s_oledHandle, 0x00u) != 0u) ||   /* 列低 4 位 = 0 */
        (ssd1315_set_high_column_start_address(&s_oledHandle, 0x00u) != 0u))    /* 列高 3 位 = 0 */
    {
        return 1u;
    }

    return (ssd1315_write_data(&s_oledHandle, buf, 128u) != 0u) ? 1u : 0u;      /* 一次事务写整页 */
}

/**
  * @brief  把屏幕上一段 y 坐标区间占用的显存页推送到屏幕
  * @note   UI 区域（标题栏/参数行/进度条）高度不一且可能横跨多个显存页
  *         （每页 8 行），本函数按 y 区间自动计算页范围并逐页推送。
  * @param  y0 起始 y 坐标（含）
  * @param  y1 结束 y 坐标（含）
  * @retval 0=成功，1=推送失败
  */
static uint8_t Oled_FlushRange(uint8_t y0, uint8_t y1)
{
    uint8_t page0 = (uint8_t)(y0 / 8u);                           /* 起始页 */
    uint8_t page1 = (uint8_t)(y1 / 8u);                           /* 结束页 */
    uint8_t page;

    for (page = page0; page <= page1; page++)
    {
        if (Oled_FlushPage(page) != 0u)
        {
            return 1u;
        }
    }
    return 0u;
}

/**
  * @brief  OLED UI 任务：运行态监控（约束 2~4）+ 行差分无感刷新
  * @note   每 200ms 一个周期：
  *         1. 取系统状态快照，执行运行态监控规则——
  *            约束 2：运行中检测到当前水温高于目标温度 → 转空闲；
  *            约束 3/4：运行中当前时间每 1s 加 1 且不超过目标时间，
  *            达到目标时间 → 转空闲（目标时间为 0 视为不设限）；
  *            IDLE→RUN 切换瞬间把当前时间清零重新计时；
  *         2. 重新取快照，按区域（标题栏/3 参数行/进度条）生成签名，与
  *            签名缓存比对，仅重绘变化的区域（写驱动显存 + 推送对应页），
  *            实现无感刷新；参数区聚焦行整行反白高亮。
  * @param  argument 未使用
  * @retval 无
  */
  static void OledUiTask(void *argument)
  {
     (void)argument;
     SysState_t view;
     char       sig[24u];
     uint8_t    idx;
     uint8_t    invert;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(OLED_UI_PERIOD_MS));

        /*---------- 第 1 步：运行态监控（约束 2~4） ----------*/
        SysTem_GetSnapshot(&view);

        if (view.State == SYS_STATE_RUN)
        {
            /* 运行开始瞬间（上一周期是空闲）：当前时间清零重新计时 */
            if (s_prevState == SYS_STATE_IDLE)
            {
                SysTem_SetCurrentTime(0u);
                view.CurrentTime = 0u;
                s_tickCnt        = 0u;
            }

            /* 约束 2：当前水温高于目标温度 → 立即转空闲（当前时间归零） */
            if (view.CurrentTemp > view.TargetTemp)
            {
                SysTem_SetCurrentTime(0u);          /* 中止运行：当前时间归零 */
                view.CurrentTime = 0u;
                SysTem_SetState(SYS_STATE_IDLE);
                view.State = SYS_STATE_IDLE;
                s_tickCnt  = 0u;
            }
            /* 约束 3：当前时间已达目标时间（兜底检查）→ 归零并转空闲 */
            else if ((view.TargetTime > 0u) && (view.CurrentTime >= view.TargetTime))
            {
                SysTem_SetCurrentTime(0u);          /* 当前时间归零 */
                view.CurrentTime = 0u;
                SysTem_SetState(SYS_STATE_IDLE);
                view.State = SYS_STATE_IDLE;
                s_tickCnt  = 0u;
            }
            else
            {
                /* 约束 4：每 1s 当前时间 +1，且不能超过目标时间 */
                s_tickCnt++;
                if (s_tickCnt >= OLED_TICKS_PER_SEC)
                {
                    uint32_t next = view.CurrentTime + 1u;

                    s_tickCnt = 0u;
                    if ((view.TargetTime > 0u) && (next >= view.TargetTime))
                    {
                        /* 达到目标时间：当前时间归零 → 转空闲 */
                        SysTem_SetCurrentTime(0u);
                        view.CurrentTime = 0u;
                        SysTem_SetState(SYS_STATE_IDLE);
                        view.State = SYS_STATE_IDLE;
                    }
                    else
                    {
                        SysTem_SetCurrentTime(next);
                        view.CurrentTime = next;
                    }
                }
            }
        }
        else
        {
            s_tickCnt = 0u;   /* 非运行态：节拍复位 */
        }
        s_prevState = view.State;

        /*---------- 第 2 步：区域差分无感刷新 ----------*/
        SysTem_GetSnapshot(&view);   /* 重新取快照，渲染含本周期修改后的最新值 */

        /* 标题栏（反白，ID 左 + 状态右对齐，下方带分隔线） */
        Oled_BuildHeader(&view, sig);
        if (strcmp(sig, s_sig[OLED_REG_HEADER]) != 0)             /* 签名有变化才动屏幕 */
        {
            Oled_DrawHeader(&view);
            (void)Oled_FlushRange(OLED_HEADER_Y0, OLED_SEP_Y);    /* 只推送该区域占用的显存页 */
            (void)strcpy(s_sig[OLED_REG_HEADER], sig);
        }

        /* 三行参数（8x16 大字，聚焦行整行反白高亮） */
        for (idx = 0u; idx < 3u; idx++)
        {
            invert = (view->Focus == (SysFocus_t)idx) ? 1u : 0u;
            Oled_BuildRow(idx, &view, invert, sig);
            if (strcmp(sig, s_sig[OLED_REG_ROW0 + idx]) != 0)
            {
                Oled_DrawRow(idx, &sig[1], invert);
                (void)Oled_FlushRange((uint8_t)(OLED_ROW_Y0 + idx * OLED_ROW_H),
                                      (uint8_t)(OLED_ROW_Y0 + idx * OLED_ROW_H + OLED_ROW_H - 1u));
                (void)strcpy(s_sig[OLED_REG_ROW0 + idx], sig);
            }
        }

        /* 底部时间进度条（运行态随时间推进填充） */
        Oled_BuildBar(&view, sig);
        if (strcmp(sig, s_sig[OLED_REG_BAR]) != 0)
        {
            Oled_DrawBar(&view);
            (void)Oled_FlushRange(OLED_BAR_Y0, OLED_BAR_Y1);
            (void)strcpy(s_sig[OLED_REG_BAR], sig);
        }
    }
}

/**
  * @brief  OLED 模块初始化：完成芯片配置并创建 UI 刷新任务
  * @note   内部含阻塞式 I2C 读写与毫秒级延时，须在任务上下文或调度器启动前
  *         调用；任一步失败立即返回 0 并打印定位信息。UI 任务优先级 1（低），
  *         只在行内容变化时占用 I2C 总线。
  * @retval 1=成功，0=失败
  */
uint8_t Oled_Init(void)
{
    /* 1. 清空句柄并链接全部驱动回调（含 SPI/GPIO 空桩，满足驱动指针检查） */
    DRIVER_SSD1315_LINK_INIT(&s_oledHandle, ssd1315_handle_t);
    DRIVER_SSD1315_LINK_IIC_INIT(&s_oledHandle, Oled_IicInit);
    DRIVER_SSD1315_LINK_IIC_DEINIT(&s_oledHandle, Oled_IicDeinit);
    DRIVER_SSD1315_LINK_IIC_WRITE(&s_oledHandle, Oled_IicWrite);
    DRIVER_SSD1315_LINK_SPI_INIT(&s_oledHandle, Oled_SpiInitStub);
    DRIVER_SSD1315_LINK_SPI_DEINIT(&s_oledHandle, Oled_SpiDeinitStub);
    DRIVER_SSD1315_LINK_SPI_WRITE_COMMAND(&s_oledHandle, Oled_SpiWriteCmdStub);
    DRIVER_SSD1315_LINK_SPI_COMMAND_DATA_GPIO_INIT(&s_oledHandle, Oled_SpiGpioInitStub);
    DRIVER_SSD1315_LINK_SPI_COMMAND_DATA_GPIO_DEINIT(&s_oledHandle, Oled_SpiGpioDeinitStub);
    DRIVER_SSD1315_LINK_SPI_COMMAND_DATA_GPIO_WRITE(&s_oledHandle, Oled_SpiGpioWriteStub);
    DRIVER_SSD1315_LINK_RESET_GPIO_INIT(&s_oledHandle, Oled_ResetGpioInitStub);
    DRIVER_SSD1315_LINK_RESET_GPIO_DEINIT(&s_oledHandle, Oled_ResetGpioDeinitStub);
    DRIVER_SSD1315_LINK_RESET_GPIO_WRITE(&s_oledHandle, Oled_ResetGpioWriteStub);
    DRIVER_SSD1315_LINK_DEBUG_PRINT(&s_oledHandle, Oled_DebugPrint);
    DRIVER_SSD1315_LINK_DELAY_MS(&s_oledHandle, Oled_DelayMs);

    /* 2. 选择 IIC 接口与器件地址（SA0 接地 → 0x78） */
    if (ssd1315_set_interface(&s_oledHandle, SSD1315_INTERFACE_IIC) != 0u)
    {
        printf("[Oled] set interface failed!\r\n");
        return 0;
    }
    if (ssd1315_set_addr_pin(&s_oledHandle, SSD1315_ADDR_SA0_0) != 0u)
    {
        printf("[Oled] set addr pin failed!\r\n");
        return 0;
    }

    /* 3. 芯片初始化（本驱动版本的 init 仅完成回调检查+复位+IIC 就绪，
     *    不含寄存器命令序列，必须由本模块补齐标准配置后才能点亮） */
    if (ssd1315_init(&s_oledHandle) != 0u)
    {
        printf("[Oled] ssd1315 init failed!\r\n");
        return 0;
    }

    /* 4. 补齐 SSD1315 标准初始化命令序列（与 SSD1306 128x64 模块通用）：
     *    显示时钟 → 多路复用 → 偏移/起始行 → 电荷泵 → 寻址模式 →
     *    扫描方向/COM 引脚 → 对比度/预充电 → 关闪 → 显示模式 → 显示开 */
    if ((ssd1315_set_display_clock(&s_oledHandle, 0x08u, 0x00u) != 0u) ||          /* 振荡器中档、不分频 */
        (ssd1315_set_multiplex_ratio(&s_oledHandle, 0x3Fu) != 0u) ||               /* 64 行多路复用 */
        (ssd1315_set_display_offset(&s_oledHandle, 0x00u) != 0u) ||                /* COM 偏移 0 */
        (ssd1315_set_display_start_line(&s_oledHandle, 0x00u) != 0u) ||            /* 起始行 0 */
        (ssd1315_set_charge_pump(&s_oledHandle,
                                 SSD1315_CHARGE_PUMP_ENABLE,
                                 SSD1315_CHARGE_PUMP_MODE_9P0V) != 0u) ||          /* 开内部升压（无 VCC 方案必需） */
        (ssd1315_set_memory_addressing_mode(&s_oledHandle,
                                            SSD1315_MEMORY_ADDRESSING_MODE_PAGE) != 0u) || /* 页寻址（按页推送显存） */
        (ssd1315_set_segment_remap(&s_oledHandle,
                                   SSD1315_SEGMENT_COLUMN_ADDRESS_127) != 0u) ||       /* 列 127 映射 SEG0，正常左右方向 */
        (ssd1315_set_scan_direction(&s_oledHandle,
                                    SSD1315_SCAN_DIRECTION_COMN_1_START) != 0u) ||     /* COM N-1 扫描，正常上下方向 */
        (ssd1315_set_com_pins_hardware_conf(&s_oledHandle,
                                            SSD1315_PIN_CONF_ALTERNATIVE,
                                            SSD1315_LEFT_RIGHT_REMAP_DISABLE) != 0u) || /* 128x64 标准 COM 引脚配置 */
        (ssd1315_set_contrast(&s_oledHandle, 0xCFu) != 0u) ||                      /* 对比度 0xCF */
        (ssd1315_set_precharge_period(&s_oledHandle, 0x01u, 0x0Fu) != 0u) ||       /* 预充电 1/15 DCLK */
        (ssd1315_set_deselect_level(&s_oledHandle,
                                    SSD1315_DESELECT_LEVEL_0P77) != 0u) ||         /* VCOMH 0.77xVCC */
        (ssd1315_set_entire_display(&s_oledHandle,
                                    SSD1315_ENTIRE_DISPLAY_OFF) != 0u) ||          /* 正常显示（跟随显存，非全亮） */
        (ssd1315_set_display_mode(&s_oledHandle,
                                  SSD1315_DISPLAY_MODE_NORMAL) != 0u) ||           /* 非反显 */
        (ssd1315_set_display(&s_oledHandle, SSD1315_DISPLAY_ON) != 0u))            /* 打开显示 */
    {
        printf("[Oled] config sequence failed!\r\n");
        return 0;
    }

    /* 5. 清屏（显存清零并刷新，确保上电画面干净） */
    if (ssd1315_clear(&s_oledHandle) != 0u)
    {
        printf("[Oled] clear failed!\r\n");
        return 0;
    }

    /* 6. 清空区域签名缓存：首个 UI 周期全量绘制一次 */
    (void)memset(s_sig, 0, sizeof(s_sig));

    /* 7. 创建 UI 刷新任务：优先级 1（低于采集/按键等业务任务），栈 512 字 */
    if (xTaskCreate(OledUiTask, "OledUI", 512u, NULL, 1u, NULL) != pdPASS)
    {
        printf("[Oled] UI task create failed!\r\n");
        return 0;
    }

    return 1;   /* 全部成功 */
}

/**
  * @brief  设置 UI 参数聚焦点（供将来的按键任务调用）
  * @note   通过 SysTem_SetFocus 写入全局状态（按键任务仍为唯一写者），
  *         UI 任务下一周期自动重绘 '>' 标记；非法值不生效。
  * @param  focus 聚焦点（SYS_FOCUS_TEMP/SYS_FOCUS_RPM/SYS_FOCUS_TIME）
  * @retval 无
  */
void Oled_SetFocus(SysFocus_t focus)
{
    if (focus == SYS_FOCUS_TEMP || focus == SYS_FOCUS_RPM || focus == SYS_FOCUS_TIME)
    {
        SysTem_SetFocus(focus);
    }
}

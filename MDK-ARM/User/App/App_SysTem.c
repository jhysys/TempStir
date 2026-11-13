/**
  ******************************************************************************
  * @file    App_SysTem.c
  * @brief   系统状态全局数据模块实现（结构体存储 + 多任务并发安全访问）
  * @note    并发保护实现细节：
  *          1. 单成员 setter/getter：临界区保护，临界区内只有一条赋值语句，
  *             几十个时钟周期，不影响实时性；
  *          2. 整体快照：临界区内结构体整体赋值（编译为块拷贝），保证
  *             各成员读到同一时刻的值；
  *          3. 临界区内绝不调用任何 FreeRTOS 阻塞 API；
  *          4. g_sysState 为静态存储、零初始化，调度器启动前由
  *             SysTem_Init() 赋安全默认值。
  ******************************************************************************
  */
#include "App_SysTem.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include <string.h>
#include "drv_m24c02.h"
#include "bsp_i2c2.h"

/*------------------------------ 私有宏定义（参数持久化） ------------------*/
#define SYS_PARAMS_BLOCK_SIZE     32u     /* 参数块大小：32 字节（M24C02 两页） */
#define SYS_PARAMS_BLOCKA_ADDR    0x00u   /* 参数块 A 首地址：0x00~0x1F */
#define SYS_PARAMS_BLOCKB_ADDR    0x20u   /* 参数块 B 首地址：0x20~0x3F */
#define SYS_PARAMS_MAGIC          0x5AA5u /* 块有效魔数：区分空白/损坏块 */
#define SYS_PARAMS_SAVE_DELAY_MS  3000u   /* 参数变化后延时确认时间（ms） */

/* 参数块布局（共 32 字节，无填充）：
 * [0-1]魔数 [2-3]写入序号 [4-7]目标温度 [8-11]目标转速 [12-15]目标时间
 * [16-29]保留 [30-31]CRC16（覆盖前 30 字节） */
typedef struct
{
  uint16_t magic;        /* 块有效魔数 SYS_PARAMS_MAGIC */
  uint16_t seq;          /* 写入序号，越大越新（回绕用带符号差比较） */
  float    TargetTemp;   /* 目标温度（摄氏度） */
  float    TargetRPM;    /* 目标转速（转/分钟） */
  uint32_t TargetTime;   /* 目标时间（秒） */
  uint8_t  reserved[14]; /* 保留，预留将来扩展参数，写 0 */
  uint16_t crc;          /* CRC16-Modbus 校验值 */
} SysParamsBlock_t;

/* 编译期确认参数块大小为 32 字节，成员增减导致布局漂移时编译报错 */
typedef char SysTem_ParamsBlockSizeCheck[(sizeof(SysParamsBlock_t) == SYS_PARAMS_BLOCK_SIZE) ? 1 : -1];

/*------------------------------ 私有变量 ---------------------------------*/
/* 全局系统状态：全项目唯一实例，各任务通过本模块接口访问，禁止 extern 直改 */
static SysState_t g_sysState;

/* 参数持久化运行状态 */
static TimerHandle_t s_saveTimer  = NULL; /* 3s 延时保存单次定时器 */
static uint8_t       s_activeBlock = 0u;  /* 当前有效块：0=块A，1=块B */
static uint16_t      s_seq         = 0u;  /* 最新写入序号 */

/*------------------------------ 私有函数 ---------------------------------*/
static uint32_t SysTem_CalcUID32(void);
static uint16_t SysTem_Crc16(const uint8_t *data, uint32_t len);
static void     SysTem_LoadParams(void);
static void     SysTem_SaveTimerCb(TimerHandle_t timer);
static void     SysTem_ParamsScheduleSave(void);

/**
  * @brief  读取片内 96 位唯一 ID（UID）并压缩为 32 位设备标识
  * @note   STM32F103 全系列 96 位 UID 固定位于地址 0x1FFFF7E8 起的
  *         连续 3 个 32 位字，出厂固化、只读、全球唯一。
  *         压缩算法：对 12 字节 UID 做软件 CRC32（多项式 0x04C11DB7 反射
  *         形式 0xEDB88320），抗碰撞特性优于简单异或折叠。
  *         本函数为纯读取操作，可在任意上下文调用。
  * @retval 32 位设备标识（同一颗芯片每次上电结果相同，不同芯片概率不同）
  */
static uint32_t SysTem_CalcUID32(void)
{
  /* F103 片内 UID 固定地址：3 个 32 位字（96 位） */
  const uint32_t uidAddr[3] = { 0x1FFFF7E8u, 0x1FFFF7ECu, 0x1FFFF7F0u };
  uint32_t crc = 0xFFFFFFFFu;   /* CRC32 初始值 */
  uint32_t i;
  uint32_t w;

  for (i = 0u; i < 3u; i++)
  {
    uint8_t byte;
    uint8_t shift;

    w = *(volatile const uint32_t *)uidAddr[i];   /* 读取 UID 字（只读内存） */

    /* 按 4 个字节逐字节计算 CRC32（小端序，与字节流传输一致） */
    for (shift = 0u; shift < 4u; shift++)
    {
      uint8_t bit;

      byte = (uint8_t)(w >> (shift * 8u));
      crc ^= byte;
      for (bit = 0u; bit < 8u; bit++)
      {
        if ((crc & 1u) != 0u)
        {
          crc = (crc >> 1u) ^ 0xEDB88320u;   /* 反射多项式 */
        }
        else
        {
          crc >>= 1u;
        }
      }
    }
  }

  return ~crc;   /* CRC32 最终取反，得到 32 位设备标识 */
}

/**
  * @brief  计算 CRC16-Modbus 校验值
  * @note   多项式 0xA001（反射形式），初始值 0xFFFF，与 Modbus RTU 一致；
  *         用于参数块完整性校验，防止 EEPROM 数据损坏被误用。
  * @param  data 待校验数据首地址
  * @param  len  待校验字节数
  * @retval 16 位校验值
  */
static uint16_t SysTem_Crc16(const uint8_t *data, uint32_t len)
{
  uint16_t crc = 0xFFFFu;

  while (len-- > 0u)
  {
    uint8_t bit;

    crc ^= *data++;
    for (bit = 0u; bit < 8u; bit++)
    {
      if ((crc & 1u) != 0u)
      {
        crc = (crc >> 1u) ^ 0xA001u;   /* 反射多项式 */
      }
      else
      {
        crc >>= 1u;
      }
    }
  }
  return crc;
}

/**
  * @brief  从 EEPROM 恢复目标参数（上电时调用，此时无并发可直接写状态体）
  * @note   双块策略：读块 A、块 B，各自做魔数+CRC16 校验，都有效时取写入
  *         序号新的那块（序号用带符号差比较，兼容 65536 回绕）；两块都无效
  *         （首次上电/数据损坏）则保持默认值。恢复成功同时更新当前有效块
  *         与序号，作为下次保存的基准。
  * @retval 无
  */
static void SysTem_LoadParams(void)
{
  uint8_t         buf[SYS_PARAMS_BLOCK_SIZE];
  SysParamsBlock_t blkA;
  SysParamsBlock_t blkB;
  uint8_t         okA = 0u;
  uint8_t         okB = 0u;
  const SysParamsBlock_t *newest;

  /* 读块 A 并校验 */
  if ((M24C02_Read(SYS_PARAMS_BLOCKA_ADDR, buf, SYS_PARAMS_BLOCK_SIZE) == 1u) &&
      (memcpy(&blkA, buf, sizeof(blkA)), blkA.magic == SYS_PARAMS_MAGIC) &&
      (blkA.crc == SysTem_Crc16(buf, SYS_PARAMS_BLOCK_SIZE - 2u)))
  {
    okA = 1u;
  }

  /* 读块 B 并校验 */
  if ((M24C02_Read(SYS_PARAMS_BLOCKB_ADDR, buf, SYS_PARAMS_BLOCK_SIZE) == 1u) &&
      (memcpy(&blkB, buf, sizeof(blkB)), blkB.magic == SYS_PARAMS_MAGIC) &&
      (blkB.crc == SysTem_Crc16(buf, SYS_PARAMS_BLOCK_SIZE - 2u)))
  {
    okB = 1u;
  }

  /* 选择最新有效块 */
  if ((okA == 0u) && (okB == 0u))
  {
    /* 双块均无效：首次上电或数据损坏，保持默认值，从块 A 重新开始 */
    printf("[SysTem] EEPROM no valid params, use defaults\r\n");
    s_activeBlock = 0u;
    s_seq         = 0u;
    return;
  }

  if (okA == 0u)
  {
    newest = &blkB;
    s_activeBlock = 1u;
  }
  else if (okB == 0u)
  {
    newest = &blkA;
    s_activeBlock = 0u;
  }
  else
  {
    /* 双块都有效：带符号差比较序号，兼容 65536 回绕 */
    if ((int16_t)((uint16_t)(blkA.seq - blkB.seq)) > 0)
    {
      newest = &blkA;
      s_activeBlock = 0u;
    }
    else
    {
      newest = &blkB;
      s_activeBlock = 1u;
    }
  }

  s_seq = newest->seq;

  /* 恢复参数到状态体（上电阶段无并发，可直接写入） */
  g_sysState.TargetTemp = newest->TargetTemp;
  g_sysState.TargetRPM  = newest->TargetRPM;
  g_sysState.TargetTime = newest->TargetTime;

  printf("[SysTem] params restored: temp=%d, rpm=%d, time=%u\r\n",
         (int)newest->TargetTemp, (int)newest->TargetRPM,
         (unsigned int)newest->TargetTime);
}

/**
  * @brief  立即把当前目标参数（目标温度/目标转速/目标时间）持久化到 M24C02
  * @note   双块轮换写入：先写当前有效块的备用块，再回读校验，通过后才切换
  *         有效块——任何时刻两块中至少有一块数据完整，写过程掉电不丢参数。
  *         仅允许任务上下文调用（内部含阻塞式 I2C 读写与写周期等待）；
  *         参数 setter 内部已实现"变化才写 + 3s 延时确认写"，此接口供
  *         特殊场景手动触发保存。
  * @retval 1=保存成功，0=保存失败（器件未就绪/写入失败/回读校验失败）
  */
uint8_t SysTem_SaveParams(void)
{
  uint8_t          buf[SYS_PARAMS_BLOCK_SIZE];
  uint8_t          rdbuf[SYS_PARAMS_BLOCK_SIZE];
  SysParamsBlock_t chk;
  uint8_t          target;    /* 本次写入的备用块编号 */
  uint8_t          addr;      /* 本次写入的首地址 */

  /* 写入前确认 EEPROM 在线，避免对故障器件做无意义写操作 */
  if (M24C02_IsReady() == 0u)
  {
    printf("[SysTem] EEPROM not ready, save failed\r\n");
    return 0;
  }

  /* 打包参数块：序号递增，CRC 覆盖前 30 字节 */
  {
    SysParamsBlock_t blk;

    blk.magic = SYS_PARAMS_MAGIC;
    blk.seq   = (uint16_t)(s_seq + 1u);

    /* 临界区内一次性取齐三个参数，保证块内数据为同一时刻状态 */
    taskENTER_CRITICAL();
    blk.TargetTemp = g_sysState.TargetTemp;
    blk.TargetRPM  = g_sysState.TargetRPM;
    blk.TargetTime = g_sysState.TargetTime;
    taskEXIT_CRITICAL();

    (void)memset(blk.reserved, 0, sizeof(blk.reserved));
    blk.crc = SysTem_Crc16((const uint8_t *)&blk, SYS_PARAMS_BLOCK_SIZE - 2u);
    (void)memcpy(buf, &blk, sizeof(blk));
  }

  /* 写入备用块（当前有效块的另一块），实现双块轮换 */
  target = (s_activeBlock == 0u) ? 1u : 0u;
  addr   = (target == 0u) ? SYS_PARAMS_BLOCKA_ADDR : SYS_PARAMS_BLOCKB_ADDR;

  if (M24C02_Write(addr, buf, SYS_PARAMS_BLOCK_SIZE) != 1u)
  {
    printf("[SysTem] EEPROM write failed (block %c)\r\n", (target == 0u) ? 'A' : 'B');
    return 0;
  }

  /* 回读校验：确认落盘数据完整且为本次写入 */
  if ((M24C02_Read(addr, rdbuf, SYS_PARAMS_BLOCK_SIZE) != 1u) ||
      (memcpy(&chk, rdbuf, sizeof(chk)), chk.magic != SYS_PARAMS_MAGIC) ||
      (chk.crc != SysTem_Crc16(rdbuf, SYS_PARAMS_BLOCK_SIZE - 2u)) ||
      (chk.seq != (uint16_t)(s_seq + 1u)))
  {
    printf("[SysTem] EEPROM verify failed, keep old block\r\n");
    return 0;   /* 校验失败不切换有效块，旧块数据仍然完好 */
  }

  /* 保存成功：切换有效块并更新序号 */
  s_activeBlock = target;
  s_seq         = (uint16_t)(s_seq + 1u);
  printf("[SysTem] params saved to block %c, seq=%u\r\n",
         (s_activeBlock == 0u) ? 'A' : 'B', (unsigned int)s_seq);
  return 1;
}

/**
  * @brief  参数延时保存定时器回调（运行于定时器服务任务上下文）
  * @note   参数 setter 检测到变化后启动 3s 单次定时器，3s 内无新变化才
  *         到期进入本回调执行真正的 EEPROM 落盘，实现"变化才写 + 延时确认"。
  * @param  timer 定时器句柄（未使用）
  * @retval 无
  */
static void SysTem_SaveTimerCb(TimerHandle_t timer)
{
  (void)timer;
  (void)SysTem_SaveParams();
}

/**
  * @brief  参数变化后调度延时保存（重置 3s 定时器）
  * @note   定时器已激活时 xTimerStart 会重新计时，因此连续多次修改参数
  *         只会在最后一次修改 3s 后落盘一次；调度器未启动（如初始化阶段
  *         直接赋值）时不投递，避免误触发。
  * @retval 无
  */
static void SysTem_ParamsScheduleSave(void)
{
  if ((s_saveTimer != NULL) &&
      (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
  {
    (void)xTimerStart(s_saveTimer, 0);
  }
}

/**
  * @brief  系统状态模块初始化：给全部成员赋安全默认值并恢复持久化参数
  * @note   必须在 vTaskStartScheduler() 之前调用，此时无并发，可直接写入；
  *         静态变量默认零值已对应"空闲/不聚焦"的安全组合，此处再显式赋值
  *         业务默认参数，保证上电即处于确定状态。
  *         设备 ID 取自片内 96 位 UID 压缩值；随后从 M24C02 恢复上次保存的
  *         目标参数（双块 CRC 校验取新）。
  * @retval 1=成功，0=失败
  */
uint8_t SysTem_Init(void)
{
  g_sysState.DeviceID    = SysTem_CalcUID32();  /* 设备 ID = 片内 UID 压缩值 */
  g_sysState.TargetTemp  = 0.0f;           /* 目标温度上电默认 0（待用户/上位机设定） */
  g_sysState.CurrentTemp = 0.0f;           /* 当前温度等采集任务更新 */
  g_sysState.TargetRPM   = 0.0f;           /* 目标转速默认停止 */
  g_sysState.CurrentRPM  = 0.0f;           /* 当前转速等采集任务更新 */
  g_sysState.TargetTime  = 0u;             /* 目标时间默认 0 秒 */
  g_sysState.CurrentTime = 0u;             /* 已运行时间清零 */
  g_sysState.Focus       = SYS_FOCUS_TEMP; /* 上电默认聚焦目标温度 */
  g_sysState.State       = SYS_STATE_IDLE; /* 上电默认空闲状态 */

  /* 初始化 I2C2 BSP：创建跨任务互斥量（调度器启动前创建互斥量合法） */
  Bsp_I2c2Init();

  /* 创建参数延时保存定时器：单次触发，3s 后落盘（xTimerCreate 允许在调度器前调用） */
  s_saveTimer = xTimerCreate("ParamSave",
                             pdMS_TO_TICKS(SYS_PARAMS_SAVE_DELAY_MS),
                             pdFALSE, NULL, SysTem_SaveTimerCb);
  if (s_saveTimer == NULL)
  {
    printf("[SysTem] save timer create failed!\r\n");
    return 0;
  }

  /* 从 M24C02 恢复上次保存的目标参数（此时无并发，可直接写状态体） */
  SysTem_LoadParams();

  return 1;   /* 初始化成功 */
}

/**
  * @brief  获取系统状态整体快照（多成员一致读）
  * @param  out 输出快照缓冲（调用方提供，不能为 NULL）
  * @retval 无
  */
void SysTem_GetSnapshot(SysState_t *out)
{
  if (out == (void *)0)
  {
    return;   /* 非法参数，直接返回，不触碰空指针 */
  }

  taskENTER_CRITICAL();      /* 进入临界区：期间无任务抢占、无中断修改状态 */
  {
    *out = g_sysState;       /* 结构体整体拷贝，保证各成员同一时刻取值 */
  }
  taskEXIT_CRITICAL();       /* 退出临界区 */
}

/*------------------------------ 单成员读取实现 ----------------------------*/

/**
  * @brief  读设备唯一标识符 ID
  * @retval 设备 ID 值
  */
uint32_t SysTem_GetDeviceID(void)
{
  uint32_t val;
  taskENTER_CRITICAL();
  val = g_sysState.DeviceID;
  taskEXIT_CRITICAL();
  return val;
}

/**
  * @brief  读目标温度
  * @retval 目标温度（摄氏度）
  */
float SysTem_GetTargetTemp(void)
{
  float val;
  taskENTER_CRITICAL();
  val = g_sysState.TargetTemp;
  taskEXIT_CRITICAL();
  return val;
}

/**
  * @brief  读当前温度
  * @retval 当前温度（摄氏度）
  */
float SysTem_GetCurrentTemp(void)
{
  float val;
  taskENTER_CRITICAL();
  val = g_sysState.CurrentTemp;
  taskEXIT_CRITICAL();
  return val;
}

/**
  * @brief  读目标转速
  * @retval 目标转速（转/分钟）
  */
float SysTem_GetTargetRPM(void)
{
  float val;
  taskENTER_CRITICAL();
  val = g_sysState.TargetRPM;
  taskEXIT_CRITICAL();
  return val;
}

/**
  * @brief  读当前转速
  * @retval 当前转速（转/分钟）
  */
float SysTem_GetCurrentRPM(void)
{
  float val;
  taskENTER_CRITICAL();
  val = g_sysState.CurrentRPM;
  taskEXIT_CRITICAL();
  return val;
}

/**
  * @brief  读目标时间
  * @retval 目标时间（秒）
  */
uint32_t SysTem_GetTargetTime(void)
{
  uint32_t val;
  taskENTER_CRITICAL();
  val = g_sysState.TargetTime;
  taskEXIT_CRITICAL();
  return val;
}

/**
  * @brief  读当前已运行时间
  * @retval 本次运行已进行时间（秒）
  */
uint32_t SysTem_GetCurrentTime(void)
{
  uint32_t val;
  taskENTER_CRITICAL();
  val = g_sysState.CurrentTime;
  taskEXIT_CRITICAL();
  return val;
}

/**
  * @brief  读参数聚焦点
  * @retval 聚焦点：SYS_FOCUS_TEMP / SYS_FOCUS_RPM / SYS_FOCUS_TIME
  */
SysFocus_t SysTem_GetFocus(void)
{
  SysFocus_t val;
  taskENTER_CRITICAL();
  val = g_sysState.Focus;
  taskEXIT_CRITICAL();
  return val;
}

/**
  * @brief  读系统状态
  * @retval 系统状态：SYS_STATE_IDLE / SYS_STATE_RUN / SYS_STATE_FAULT
  */
SysRunState_t SysTem_GetState(void)
{
  SysRunState_t val;
  taskENTER_CRITICAL();
  val = g_sysState.State;
  taskEXIT_CRITICAL();
  return val;
}

/*------------------------------ 单成员写入实现 ----------------------------*/

/**
  * @brief  写设备唯一标识符 ID
  * @param  id 设备 ID 值
  * @retval 无
  */
void SysTem_SetDeviceID(uint32_t id)
{
  taskENTER_CRITICAL();
  g_sysState.DeviceID = id;
  taskEXIT_CRITICAL();
}

/**
  * @brief  写目标温度
  * @param  temp 目标温度（摄氏度），业务上仅按键/Modbus 任务调用
  * @retval 无
  */
void SysTem_SetTargetTemp(float temp)
{
  uint8_t changed = 0u;

  taskENTER_CRITICAL();
  if (g_sysState.TargetTemp != temp)   /* 变化才写：值未变不触发保存 */
  {
    g_sysState.TargetTemp = temp;
    changed = 1u;
  }
  taskEXIT_CRITICAL();

  if (changed != 0u)
  {
    SysTem_ParamsScheduleSave();       /* 重置 3s 延时确认定时器 */
  }
}

/**
  * @brief  写当前温度
  * @param  temp 当前温度（摄氏度），业务上仅采集任务调用
  * @retval 无
  */
void SysTem_SetCurrentTemp(float temp)
{
  taskENTER_CRITICAL();
  g_sysState.CurrentTemp = temp;
  taskEXIT_CRITICAL();
}

/**
  * @brief  写目标转速
  * @param  rpm 目标转速（转/分钟），业务上仅按键/Modbus 任务调用
  * @retval 无
  */
void SysTem_SetTargetRPM(float rpm)
{
  uint8_t changed = 0u;

  taskENTER_CRITICAL();
  if (g_sysState.TargetRPM != rpm)     /* 变化才写：值未变不触发保存 */
  {
    g_sysState.TargetRPM = rpm;
    changed = 1u;
  }
  taskEXIT_CRITICAL();

  if (changed != 0u)
  {
    SysTem_ParamsScheduleSave();       /* 重置 3s 延时确认定时器 */
  }
}

/**
  * @brief  写当前转速
  * @param  rpm 当前转速（转/分钟），业务上仅采集任务调用
  * @retval 无
  */
void SysTem_SetCurrentRPM(float rpm)
{
  taskENTER_CRITICAL();
  g_sysState.CurrentRPM = rpm;
  taskEXIT_CRITICAL();
}

/**
  * @brief  写目标时间
  * @param  sec 目标时间（秒），业务上仅按键/Modbus 任务调用
  * @retval 无
  */
void SysTem_SetTargetTime(uint32_t sec)
{
  uint8_t changed = 0u;

  taskENTER_CRITICAL();
  if (g_sysState.TargetTime != sec)    /* 变化才写：值未变不触发保存 */
  {
    g_sysState.TargetTime = sec;
    changed = 1u;
  }
  taskEXIT_CRITICAL();

  if (changed != 0u)
  {
    SysTem_ParamsScheduleSave();       /* 重置 3s 延时确认定时器 */
  }
}

/**
  * @brief  写当前已运行时间
  * @param  sec 本次运行已进行时间（秒），业务上仅主控任务调用
  * @retval 无
  */
void SysTem_SetCurrentTime(uint32_t sec)
{
  taskENTER_CRITICAL();
  g_sysState.CurrentTime = sec;
  taskEXIT_CRITICAL();
}

/**
  * @brief  写参数聚焦点
  * @param  focus 聚焦点，业务上仅按键任务调用
  * @retval 无
  */
void SysTem_SetFocus(SysFocus_t focus)
{
  taskENTER_CRITICAL();
  g_sysState.Focus = focus;
  taskEXIT_CRITICAL();
}

/**
  * @brief  写系统状态
  * @param  state 系统状态，业务上仅主控任务调用
  * @retval 无
  */
void SysTem_SetState(SysRunState_t state)
{
  taskENTER_CRITICAL();
  g_sysState.State = state;
  taskEXIT_CRITICAL();
}

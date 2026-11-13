/**
  ******************************************************************************
  * @file    App_SysTem.h
  * @brief   系统状态全局数据模块接口（结构体存储 + 多任务并发安全访问）
  * @note    多任务并发方案（解决多任务抢占问题）：
  *          1. 所有成员均为 32 位对齐类型，单成员读写在 Cortex-M3 上为原子操作；
  *          2. 所有 setter/getter 内部用 FreeRTOS 临界区保护，兼容将来扩展
  *             64 位成员或位域，且临界区内无任何阻塞调用，开销极小；
  *          3. 需要同时读取多个相关成员（如 UI 刷屏）时，调用
  *             SysTem_GetSnapshot() 整体快照后使用快照，避免读到
  *             "半新半旧"的组合；
  *          4. 写入纪律：每个成员在业务上只有唯一的"写者任务"（如当前温度
  *             只由采集任务写、目标温度只由按键/Modbus 写），跨任务请求修改
  *             复杂联动状态时用队列命令，不直接多任务乱写。
  ******************************************************************************
  */
#ifndef __App_SysTem_H__
#define __App_SysTem_H__

#include <stdint.h>

/* 系统运行状态：三态 */
typedef enum
{
  SYS_STATE_IDLE  = 0,   /* 空闲状态：设备待机，未启动搅拌加热 */
  SYS_STATE_RUN   = 1,   /* 运行状态：正在执行搅拌/加热流程 */
  SYS_STATE_FAULT = 2,   /* 故障状态：检测到故障，停止运行等待处理 */
} SysRunState_t;

/* 参数聚焦点：UI/按键当前聚焦的参数项，将来在三个目标参数间来回切换 */
typedef enum
{
  SYS_FOCUS_TEMP = 0,    /* 聚焦：目标温度 */
  SYS_FOCUS_RPM  = 1,    /* 聚焦：目标转速 */
  SYS_FOCUS_TIME = 2,    /* 聚焦：目标时间 */
} SysFocus_t;

/* 系统状态结构体：全项目共用数据，成员按 32 位对齐排列，禁止中间插新成员 */
typedef struct
{
  uint32_t      DeviceID;      /* 设备唯一标识符 ID（暂定软件默认值，将来从 M24C02 读取） */
  float         TargetTemp;    /* 目标温度（摄氏度） */
  float         CurrentTemp;   /* 当前温度（摄氏度，采集任务唯一写者） */
  float         TargetRPM;     /* 目标转速（转/分钟） */
  float         CurrentRPM;    /* 当前转速（转/分钟，采集任务唯一写者） */
  uint32_t      TargetTime;    /* 目标时间（秒） */
  uint32_t      CurrentTime;   /* 当前时间：本次运行已进行时间（秒） */
  SysFocus_t    Focus;         /* 参数聚焦点：在目标温度/转速/时间间切换 */
  SysRunState_t State;         /* 系统状态：空闲/运行/故障 */
} SysState_t;

/**
  * @brief  系统状态模块初始化：给全部成员赋安全默认值
  * @note   必须在 vTaskStartScheduler() 之前调用，此时无并发，可直接写入。
  * @retval 1=成功，0=失败
  */
uint8_t SysTem_Init(void);

/**
  * @brief  获取系统状态整体快照（多成员一致读）
  * @note   临界区内整体拷贝到输出缓冲，保证一次调用读到的各成员相互一致；
  *         调用后应只使用快照成员，不要再直接访问全局状态。
  * @param  out 输出快照缓冲（调用方提供，不能为 NULL）
  * @retval 无
  */
void SysTem_GetSnapshot(SysState_t *out);

/*------------------------------ 参数持久化接口 ----------------------------*/
/* 目标温度/目标转速/目标时间持久化于 M24C02（I2C2）双参数块：
 * 块 A 0x00~0x1F、块 B 0x20~0x3F，含魔数+序号+CRC16 校验；
 * setter 检测到参数变化后自动 3s 延时确认写入，无需业务干预。 */

/**
  * @brief  立即把当前目标参数持久化到 M24C02（双块轮换写入 + 回读校验）
  * @note   仅任务上下文调用（内部含阻塞式 I2C 读写）；一般无需手动调用，
  *         setter 已实现"变化才写 + 3s 延时确认写"，此接口供特殊场景使用。
  * @retval 1=保存成功，0=保存失败（器件未就绪/写入失败/回读校验失败）
  */
uint8_t SysTem_SaveParams(void);

/*------------------------------ 单成员读取接口 ----------------------------*/
uint32_t      SysTem_GetDeviceID(void);     /* 读设备唯一标识符 ID */
float         SysTem_GetTargetTemp(void);   /* 读目标温度 */
float         SysTem_GetCurrentTemp(void);  /* 读当前温度 */
float         SysTem_GetTargetRPM(void);    /* 读目标转速 */
float         SysTem_GetCurrentRPM(void);   /* 读当前转速 */
uint32_t      SysTem_GetTargetTime(void);   /* 读目标时间（秒） */
uint32_t      SysTem_GetCurrentTime(void);  /* 读当前已运行时间（秒） */
SysFocus_t    SysTem_GetFocus(void);        /* 读参数聚焦点 */
SysRunState_t SysTem_GetState(void);        /* 读系统状态 */

/*------------------------------ 单成员写入接口 ----------------------------*/
/* 注意：仅允许该成员业务上的唯一写者任务调用 */
void SysTem_SetDeviceID(uint32_t id);        /* 写设备唯一标识符 ID */
void SysTem_SetTargetTemp(float temp);       /* 写目标温度（按键/Modbus 写） */
void SysTem_SetCurrentTemp(float temp);      /* 写当前温度（采集任务写） */
void SysTem_SetTargetRPM(float rpm);         /* 写目标转速（按键/Modbus 写） */
void SysTem_SetCurrentRPM(float rpm);        /* 写当前转速（采集任务写） */
void SysTem_SetTargetTime(uint32_t sec);     /* 写目标时间（秒） */
void SysTem_SetCurrentTime(uint32_t sec);    /* 写当前已运行时间（秒） */
void SysTem_SetFocus(SysFocus_t focus);      /* 写参数聚焦点（按键任务写） */
void SysTem_SetState(SysRunState_t state);   /* 写系统状态（主控任务写） */

#endif /* __App_SysTem_H__ */

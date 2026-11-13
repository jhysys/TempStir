#ifndef BSP_I2C2_H
#define BSP_I2C2_H
#include <stdint.h>
#include "stm32f1xx_hal.h"
/* 创建 I2C2 互斥量；应在 EEPROM/RTC 任务创建前调用一次。 */
void Bsp_I2c2Init(void);

/*
 * 以下接口均为阻塞式 HAL 封装，设备地址沿用 HAL 左移一位的约定；
 * 调度器运行后自动串行化访问，不得从中断服务函数调用。
 */
HAL_StatusTypeDef Bsp_I2c2MasterTransmit(uint16_t devAddress, const uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef Bsp_I2c2MemWrite(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize,
                                   const uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef Bsp_I2c2MemRead(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize,
                                  uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef Bsp_I2c2IsDeviceReady(uint16_t devAddress, uint32_t trials, uint32_t timeout);

#endif /* BSP_I2C2_H */

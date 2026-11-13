#include "bsp_i2c2.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

static SemaphoreHandle_t s_i2c2Mutex = NULL;

void Bsp_I2c2Init(void)
{
    if (s_i2c2Mutex == NULL)

    {
        s_i2c2Mutex = xSemaphoreCreateMutex();
    }
}

static void Bsp_I2c2Lock(void)
{
    /* 启动阶段尚无任务并发；调度器启动后才获取互斥量。 */
    if ((s_i2c2Mutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))

    {
        (void)xSemaphoreTake(s_i2c2Mutex, portMAX_DELAY);
    }
}

static void Bsp_I2c2Unlock(void)
{
    /* 与 Bsp_I2c2Lock() 条件严格匹配，未获取时绝不释放。 */
    if ((s_i2c2Mutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))

    {
        (void)xSemaphoreGive(s_i2c2Mutex);
    }
}

HAL_StatusTypeDef Bsp_I2c2MasterTransmit(uint16_t devAddress, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    HAL_StatusTypeDef status;

    Bsp_I2c2Lock();
    status = HAL_I2C_Master_Transmit(&hi2c2, devAddress, (uint8_t *)data, size, timeout);
    Bsp_I2c2Unlock();

    return status;
}

HAL_StatusTypeDef Bsp_I2c2MemWrite(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize,
                                   const uint8_t *data, uint16_t size, uint32_t timeout)
{
    HAL_StatusTypeDef status;

    Bsp_I2c2Lock();
    status = HAL_I2C_Mem_Write(&hi2c2, devAddress, memAddress, memAddSize, (uint8_t *)data, size, timeout);
    Bsp_I2c2Unlock();

    return status;
}

HAL_StatusTypeDef Bsp_I2c2MemRead(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize,
                                  uint8_t *data, uint16_t size, uint32_t timeout)
{
    HAL_StatusTypeDef status;

    Bsp_I2c2Lock();
    status = HAL_I2C_Mem_Read(&hi2c2, devAddress, memAddress, memAddSize, data, size, timeout);
    Bsp_I2c2Unlock();

    return status;
}

HAL_StatusTypeDef Bsp_I2c2IsDeviceReady(uint16_t devAddress, uint32_t trials, uint32_t timeout)
{
    HAL_StatusTypeDef status;

    Bsp_I2c2Lock();
    status = HAL_I2C_IsDeviceReady(&hi2c2, devAddress, trials, timeout);
    Bsp_I2c2Unlock();

    return status;
}

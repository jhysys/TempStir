/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "App_Buzzer.h"
#include "App_ADC.h"
#include "App_Encoder.h"
#include "App_Fan.h"
#include "App_Heat.h"
#include "App_Key.h"
#include "App_Motor.h"
#include "App_Oled.h"
#include "App_SysTem.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ===== 临时温度打印测试参数（验证完成后可整段删除） ===== */
#define TEMP_TEST_TASK_STACK    256u   /* 测试任务栈深度（字；含 printf 需留余量） */
#define TEMP_TEST_TASK_PRIO     2u     /* 测试任务优先级（与采集/控制任务同级） */
#define TEMP_TEST_PRINT_MS      1000u  /* 打印周期（ms） */
#define TEMP_TEST_DURATION_S    300u   /* 测试总时长（s）：5 分钟后自动结束并自删除 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void TempPrintTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  /* 系统状态模块初始化：赋安全默认值（须在启动调度器前完成，此时无并发） */
  if (SysTem_Init() != 1u)
  {
    printf("SysState init failed!\r\n");
    Error_Handler();
  }

  /* 蜂鸣器模块初始化：创建鸣叫命令队列与蜂鸣器任务（须在启动调度器前完成） */
  if (Buzzer_Init() != 1u)
  {
    printf("Buzzer init failed!\r\n");
    Error_Handler();
  }

  /* 风扇模块初始化：使能脚 PA1 置为低电平 + 创建状态联动任务
   * （系统运行 → 风扇启动，空闲/故障 → 风扇停止，须在启动调度器前完成） */
  if (Fan_Init() != 1u)
  {
    printf("Fan init failed!\r\n");
    Error_Handler();
  }

  /* 电机模块初始化：IN2 锁定低电平（仅正转），TIM4_CH3 输出 0% 占空比 +
   * 创建 PID 转速闭环任务（系统 RUN → 按目标转速闭环调速，IDLE/FAULT →
   * 停机；须在 MX_TIM4_Init 之后调用） */
  if (Motor_Init() != 1u)
  {
    printf("Motor init failed!\r\n");
    Error_Handler();
  }

  /* 编码器模块初始化：启动 TIM3 正交编码器（×4 解码）+ 创建 100ms 测速任务
   * （结果自动写入 SysTem 的 CurrentRPM，须在 MX_TIM3_Init 之后调用） */
  if (Encoder_Init() != 1u)
  {
    printf("Encoder init failed!\r\n");
    Error_Handler();
  }

  /* 温度采集模块初始化：ADC 自校准 + 创建采集任务（200ms 周期采集板温/水温，
   * 水温自动写入 SysTem 的 CurrentTemp；须在 MX_ADC1_Init 之后调用） */
  if (Adc_Init() != 1u)
  {
    printf("Adc init failed!\r\n");
    Error_Handler();
  }

  /* 加热模块初始化：清除硬件故障锁存、TIM2_CH1 输出 0% 占空比（不加热）+
   * 创建 PID 温度闭环任务（系统 RUN → 按目标水温闭环加热，IDLE/FAULT →
   * 停止；内置探头故障/超温/硬件锁存三重保护；须在 MX_TIM2_Init 之后调用） */
  if (Heat_Init() != 1u)
  {
    printf("Heat init failed!\r\n");
    Error_Handler();
  }

  /* OLED 模块初始化：点亮屏幕并显示系统状态 UI（阻塞式 I2C，须在启动调度器前完成） */
  if (Oled_Init() != 1u)
  {
    printf("Oled init failed!\r\n");
    Error_Handler();
  }

  /* 按键模块初始化：创建按键扫描任务（须在启动调度器前完成） */
  if (Key_Init() != 1u)
  {
    printf("Key init failed!\r\n");
    Error_Handler();
  }

  /* ===== 临时温度打印测试任务（验证完成后删除本段及 TempPrintTask 实现） ===== */
  if (xTaskCreate(TempPrintTask, "TempPrint", TEMP_TEST_TASK_STACK, NULL,
                  TEMP_TEST_TASK_PRIO, NULL) != pdPASS)
  {
    printf("Temp print task create failed!\r\n");
  }

  /* 在此创建其余业务任务，然后启动 FreeRTOS 调度器，此后代码不再返回 */
  vTaskStartScheduler();

  /* 只有调度器启动失败（如堆内存不足）才会执行到这里 */
  printf("FreeRTOS scheduler start failed!\r\n");
  Error_Handler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief  栈溢出钩子函数（FreeRTOSConfig.h 中 configCHECK_FOR_STACK_OVERFLOW=2 时必需）
 * @note   FreeRTOS 在任务切换时检测到任务栈溢出会调用此函数；
 *         此处通过 USART1 打印溢出的任务名后停机，便于定位问题。
 * @param  xTask    发生溢出的任务句柄
 * @param  pcTaskName 发生溢出的任务名
 * @retval 无
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  printf("[ERROR] Stack overflow in task: %s\r\n", pcTaskName);
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

/**
 * @brief  堆内存分配失败钩子函数（FreeRTOSConfig.h 中 configUSE_MALLOC_FAILED_HOOK=1 时必需）
 * @note   FreeRTOS 内核调用 pvPortMalloc 分配失败（如 FreeRTOS 堆 24KB 用尽）会调用此函数；
 *         此处打印提示后停机，便于发现堆配置过小的问题。
 * @retval 无
 */
void vApplicationMallocFailedHook(void)
{
  printf("[ERROR] FreeRTOS heap malloc failed!\r\n");
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

/**
 * @brief  临时温度打印辅助：格式化单个温度（保留 1 位小数，支持负温）
 * @param  name   通道名（"board"/"water"）
 * @param  temp   温度（℃）
 * @retval 无（直接 printf 输出，不换行）
 */
static void TempPrintOne(const char *name, float temp)
{
  int32_t t10 = (int32_t)(temp * 10.0f + ((temp >= 0.0f) ? 0.5f : -0.5f));

  if (t10 < 0)
  {
    printf("%s=-%d.%uC ", name, (int)((-t10) / 10), (unsigned int)((-t10) % 10));
  }
  else
  {
    printf("%s=%d.%uC ", name, (int)(t10 / 10), (unsigned int)(t10 % 10));
  }
}

/**
 * @brief  临时温度打印测试任务：周期打印板温/水温及传感器故障标志
 * @note   数据来自 App_ADC 模块的滤波后读数（与 SysTem 的 CurrentTemp
 *         同源），本任务只读不写。printf 用整数格式（%d/%u）拆分小数，
 *         不用 %f，避免浮点打印依赖。
 *         运行 TEMP_TEST_DURATION_S 后打印结束提示并自删除任务；
 *         期间水温传感器若断线/短路，fault 位会置 1 且温度保持最后有效值。
 *         验证完成后删除本函数、TempPrintOne、PFP 原型、PD 宏与 main()
 *         中的创建语句即可，不影响 App_ADC 模块本身。
 * @param  argument 未使用
 * @retval 无（时间到后任务自我删除）
 */
static void TempPrintTask(void *argument)
{
  uint32_t i;

  (void)argument;

  printf("\r\n===== Temp print test: %us, every %ums =====\r\n",
         (unsigned int)TEMP_TEST_DURATION_S, (unsigned int)TEMP_TEST_PRINT_MS);

  for (i = 0u; i < TEMP_TEST_DURATION_S; i++)
  {
    vTaskDelay(pdMS_TO_TICKS(TEMP_TEST_PRINT_MS));

    printf("[Temp] ");
    TempPrintOne("board", Adc_GetBoardTempC());
    TempPrintOne("water", Adc_GetWaterTempC());
    printf("raw(b/w)=%u/%u fault(b/w)=%u/%u\r\n",
           (unsigned int)Adc_GetBoardRaw(), (unsigned int)Adc_GetWaterRaw(),
           (unsigned int)Adc_GetBoardFault(), (unsigned int)Adc_GetWaterFault());
  }

  printf("[Temp] test finished, task deleted.\r\n");
  vTaskDelete(NULL);
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

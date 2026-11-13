/*
 * bsp_pins.h
 * 原理图网络名到 STM32F103RCT6 GPIO 的唯一映射表。
 */
#ifndef BSP_PINS_H
#define BSP_PINS_H

#include "main.h"

/*
 * 本文件只定义“网络连接与有效电平”，不配置 GPIO 电气属性。推挽、开漏、
 * 上拉/下拉、复用和 ADC 模式统一由 Core/Src/gpio.c 及 HAL MSP 初始化负责；
 * 应用层必须使用这些名称，不能再次硬编码端口、引脚号或有效电平。
 */

/* 四个按键：内部上拉，按下时接地，因此低电平有效。 */
#define KEY1_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_6
#define KEY2_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_7
#define KEY3_GPIO_Port GPIOC
#define KEY3_Pin GPIO_PIN_8
#define KEY4_GPIO_Port GPIOC
#define KEY4_Pin GPIO_PIN_9
#define KEY_PRESSED_STATE GPIO_PIN_RESET

/* OLED：I2C1，PB6=SCL、PB7=SDA，复用开漏。 */
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SDA_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7

/* LSM6DSM：SPI2，片选由普通推挽 GPIO 控制，中断输入位于 PC4/PC5。 */
#define LSM_CS_GPIO_Port GPIOB
#define LSM_CS_Pin GPIO_PIN_12
#define LSM_INT1_GPIO_Port GPIOC
#define LSM_INT1_Pin GPIO_PIN_4
#define LSM_INT2_GPIO_Port GPIOC
#define LSM_INT2_Pin GPIO_PIN_5
#define LSM_CS_ACTIVE_STATE GPIO_PIN_RESET
#define LSM_CS_INACTIVE_STATE GPIO_PIN_SET

/* 蜂鸣器：TIM1_CH1 PWM 输出。 */
#define BUZZ_GPIO_Port GPIOA
#define BUZZ_Pin GPIO_PIN_8

/* 电机：PB4/PB5 为 TIM3 编码器输入，PB8/PB9 对应 U7 的两个控制输入。 */
#define M1_ENC_A_GPIO_Port GPIOB
#define M1_ENC_A_Pin GPIO_PIN_4
#define M1_ENC_B_GPIO_Port GPIOB
#define M1_ENC_B_Pin GPIO_PIN_5

#define MOTOR1_GPIO_Port GPIOB
#define MOTOR1_Pin GPIO_PIN_8
#define MOTOR2_GPIO_Port GPIOB
#define MOTOR2_Pin GPIO_PIN_9
#define MOTOR2_FIXED_STATE GPIO_PIN_RESET

/* 加热与风扇：PA0 为 TIM2_CH1 加热 PWM，PA1 为风扇使能（.ioc 标签 MOTOR_ON）。 */
#define HEAT_PWM_GPIO_Port GPIOA
#define HEAT_PWM_Pin GPIO_PIN_0
#define MOTOR_ON_GPIO_Port GPIOA
#define MOTOR_ON_Pin GPIO_PIN_1
#define MOTOR_ON_ACTIVE_STATE GPIO_PIN_SET
#define MOTOR_ON_INACTIVE_STATE GPIO_PIN_RESET

/* ADC1 输入：板载热敏、液体热敏、加热电流以及 24 V 分压检测。 */
#define BOARD_ADC_GPIO_Port GPIOB
#define BOARD_ADC_Pin GPIO_PIN_0
#define LIQUID_ADC_GPIO_Port GPIOB
#define LIQUID_ADC_Pin GPIO_PIN_1
#define HEAT_I_ADC_GPIO_Port GPIOA
#define HEAT_I_ADC_Pin GPIO_PIN_4
#define SUPPLY_24V_ADC_GPIO_Port GPIOC
#define SUPPLY_24V_ADC_Pin GPIO_PIN_0

/* DS3231 INT/SQW 为开漏低有效输入，硬件已有外部上拉。 */
#define DS_INT_GPIO_Port GPIOC
#define DS_INT_Pin GPIO_PIN_12
#define DS_INT_ACTIVE_STATE GPIO_PIN_RESET

/*
 * U21 硬件故障锁存接口：FAULT_RST_N 为开漏复位输出，HEAT_FAULT_N 为
 * 低有效锁存状态输入。释放开漏输出时写高，由外部上拉建立高电平。
 */
#define FAULT_RST_N_GPIO_Port GPIOC
#define FAULT_RST_N_Pin GPIO_PIN_2
#define FAULT_RST_ASSERT_STATE GPIO_PIN_RESET
#define FAULT_RST_RELEASE_STATE GPIO_PIN_SET
#define HEAT_FAULT_N_GPIO_Port GPIOC
#define HEAT_FAULT_N_Pin GPIO_PIN_3
#define HEAT_FAULT_ACTIVE_STATE GPIO_PIN_RESET

#endif /* BSP_PINS_H */

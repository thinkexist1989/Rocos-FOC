/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay.h"
#include "task.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;


extern RTC_HandleTypeDef hrtc;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern unsigned long timer_counter;

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void _writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY_Pin GPIO_PIN_13
#define KEY_GPIO_Port GPIOC
#define DRV_FAULT_Pin GPIO_PIN_14
#define DRV_FAULT_GPIO_Port GPIOC
#define DRV_EN_Pin GPIO_PIN_15
#define DRV_EN_GPIO_Port GPIOC
#define SENSOR_A_Pin GPIO_PIN_0
#define SENSOR_A_GPIO_Port GPIOA
#define SENSOR_B_Pin GPIO_PIN_1
#define SENSOR_B_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define PWM_C_Pin GPIO_PIN_0
#define PWM_C_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define ENCODER_A_Pin GPIO_PIN_8
#define ENCODER_A_GPIO_Port GPIOA
#define ENCODER_B_Pin GPIO_PIN_9
#define ENCODER_B_GPIO_Port GPIOA
#define ENCODER_Z_Pin GPIO_PIN_10
#define ENCODER_Z_GPIO_Port GPIOA
#define PWM_AUX1_Pin GPIO_PIN_15
#define PWM_AUX1_GPIO_Port GPIOA
#define PWM_AUX2_Pin GPIO_PIN_3
#define PWM_AUX2_GPIO_Port GPIOB
#define PWM_A_Pin GPIO_PIN_4
#define PWM_A_GPIO_Port GPIOB
#define PWM_B_Pin GPIO_PIN_5
#define PWM_B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define PWM_RANGE 1800 //!< pwm range 1800 by Yang Luo
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

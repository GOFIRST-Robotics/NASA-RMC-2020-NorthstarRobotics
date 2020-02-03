/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define A_BLESSYOU_PotR_Pin GPIO_PIN_0
#define A_BLESSYOU_PotR_GPIO_Port GPIOA
#define A_BLESSYOU_PotL_Pin GPIO_PIN_1
#define A_BLESSYOU_PotL_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define GESUNDHEIT_HallL_Pin GPIO_PIN_5
#define GESUNDHEIT_HallL_GPIO_Port GPIOC
#define GESUNDHEIT_HallR_Pin GPIO_PIN_0
#define GESUNDHEIT_HallR_GPIO_Port GPIOB
#define GESUNDHEIT_LimitLI_Pin GPIO_PIN_1
#define GESUNDHEIT_LimitLI_GPIO_Port GPIOB
#define GESUNDHEIT_LimitLO_Pin GPIO_PIN_2
#define GESUNDHEIT_LimitLO_GPIO_Port GPIOB
#define GESUNDHEIT_LimitRI_Pin GPIO_PIN_10
#define GESUNDHEIT_LimitRI_GPIO_Port GPIOB
#define GESUNDHEIT_LimitRO_Pin GPIO_PIN_11
#define GESUNDHEIT_LimitRO_GPIO_Port GPIOB
#define R_DOOR_RV_Pin GPIO_PIN_12
#define R_DOOR_RV_GPIO_Port GPIOB
#define R_DOOR_FW_Pin GPIO_PIN_13
#define R_DOOR_FW_GPIO_Port GPIOB
#define BLESSYOU_LimitLL_Pin GPIO_PIN_14
#define BLESSYOU_LimitLL_GPIO_Port GPIOB
#define BLESSYOU_LimitLH_Pin GPIO_PIN_15
#define BLESSYOU_LimitLH_GPIO_Port GPIOB
#define BLESSYOU_LimitRL_Pin GPIO_PIN_6
#define BLESSYOU_LimitRL_GPIO_Port GPIOC
#define BLESSYOU_LimitRH_Pin GPIO_PIN_7
#define BLESSYOU_LimitRH_GPIO_Port GPIOC
#define DIGGER_Clock_Pin GPIO_PIN_8
#define DIGGER_Clock_GPIO_Port GPIOC
#define ACHOO_LimitLL_Pin GPIO_PIN_9
#define ACHOO_LimitLL_GPIO_Port GPIOC
#define ACHOO_LimitLH_Pin GPIO_PIN_8
#define ACHOO_LimitLH_GPIO_Port GPIOA
#define ACHOO_LimitRL_Pin GPIO_PIN_9
#define ACHOO_LimitRL_GPIO_Port GPIOA
#define ACHOO_LimitRH_Pin GPIO_PIN_10
#define ACHOO_LimitRH_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define R_GESUNDR_RV_Pin GPIO_PIN_15
#define R_GESUNDR_RV_GPIO_Port GPIOA
#define R_GESUNDR_FW_Pin GPIO_PIN_10
#define R_GESUNDR_FW_GPIO_Port GPIOC
#define R_GESUNDL_RV_Pin GPIO_PIN_11
#define R_GESUNDL_RV_GPIO_Port GPIOC
#define R_GESUNDL_FW_Pin GPIO_PIN_12
#define R_GESUNDL_FW_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define R_GESUNDEXT_RV_Pin GPIO_PIN_4
#define R_GESUNDEXT_RV_GPIO_Port GPIOB
#define R_GESUNDEXT_FW_Pin GPIO_PIN_5
#define R_GESUNDEXT_FW_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

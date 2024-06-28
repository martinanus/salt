/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define SIS_2_CT_BP_C_Pin GPIO_PIN_2
#define SIS_2_CT_BP_C_GPIO_Port GPIOE
#define SIS_2_CT_BP_M_Pin GPIO_PIN_3
#define SIS_2_CT_BP_M_GPIO_Port GPIOE
#define SIS_3_CT_BP_C_Pin GPIO_PIN_4
#define SIS_3_CT_BP_C_GPIO_Port GPIOE
#define SIS_3_FE_BP_M_Pin GPIO_PIN_5
#define SIS_3_FE_BP_M_GPIO_Port GPIOE
#define RS485_2_DIR_Pin GPIO_PIN_6
#define RS485_2_DIR_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define SIS_4_CT_BP_M_Pin GPIO_PIN_0
#define SIS_4_CT_BP_M_GPIO_Port GPIOF
#define SIS_4_FE_BP_M_Pin GPIO_PIN_1
#define SIS_4_FE_BP_M_GPIO_Port GPIOF
#define SIS_3_CT_BP_M_Pin GPIO_PIN_2
#define SIS_3_CT_BP_M_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define GPS_PW_ON_Pin GPIO_PIN_0
#define GPS_PW_ON_GPIO_Port GPIOA
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define RS485_1_DIR_Pin GPIO_PIN_4
#define RS485_1_DIR_GPIO_Port GPIOA
#define BUZZER_C_Pin GPIO_PIN_5
#define BUZZER_C_GPIO_Port GPIOA
#define CHOP_SEL_Pin GPIO_PIN_6
#define CHOP_SEL_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define CT_C_Pin GPIO_PIN_11
#define CT_C_GPIO_Port GPIOF
#define CT_EN_1_M_Pin GPIO_PIN_12
#define CT_EN_1_M_GPIO_Port GPIOF
#define CT_M_Pin GPIO_PIN_13
#define CT_M_GPIO_Port GPIOF
#define ON_SW_MAL_1_Pin GPIO_PIN_14
#define ON_SW_MAL_1_GPIO_Port GPIOF
#define CT_EN_2_M_Pin GPIO_PIN_15
#define CT_EN_2_M_GPIO_Port GPIOF
#define SIS_5_CT_BP_C_Pin GPIO_PIN_0
#define SIS_5_CT_BP_C_GPIO_Port GPIOG
#define SIS_5_FE_BP_C_Pin GPIO_PIN_1
#define SIS_5_FE_BP_C_GPIO_Port GPIOG
#define REG_2_C_Pin GPIO_PIN_7
#define REG_2_C_GPIO_Port GPIOE
#define ZONA_C_Pin GPIO_PIN_8
#define ZONA_C_GPIO_Port GPIOE
#define ON_SW_MAL_2_Pin GPIO_PIN_9
#define ON_SW_MAL_2_GPIO_Port GPIOE
#define FE_M_Pin GPIO_PIN_10
#define FE_M_GPIO_Port GPIOE
#define SPI4_CS_Pin GPIO_PIN_11
#define SPI4_CS_GPIO_Port GPIOE
#define SD_CD_Pin GPIO_PIN_15
#define SD_CD_GPIO_Port GPIOE
#define ON_SW_MAT_2_Pin GPIO_PIN_11
#define ON_SW_MAT_2_GPIO_Port GPIOB
#define ON_SW_MAT_1_Pin GPIO_PIN_12
#define ON_SW_MAT_1_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define FE_C_Pin GPIO_PIN_10
#define FE_C_GPIO_Port GPIOD
#define REG_1_C_Pin GPIO_PIN_11
#define REG_1_C_GPIO_Port GPIOD
#define REG_3_C_Pin GPIO_PIN_12
#define REG_3_C_GPIO_Port GPIOD
#define REG_4_C_Pin GPIO_PIN_13
#define REG_4_C_GPIO_Port GPIOD
#define REG_1_M_Pin GPIO_PIN_14
#define REG_1_M_GPIO_Port GPIOD
#define FE_EN_2_M_Pin GPIO_PIN_15
#define FE_EN_2_M_GPIO_Port GPIOD
#define SIS_2_FE_BP_M_Pin GPIO_PIN_2
#define SIS_2_FE_BP_M_GPIO_Port GPIOG
#define SIS_1_CT_BP_C_Pin GPIO_PIN_3
#define SIS_1_CT_BP_C_GPIO_Port GPIOG
#define FE_EN_1_M_Pin GPIO_PIN_4
#define FE_EN_1_M_GPIO_Port GPIOG
#define FE_DES_2_Pin GPIO_PIN_5
#define FE_DES_2_GPIO_Port GPIOG
#define FE_DES_1_Pin GPIO_PIN_6
#define FE_DES_1_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define CT_DES_1_Pin GPIO_PIN_8
#define CT_DES_1_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define ESP_EN_Pin GPIO_PIN_15
#define ESP_EN_GPIO_Port GPIOA
#define SIS_4_CT_BP_C_Pin GPIO_PIN_0
#define SIS_4_CT_BP_C_GPIO_Port GPIOD
#define SIS_4_FE_BP_C_Pin GPIO_PIN_1
#define SIS_4_FE_BP_C_GPIO_Port GPIOD
#define SIS_1_FE_BP_M_Pin GPIO_PIN_3
#define SIS_1_FE_BP_M_GPIO_Port GPIOD
#define SIS_1_CT_BP_M_Pin GPIO_PIN_4
#define SIS_1_CT_BP_M_GPIO_Port GPIOD
#define SIS_1_FE_BP_C_Pin GPIO_PIN_5
#define SIS_1_FE_BP_C_GPIO_Port GPIOD
#define SIS_2_FE_BP_C_Pin GPIO_PIN_6
#define SIS_2_FE_BP_C_GPIO_Port GPIOD
#define SIS_3_FE_BP_C_Pin GPIO_PIN_7
#define SIS_3_FE_BP_C_GPIO_Port GPIOD
#define SIS_5_FE_BP_M_Pin GPIO_PIN_9
#define SIS_5_FE_BP_M_GPIO_Port GPIOG
#define REG_4_M_Pin GPIO_PIN_10
#define REG_4_M_GPIO_Port GPIOG
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define ZONA_M_Pin GPIO_PIN_12
#define ZONA_M_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define CT_DES_2_Pin GPIO_PIN_14
#define CT_DES_2_GPIO_Port GPIOG
#define SIS_5_CT_BP_M_Pin GPIO_PIN_15
#define SIS_5_CT_BP_M_GPIO_Port GPIOG
#define REG_2_M_Pin GPIO_PIN_3
#define REG_2_M_GPIO_Port GPIOB
#define REG_3_M_Pin GPIO_PIN_5
#define REG_3_M_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define ESP_UART_HANDLE huart4
#define GPS_UART_HANDLE huart5
#define RS485_1_UART_HANDLE huart7
#define RS485_2_UART_HANDLE huart8
#define DEBUG_UART_HANDLE huart3
#define ADC_HANDLE hadc3
#define I2C_HANDLE hi2c1
#define SPI_HANDLE hspi4
#define SD_SPI_HANDLE hspi4
#define SD_CS_GPIO_Port GPIOE
#define SD_CS_Pin GPIO_PIN_11


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

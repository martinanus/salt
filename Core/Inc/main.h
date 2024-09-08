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
    typedef enum
    {
        MODO_NORMAL = 0,
        MODO_LIMITADO = 1,
        MODO_TOTAL = 2,
        MODO_PARADA = 3,
        MODO_COCHE_DERIVA = 4,
        MODO_INTERMITENTE = 5
    } salt_mode_t;

    typedef enum
    {
        SPEED_NONE = 0,
        HASLER = 1,
        PULSE_GENERATOR = 2,
        GPS = 3
    } speed_source_t;

    typedef enum
    {
        NO_ZONE = 0,
        ZONE_1 = 1,
        ZONE_2 = 2,
        ZONE_3 = 3
    } zones_t;

    typedef enum
    {
        STATUS_OK = 0,
        STATUS_ERROR = 1
    } status_t;

    typedef enum
    {
        RELAY_NORMAL = 0,
        RELAY_ENERGIZED = 1
    } relay_state_t;

    typedef enum
    {
        SIGNAL_OK = 0,
        SIGNAL_ERROR = 1
    } SIS_signal_state_t;

    typedef struct
    {
        SIS_signal_state_t FE_state;
        SIS_signal_state_t CT_state;
    } SIS_state_t;

    typedef enum
    {
        SIGNAL_UNINTERFERED = 0,
        SIGNAL_BYPASSED = 1,
        SIGNAL_OPEN = 2
    } critical_signal_state_t;

    typedef enum
    {
        SWITCH_ON = 0,
        SWITCH_OFF = 1
    } switch_state_t;

    typedef enum
    {
        COMMAND_INACTIVE = 0,
        COMMAND_ACTIVE = 1
    } command_states_t;

    typedef enum
    {
        //    RGB
        LED_ALL_OFF = 0, // 0b 000
        LED_B = 1,       // 0b 001
        LED_G = 2,       // 0b 010
        LED_BG = 3,      // 0b 011
        LED_R = 4,       // 0b 100
        LED_RB = 5,      // 0b 101
        LED_RG = 6,      // 0b 110
        LED_RGB = 7      // 0b 111
    } rgb_led_state_t;

    typedef enum
    {
        ZERO = 0,       // 0b 0000
        ONE = 1,        // 0b 0001
        TWO = 2,        // 0b 0010
        THREE = 3,      // 0b 0011
        FOUR = 4,       // 0b 0100
        FIVE = 5,       // 0b 0101
        SIX = 6,        // 0b 0110
        SEVEN = 7,      // 0b 0111
        EIGHT = 8,      // 0b 1000
        NINE = 9,       // 0b 1001
        DASH = 10,      // 0b 1001
        NONE_DIGIT = 11 // 0b1111
    } seven_segment_digit_t;

    typedef struct
    {
        seven_segment_digit_t digit;
        uint8_t decimal_point;
    } seven_segment_t;

    typedef enum
    {
        LED_OFF = 0,
        LED_ON = 1
    } led_state_t;

    typedef enum
    {
        BUZZER_OFF = 0,
        BUZZER_ON_INTERMITENT = 1,
        BUZZER_ON_CONTINUOS = 2
    } buzzer_state_t;

    typedef enum
    {
        CHOP_PROFILE_0 = 0,
        CHOP_PROFILE_1 = 1,
        CHOP_PROFILE_2 = 2,
        CHOP_PROFILE_3 = 3,
        CHOP_PROFILE_4 = 4
    } chop_profile_t;

    typedef enum
    {
        CHOP_DEACTIVATED = 0,
        CHOP_READY_TO_START = 1,
        CHOP_ACCELERATING = 2,
        CHOP_DECELERATING = 3,
        CHOP_BRAKING = 4

    } chop_state_t;
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
#define CHOP_SEL_EXTI_IRQn EXTI9_5_IRQn
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

#define LOCAL_SERIAL_UART_HANDLE huart3
#define WIFI_UART_HANDLE huart4
#define GPS_UART_HANDLE huart5
#define RS485_1_UART_HANDLE huart7
#define RS485_2_UART_HANDLE huart8
#define DEBUG_UART_HANDLE huart3
#define ADC_HANDLE hadc3
#define I2C_HANDLE hi2c1
#define SPI_HANDLE hspi4
#define RTC_HANDLE hrtc

#define SD_SPI_HANDLE hspi4
#define SD_CS_GPIO_Port GPIOE
#define SD_CS_Pin GPIO_PIN_11
#define ADC_HANDLE hadc3
#define SIS_FAIL_THRESHOLD 0x030 // 5v-3v3=0x057 3v3=0x078 5v=0x0C0 V in SIS input

#define HASLER_SPEED_BYTE 6

#define MAX_LOG_LENGTH 256
#define MAX_COMMAND_LENGTH 128
#define MAX_BUFFER_LENGTH 256
#define BTN_DEBOUNCE_MS 100

#define SPEED_STOP 0.5            // km/h
#define SPEED_READ_VALIDITY_S 3   // s
#define MODO_LIMITADO_STARTING_BREAK_S 5 //s TODO 30secs!
#define ZONE_READ_VALIDITY_S 60   // s
#define KNOT_TO_KM_H_FACTOR 1.852 // multiplier factor
#define BUZZER_SOUND_PERIOD_S 1   // s


#define DD_LAT_ORIGIN_POINT -34.587174 // Latitude for Retiro train station
#define DD_LON_ORIGIN_POINT -58.383134 // Longitude Retiro train station
#define DISTANCE_LIMIT_ZONE_1 10       // Distance in straight line from origin point
#define DISTANCE_LIMIT_ZONE_2 40       // Distance in straight line from origin point

#define COMMAND_VALIDITY_INITIAL 10           // s
#define REPORT_STATUS_PERIOD_S 5              // s
#define NO_ZONE_SPEED_CONFIG_INITIAL {29, 24, 35, 15} // speed_limit_to_decelerate, speed_limit_to_accelerate, speed_limit_to_brake, time_to_brake_s}
#define ZONE_1_SPEED_CONFIG_INITIAL {30, 25, 36, 10} 
#define ZONE_2_SPEED_CONFIG_INITIAL {35, 27, 40, 8} 
#define ZONE_3_SPEED_CONFIG_INITIAL {40, 30, 50, 6} 

#define CHOP_PROFILE_1_INITIAL {3, 7, 3, 10}  //  time_to_accelerate, time_to_decelerate, number_of_cycles_before_break, time_to_brake
#define CHOP_PROFILE_2_INITIAL {6, 14, 4, 20}
#define CHOP_PROFILE_3_INITIAL {9, 21, 5, 30}
#define CHOP_PROFILE_4_INITIAL {12, 28, 6, 40}
#define CHOP_PROFILE_5_INITIAL {15, 35, 7, 50}

#define SPEED_SOURCE_LABELS {"SPEED_NONE", "HASLER", "PULSE_GENERATOR", "GPS"}
#define SALT_MODE_LABELS { "MODO_NORMAL", "MODO_LIMITADO", "MODO_TOTAL", "MODO_PARADA","MODO_COCHE_DERIVA", "MODO_INTERMITENTE"}
#define CRITICAL_SIGNAL_STATE_LABELS {"SIGNAL_UNINTERFERED"  , "SIGNAL_BYPASSED","SIGNAL_OPEN"}

#define REG_MODO_LIMITADO_C_Pin REG_1_C_Pin
#define REG_MODO_LIMITADO_C_GPIO_Port REG_1_C_GPIO_Port
#define REG_POWER_OK_C_Pin REG_2_C_Pin
#define REG_POWER_OK_C_GPIO_Port REG_2_C_GPIO_Port
#define REG_FE_C_Pin REG_3_C_Pin
#define REG_FE_C_GPIO_Port REG_3_C_GPIO_Port
#define REG_CT_C_Pin REG_4_C_Pin
#define REG_CT_C_GPIO_Port REG_4_C_GPIO_Port

#define REG_MODO_LIMITADO_M_Pin REG_1_M_Pin
#define REG_MODO_LIMITADO_M_GPIO_Port REG_1_M_GPIO_Port
#define REG_POWER_OK_M_Pin REG_2_M_Pin
#define REG_POWER_OK_M_GPIO_Port REG_2_M_GPIO_Port
#define REG_FE_M_Pin REG_3_M_Pin
#define REG_FE_M_GPIO_Port REG_3_M_GPIO_Port
#define REG_CT_M_Pin REG_4_M_Pin
#define REG_CT_M_GPIO_Port REG_4_M_GPIO_Port

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

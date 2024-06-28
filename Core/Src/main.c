/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "gps.h"
#include "sd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void gpsUARTCallback(UART_HandleTypeDef *huart);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
char rxBuff[100];
char line[100];
uint8_t charRead;
uint8_t idx;
uint8_t new_line;
char sentence[] = {"$GPRMC"};
struct GPRMC gprms = {
	    .log_header = "",
	    .latitude = {0, 0, 0, '\0'},
	    .longitude = {0, 0, 0, '\0'},
	    .datetime = {0, 0, 0, 0, 0, 0},
	    .status = '\0',
	    .speed = 0,
	    .checksum = ""
	};
FATFS fs;
const char * local_log_file_name 	= "registro.txt";
const char * log_timestamp   		= "2024-05-15 12:34:56";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI4_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_UART7_Init(void);
static void MX_UART8_Init(void);
static void MX_USB_OTG_FS_USB_Init(void);
/* USER CODE BEGIN PFP */
void read_system_status(void);
void read_speed(void);
void read_hasler_speed(void);
void read_pulse_generator_speed(void);
void read_gps_speed(void);
void read_current_zone(void);
void read_gps_status(void);
void read_SIS_status(void);
void read_activation_switches_state(void);
void read_MAL_switch_state(void);
void read_MAT_switch_state(void);
void read_commands(void);
void read_remote_command(void);
void read_local_command(void);
void display_system_status(void);
void build_speed_display(void);
void build_system_status(void);
void send_system_status(void);
void activate_zone_relay(void);
void save_logs_local(void);
void transmit_events_remote(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Redirect printf to uart debug
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}


void gpsUARTCallback(UART_HandleTypeDef *huart)
{
	  if (charRead == '$' || idx == 100){
		  idx = 0;
		  rxBuff[idx++] = '$';
	  } else if (charRead == '\r' || charRead == '\n'){

		  if (strncmp((char *)rxBuff, sentence, strlen(sentence)) == 0){
			  rxBuff[idx++] = '\r';
			  rxBuff[idx++] = '\n';
			  rxBuff[idx++] = '\0';
			  memcpy(line, rxBuff, idx);
			  new_line = 1;
			  rxBuff[0] = 0;
		  } else {
			  idx = 0;
		  }
	  } else {
		  rxBuff[idx++] = charRead;
	  }


	  HAL_UART_Receive_IT(&GPS_UART_HANDLE, &charRead, 1);

}


/* -----------------------------------------------------------------*/
/* ------------START READING----------------------------------------*/
/* -----------------------------------------------------------------*/

void read_system_status(void){
    read_speed();
    read_current_zone();
    read_gps_status();
    read_SIS_status();
    read_activation_switches_state();	
    read_commands();	
}

typedef enum
{
  SPEED_NONE        = 0,      
  HASLER            = 1,      
  PULSE_GENERATOR   = 2,      
  GPS               = 3      
} speed_source_t;

float speed;
float hasler_speed;
float pulse_generator_speed;
float gps_speed;
speed_source_t speed_source;

void read_speed(void){
    read_hasler_speed();
    read_pulse_generator_speed();
    read_gps_speed();

    if (hasler_speed != -1){
        speed = hasler_speed;
        speed_source = HASLER;
    } else if (pulse_generator_speed != -1){
        speed = pulse_generator_speed;
        speed_source = PULSE_GENERATOR;
    } else if (gps_speed != -1){
        speed = gps_speed;
        speed_source = GPS;
    } else {
        speed = -1;
        speed_source = SPEED_NONE;
    }
}


void read_hasler_speed(void){
    hasler_speed = 14.1;
}

void read_pulse_generator_speed(void){
    pulse_generator_speed = 15.2;
}

void read_gps_speed(void){
	if (new_line){
		  printf(line);
		  parse_GPRMC((char*)line, &gprms);
		  if (gprms.status == 'A' ){
			  printf("GPS IS ACTIVE \r\n");
			  print_GPRMC(&gprms);
			  gps_speed  = gprms.speed;
		  }else{
			  printf("GPS IS NOT ACTIVE\r\n");
			  gps_speed = -1;
		  }
		  new_line = 0;
	}else {
		gps_speed = -1;
	}
}

typedef enum
{
    NO_ZONE     = 0,      
    ZONE_1      = 1,      
    ZONE_2      = 2,      
    ZONE_3      = 3   
} zones_t;
zones_t current_zone;

void read_current_zone(void){
	current_zone = ZONE_1;
}

typedef enum
{
	STATUS_OK     = 0,
	STATUS_ERROR  = 1
} status_t;
status_t gps_status;

void read_gps_status(void){
  if (gprms.status == 'A' ){
    gps_status = STATUS_OK;
  }  else {
    gps_status = STATUS_ERROR;
  }
	
}

typedef enum 
{
    RELAY_OPEN    = 0,
	RELAY_CLOSED  = 1
} relay_state_t;
relay_state_t SIS_status[5]; 

void read_SIS_status(void){
	SIS_status[0] = RELAY_CLOSED;
	SIS_status[1] = RELAY_CLOSED;
	SIS_status[2] = RELAY_OPEN;
	SIS_status[3] = RELAY_OPEN;
	SIS_status[4] = RELAY_CLOSED;

}


void read_activation_switches_state(void){
    read_MAL_switch_state();
	read_MAT_switch_state();
}

typedef enum 
{        
    SWITCH_OFF         = 0,
	SWITCH_ON          = 1
} switch_state_t;

switch_state_t MAL_switch_state;

void read_MAL_switch_state(void){
	MAL_switch_state = SWITCH_ON;
}

switch_state_t MAT_switch_state;

void read_MAT_switch_state(void){
	MAT_switch_state= SWITCH_ON;
}

#define MAX_COMMAND_LENGTH 256
typedef enum 
{
    COMMAND_INACTIVE    = 0,
	COMMAND_ACTIVE      = 1
} command_states_t;

void read_commands(void){
    read_remote_command(); 
	read_local_command(); 
}

char remote_command_buffer[MAX_COMMAND_LENGTH];
command_states_t remote_command_active;

void read_remote_command(void){
	remote_command_active = COMMAND_INACTIVE;
	sprintf(remote_command_buffer, "NO COMM");
}

char local_command_buffer[MAX_COMMAND_LENGTH];
command_states_t local_command_active;
void read_local_command(void){
	local_command_active = COMMAND_ACTIVE;
	sprintf(local_command_buffer, "myLocCom");
}


/* -----------------------------------------------------------------*/
/* ------------END READING------------------------------------------*/
/* -----------------------------------------------------------------*/

/* -----------------------------------------------------------------*/
/* ------------START DISPLAYING-------------------------------------*/
/* -----------------------------------------------------------------*/

void display_system_status(void){    
    build_system_status();
    send_system_status();
}

typedef enum 
{
                        //    RGB
    ALL_OFF     = 0,    // 0b 000
    B           = 1,    // 0b 001
    G           = 2,    // 0b 010
    BG          = 3,    // 0b 011
    R           = 4,    // 0b 100
    RB          = 5,    // 0b 101
    RG          = 6,    // 0b 110
    RGB         = 7     // 0b 111
} rgb_led_state_t;




typedef enum
{
    ZERO    = 0,    // 0b 0000
    ONE     = 1,    // 0b 0001
    TWO     = 2,    // 0b 0010
    THREE   = 3,    // 0b 0011
    FOUR    = 4,    // 0b 0100
    FIVE    = 5,    // 0b 0101
    SIX     = 6,    // 0b 0110
    SEVEN   = 7,    // 0b 0111
    EIGHT   = 8,    // 0b 1000
    NINE    = 9,    // 0b 1001
    DASH    = 10    // 0b 1001
} seven_segment_digit_t;

typedef struct 
{
  seven_segment_digit_t digit;
  uint8_t decimal_point;  
} seven_segment_t;


// RGB
rgb_led_state_t zone_led;

// only set RG
rgb_led_state_t SIS_leds[5];
rgb_led_state_t CT_led;
rgb_led_state_t FE_led;
rgb_led_state_t active_command_led;
rgb_led_state_t gps_led;

typedef enum {
	LED_OFF = 0,
	LED_ON = 1
}led_state_t;

typedef enum {
	BUZZER_OFF = 0,
	BUZZER_ON = 1
}buzzer_state_t;

// Green leds
led_state_t power_led;
led_state_t mode_MAL_led;
led_state_t mode_MAT_led;
led_state_t chop_profile_led;
buzzer_state_t buzzer_state;


seven_segment_t speed_display[4];

void build_system_status(void){
    build_speed_display();
}

void build_speed_display(void){
    int integer_part = (int) speed;
    int decimal_part = (int)((speed - integer_part) *10);
    if (speed_source == SPEED_NONE){
        for (int i=0; i<4;i++){
            speed_display[i].digit = DASH;
            speed_display[i].decimal_point = 0;
        }
    } else {
        speed_display[0].digit = (integer_part / 100) % 10;
        speed_display[0].decimal_point = 0;

        speed_display[1].digit = (integer_part / 10 ) % 10;
        speed_display[1].decimal_point = 0;

        speed_display[2].digit = integer_part % 10;
        speed_display[2].decimal_point = 1;

        speed_display[3].digit = decimal_part;
        speed_display[3].decimal_point = 0;
    }
}

void send_system_status(void){
	// I2C transmit all status
}


/* -----------------------------------------------------------------*/
/* ------------END DISPLAYING---------------------------------------*/
/* -----------------------------------------------------------------*/

/* -----------------------------------------------------------------*/
/* ------------START RELE ACTIVATIONS-------------------------------*/
/* -----------------------------------------------------------------*/
relay_state_t zone_relay; 
void activate_zone_relay(void){
    if (current_zone == ZONE_3){
    	zone_relay = RELAY_CLOSED;
    }
}

/* -----------------------------------------------------------------*/
/* ------------END RELE ACTIVATIONS---------------------------------*/
/* -----------------------------------------------------------------*/


/* -----------------------------------------------------------------*/
/* ------------START LOGGING----------------------------------------*/
/* -----------------------------------------------------------------*/
#define MAX_LOG_LENGTH 256
char local_log_buffer[MAX_LOG_LENGTH];
void save_logs_local(void){
	sprintf(local_log_buffer, "this is my local log");
	//log_event(local_log_file_name, log_timestamp, local_log_buffer);
	printf("%s: %s wrote in file: %s\r\n", log_timestamp, local_log_buffer, local_log_file_name);
}

char remote_events_buffer[MAX_LOG_LENGTH];
void transmit_events_remote(void){
	sprintf(remote_events_buffer, "This is my event");
}


/* -----------------------------------------------------------------*/
/* ------------END LOGGING------------------------------------------*/
/* -----------------------------------------------------------------*/

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
  MX_USART3_UART_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI4_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USB_OTG_FS_USB_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  mount_filesystem(&fs);
  //log_event(local_log_file_name, log_timestamp, "SD started OK");
  printf("%s: %s wrote in file: %s\r\n", log_timestamp, "SD started OK", local_log_file_name);


  HAL_UART_RegisterCallback(&GPS_UART_HANDLE, HAL_UART_RX_COMPLETE_CB_ID, gpsUARTCallback);
  HAL_UART_Receive_IT(&GPS_UART_HANDLE, &charRead, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	read_system_status();
	display_system_status();
	activate_zone_relay();
	transmit_events_remote();
	//save_logs_local();


	/*printf("Speed is: %f according to source %d \r\n", speed, speed_source);
	for (int i=0; i<4;i++){
		printf("Speed digit %d is: %d with decimal %d\r\n",i, speed_display[i].digit, speed_display[i].decimal_point);
	}*/

	HAL_Delay(1000);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SIS_2_CT_BP_C_Pin|SIS_3_CT_BP_C_Pin|RS485_2_DIR_Pin|REG_2_C_Pin
                          |ZONA_C_Pin|SPI4_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPS_PW_ON_Pin|RS485_1_DIR_Pin|BUZZER_C_Pin|ESP_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CT_C_GPIO_Port, CT_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, SIS_5_CT_BP_C_Pin|SIS_5_FE_BP_C_Pin|SIS_1_CT_BP_C_Pin|FE_DES_2_Pin
                          |FE_DES_1_Pin|CT_DES_1_Pin|CT_DES_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, FE_C_Pin|REG_1_C_Pin|REG_3_C_Pin|REG_4_C_Pin
                          |SIS_4_CT_BP_C_Pin|SIS_4_FE_BP_C_Pin|SIS_1_FE_BP_C_Pin|SIS_2_FE_BP_C_Pin
                          |SIS_3_FE_BP_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SIS_2_CT_BP_C_Pin SIS_3_CT_BP_C_Pin RS485_2_DIR_Pin REG_2_C_Pin
                           ZONA_C_Pin SPI4_CS_Pin */
  GPIO_InitStruct.Pin = SIS_2_CT_BP_C_Pin|SIS_3_CT_BP_C_Pin|RS485_2_DIR_Pin|REG_2_C_Pin
                          |ZONA_C_Pin|SPI4_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SIS_2_CT_BP_M_Pin SIS_3_FE_BP_M_Pin ON_SW_MAL_2_Pin FE_M_Pin
                           SD_CD_Pin */
  GPIO_InitStruct.Pin = SIS_2_CT_BP_M_Pin|SIS_3_FE_BP_M_Pin|ON_SW_MAL_2_Pin|FE_M_Pin
                          |SD_CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SIS_4_CT_BP_M_Pin SIS_4_FE_BP_M_Pin SIS_3_CT_BP_M_Pin CT_EN_1_M_Pin
                           CT_M_Pin ON_SW_MAL_1_Pin CT_EN_2_M_Pin */
  GPIO_InitStruct.Pin = SIS_4_CT_BP_M_Pin|SIS_4_FE_BP_M_Pin|SIS_3_CT_BP_M_Pin|CT_EN_1_M_Pin
                          |CT_M_Pin|ON_SW_MAL_1_Pin|CT_EN_2_M_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GPS_PW_ON_Pin RS485_1_DIR_Pin BUZZER_C_Pin ESP_EN_Pin */
  GPIO_InitStruct.Pin = GPS_PW_ON_Pin|RS485_1_DIR_Pin|BUZZER_C_Pin|ESP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CHOP_SEL_Pin */
  GPIO_InitStruct.Pin = CHOP_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHOP_SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CT_C_Pin */
  GPIO_InitStruct.Pin = CT_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CT_C_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SIS_5_CT_BP_C_Pin SIS_5_FE_BP_C_Pin SIS_1_CT_BP_C_Pin FE_DES_2_Pin
                           FE_DES_1_Pin CT_DES_1_Pin CT_DES_2_Pin */
  GPIO_InitStruct.Pin = SIS_5_CT_BP_C_Pin|SIS_5_FE_BP_C_Pin|SIS_1_CT_BP_C_Pin|FE_DES_2_Pin
                          |FE_DES_1_Pin|CT_DES_1_Pin|CT_DES_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ON_SW_MAT_2_Pin ON_SW_MAT_1_Pin REG_2_M_Pin REG_3_M_Pin */
  GPIO_InitStruct.Pin = ON_SW_MAT_2_Pin|ON_SW_MAT_1_Pin|REG_2_M_Pin|REG_3_M_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FE_C_Pin REG_1_C_Pin REG_3_C_Pin REG_4_C_Pin
                           SIS_4_CT_BP_C_Pin SIS_4_FE_BP_C_Pin SIS_1_FE_BP_C_Pin SIS_2_FE_BP_C_Pin
                           SIS_3_FE_BP_C_Pin */
  GPIO_InitStruct.Pin = FE_C_Pin|REG_1_C_Pin|REG_3_C_Pin|REG_4_C_Pin
                          |SIS_4_CT_BP_C_Pin|SIS_4_FE_BP_C_Pin|SIS_1_FE_BP_C_Pin|SIS_2_FE_BP_C_Pin
                          |SIS_3_FE_BP_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : REG_1_M_Pin FE_EN_2_M_Pin SIS_1_FE_BP_M_Pin SIS_1_CT_BP_M_Pin */
  GPIO_InitStruct.Pin = REG_1_M_Pin|FE_EN_2_M_Pin|SIS_1_FE_BP_M_Pin|SIS_1_CT_BP_M_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SIS_2_FE_BP_M_Pin FE_EN_1_M_Pin USB_OverCurrent_Pin SIS_5_FE_BP_M_Pin
                           REG_4_M_Pin ZONA_M_Pin SIS_5_CT_BP_M_Pin */
  GPIO_InitStruct.Pin = SIS_2_FE_BP_M_Pin|FE_EN_1_M_Pin|USB_OverCurrent_Pin|SIS_5_FE_BP_M_Pin
                          |REG_4_M_Pin|ZONA_M_Pin|SIS_5_CT_BP_M_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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

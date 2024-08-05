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
#include "led_driver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */



salt_mode_t salt_mode = MODO_NORMAL;


char WIFIrxBuff[100];
char WIFIline[100];
uint8_t WIFIcharRead;
uint8_t WIFIidx;
uint8_t WIFInew_line;


char GPSrxBuff[100];
char GPSline[100];
uint8_t GPScharRead;
uint8_t GPSidx;
uint8_t GPSnew_line;

char GPSsentence[] = {"$GPRMC"};
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

volatile uint16_t adc_results_dma[10];
volatile uint8_t ADC_ConvCplt = 0;
const int adcChannelCount = sizeof(adc_results_dma) / sizeof(adc_results_dma[0]);

float speed;
float hasler_speed;
float pulse_generator_speed;
float gps_speed;
speed_source_t speed_source;

zones_t current_zone;

status_t gps_status;

SIS_state_t SIS_state[5];

switch_state_t MAL_switch_state_1;
switch_state_t MAL_switch_state_2;
switch_state_t MAT_switch_state_1;
switch_state_t MAT_switch_state_2;

char remote_command_buffer[MAX_COMMAND_LENGTH];
command_states_t remote_command_active;

char local_command_buffer[MAX_COMMAND_LENGTH];
command_states_t local_command_active;

// RGB
rgb_led_state_t zone_led;

// only set RG
rgb_led_state_t SIS_leds[5];
rgb_led_state_t mode_MAL_led;
rgb_led_state_t mode_MAT_led;
rgb_led_state_t active_command_led;
rgb_led_state_t gps_led;

// Green leds
led_state_t power_led;
led_state_t CT_led;
led_state_t FE_led;
led_state_t chop_profile_leds[5];

buzzer_state_t buzzer_state = BUZZER_OFF;

uint8_t digit4_reg_value;
uint8_t digit5_reg_value;
uint8_t digit6_reg_value;
uint8_t digit7_reg_value;


uint8_t digit7Segment[4];
relay_state_t zone_relay; 

char local_log_buffer[MAX_LOG_LENGTH];
char remote_events_buffer[MAX_LOG_LENGTH];

uint32_t currentMillis;
uint32_t previousMillis;
chop_profile_t chop_profile = CHOP_PROFILE_0;

HAL_I2C_StateTypeDef i2c_state;
HAL_StatusTypeDef result;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
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
void GPS_UART_Callback(UART_HandleTypeDef *huart);
void WIFI_UART_Callback(UART_HandleTypeDef *huart);
void Led_Init(void);
void Handle_SaltMode_Transition(void);
void Read_SystemStatus(void);
void Read_Speed(void);
void Read_HaslerSpeed(void);
void Read_PulseGeneratorSpeed(void);
void Read_GPSSpeed(void);
void Read_CurrentZone(void);
void Read_GPSStatus(void);
void Read_SISStatus(void);
void Read_ActivationSwitchState(void);
void Read_MALSwitchState(void);
void Read_MATSwitchState(void);
void Read_RemoteCommand(void);
void Read_LocalCommand(void);
void Display_SystemStatus(void);
void Build_SystemStatus(void);
void Build_SpeedDisplay(void);
void Build_LedIndicators(void);
uint8_t extractBit(rgb_led_state_t state, uint8_t bitPos);
void setOrClearBit(uint8_t *byteVal, uint8_t bitPos, uint8_t value);
void setDigit4_regValue(void);
void setDigit5_regValue(void);
void setDigit6_regValue(void);
void setDigit7_regValue(void);
void Send_SystemStatus(void);
void Activate_ZoneRelay(void);
void Save_LocalLogs(void);
void Transmit_RemoteEvents(void);
void Activate_Buzzer(void);
void Deactivate_Buzzer(void);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Redirect printf to uart debug
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

void GPS_UART_Callback(UART_HandleTypeDef *huart)
{
	  if (GPScharRead == '$' || GPSidx == 100){
		  GPSidx = 0;
		  GPSrxBuff[GPSidx++] = '$';
	  } else if (GPScharRead == '\r' || GPScharRead == '\n'){

		  if (strncmp((char *)GPSrxBuff, GPSsentence, strlen(GPSsentence)) == 0){
			  GPSrxBuff[GPSidx++] = '\r';
			  GPSrxBuff[GPSidx++] = '\n';
			  GPSrxBuff[GPSidx++] = '\0';
			  memcpy(GPSline, GPSrxBuff, GPSidx);
			  GPSnew_line = 1;
			  GPSrxBuff[0] = 0;
		  } else {
			  GPSidx = 0;
		  }
	  } else {
		  GPSrxBuff[GPSidx++] = GPScharRead;
	  }


	  HAL_UART_Receive_IT(&GPS_UART_HANDLE, &GPScharRead, 1);

}

void WIFI_UART_Callback(UART_HandleTypeDef *huart)
{
	  if (WIFIcharRead == '\r' || WIFIcharRead == '\n'){
		  if (WIFIidx > 0){
			  memcpy(WIFIline, WIFIrxBuff, WIFIidx);
			  WIFIline[WIFIidx++] = '\r';
			  WIFIline[WIFIidx++] = '\n';
			  WIFIline[WIFIidx] = '\0';
			  WIFInew_line = 1;
			  WIFIrxBuff[0] = '\0';
			  WIFIidx = 0;
		  }
	  } else {
		  WIFIrxBuff[WIFIidx++] = WIFIcharRead;
	  }

	HAL_UART_Receive_IT(&WIFI_UART_HANDLE, &WIFIcharRead, 1);

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC3)
    {
    	ADC_ConvCplt = 1;
    	HAL_ADC_Start_DMA(&ADC_HANDLE, (uint32_t*) adc_results_dma, adcChannelCount);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
	if (salt_mode == MODO_LIMITADO){
		currentMillis = HAL_GetTick();
			if (GPIO_PIN == CHOP_SEL_Pin && currentMillis - previousMillis > BTN_DEBOUNCE_MS){
				if (chop_profile == CHOP_PROFILE_4) {
					chop_profile = CHOP_PROFILE_0;
				} else {
					chop_profile++;
				}
				printf("Chop profile changed to n %d! \n", chop_profile);
				previousMillis = currentMillis;
			}
	}
}


void Led_Init(){
  // LED RGB
  zone_led                = LED_G;
  SIS_leds[0]             = LED_G;
  SIS_leds[1]             = LED_G;
  SIS_leds[2]             = LED_G;
  SIS_leds[3]             = LED_G;
  SIS_leds[4]             = LED_G;
  active_command_led      = LED_ALL_OFF;
  gps_led                 = LED_G;  
  mode_MAL_led            = LED_ALL_OFF;
  mode_MAT_led            = LED_ALL_OFF;

  // Green LED
  power_led               = LED_ON;
  CT_led                  = LED_OFF;
  FE_led                  = LED_OFF;
  chop_profile_leds[0]    = LED_OFF;
  chop_profile_leds[1]    = LED_OFF;
  chop_profile_leds[2]    = LED_OFF;
  chop_profile_leds[3]    = LED_OFF;
  chop_profile_leds[4]    = LED_OFF;
}




void Read_SystemStatus(void){
    Read_Speed();
    Read_CurrentZone();
    Read_GPSStatus();
    Read_SISStatus();
    Read_LocalCommand();
}

void Read_Speed(void){
    Read_HaslerSpeed();
    Read_PulseGeneratorSpeed();
    Read_GPSSpeed();

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

void Read_HaslerSpeed(void){
    // TO BE IMPLEMENTED
    hasler_speed = 115.9;
}

void Read_PulseGeneratorSpeed(void){
    // TO BE IMPLEMENTED
    pulse_generator_speed = 95.2;
}

void Read_GPSSpeed(void){
	if (GPSnew_line){
		  printf(GPSline);
		  parse_GPRMC((char*)GPSline, &gprms);
		  if (gprms.status == 'A' ){
			  printf("GPS IS ACTIVE \r\n");
			  print_GPRMC(&gprms);
			  gps_speed  = gprms.speed;
		  }else{
			  printf("GPS IS NOT ACTIVE\r\n");
			  gps_speed = -1;
		  }
		  GPSnew_line = 0;
	}else {
		gps_speed = -1;
	}
}

void Read_CurrentZone(void){
	// TO BE IMPLEMENTED
	// using gprms.latitude  gprms.longitude to calculate distance from origin point
	current_zone = ZONE_1;
}

void Read_GPSStatus(void){  
  if (gprms.status == 'A' ){
    gps_status = STATUS_OK;
  }  else {
    gps_status = STATUS_ERROR;
  }
	
}

void Read_SISStatus(void){

	for (int i=0; i<5;i++){
		if (adc_results_dma[2*i] > SIS_FAIL_THRESHOLD){
			SIS_state[i].FE_state = RELAY_OPEN;
		} else {
			SIS_state[i].FE_state = RELAY_CLOSED;
		}
		if (adc_results_dma[2*i+1] > SIS_FAIL_THRESHOLD){
			SIS_state[i].CT_state = RELAY_OPEN;
		} else {
			SIS_state[i].CT_state = RELAY_CLOSED;
		}
	}
}

void Handle_SaltMode_Transition(void){

  Read_ActivationSwitchState();

  if (MAT_switch_state_1 && MAT_switch_state_2){
    // Here I can do all transtion actions
    if(salt_mode == MODO_NORMAL){
      // Do some exit actions
      // Do some entry actions
    } else if (salt_mode == MODO_LIMITADO){
      // Do some exit actions
      // Do some entry actions
    }
		salt_mode = MODO_TOTAL;
	} else if (MAL_switch_state_1 && MAL_switch_state_2){
    // Here I can do all transtion actions

    if(salt_mode == MODO_NORMAL){
      // Do some exit actions
      // Do some entry actions
    } else if (salt_mode == MODO_TOTAL){
      // Do some exit actions
      // Do some entry actions
    }
		salt_mode = MODO_LIMITADO;
	}else {
    // Here I can do all transtion actions
    if(salt_mode == MODO_LIMITADO){
      // Do some exit actions
      // Do some entry actions
    } else if (salt_mode == MODO_TOTAL){
      // Do some exit actions
      // Do some entry actions
    }
		salt_mode = MODO_NORMAL;  
	}
}

void Read_ActivationSwitchState(void){
  Read_MALSwitchState();
	Read_MATSwitchState();
}

void Read_MALSwitchState(void){

	MAL_switch_state_1 = HAL_GPIO_ReadPin(ON_SW_MAL_1_GPIO_Port, ON_SW_MAL_1_Pin );

	MAL_switch_state_2 = HAL_GPIO_ReadPin(ON_SW_MAL_2_GPIO_Port, ON_SW_MAL_2_Pin );
}

void Read_MATSwitchState(void){
	MAT_switch_state_1 = HAL_GPIO_ReadPin(ON_SW_MAT_1_GPIO_Port, ON_SW_MAT_1_Pin );

	MAT_switch_state_2 = HAL_GPIO_ReadPin(ON_SW_MAT_2_GPIO_Port, ON_SW_MAT_2_Pin );
}

void Read_RemoteCommand(void){
	if (WIFInew_line){
		  printf(WIFIline);
		  remote_command_active = COMMAND_ACTIVE;
		  sprintf(remote_command_buffer, WIFIline);
		  WIFInew_line = 0;
	}else {
		remote_command_active = COMMAND_INACTIVE;
		sprintf(remote_command_buffer, "NO COMM");
	}
}

void Read_LocalCommand(void){
	local_command_active = COMMAND_ACTIVE;
	sprintf(local_command_buffer, "myLocCom");
}

void Display_SystemStatus(void){    
    Build_SystemStatus();
    Send_SystemStatus();
}

void Build_SystemStatus(void){
    Build_SpeedDisplay();
    Build_LedIndicators();
}

void Build_SpeedDisplay(void){
	seven_segment_t speed_display[4];
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

    for (uint8_t digit=0; digit < 4 ; digit++){
        digit7Segment[digit]= digitTo7Segment(speed_display[digit].digit);

        if (speed_display[digit].decimal_point){
      	  digit7Segment[digit]|= DECIMAL_POINT_MASK;
        }

    }
}

void Build_LedIndicators(){
  if (salt_mode == MODO_NORMAL) {
      mode_MAL_led = LED_G;
      mode_MAT_led = LED_ALL_OFF;

      for (uint8_t i = 0; i < 5; i++){
        chop_profile_leds[i] = LED_OFF;
      }

      active_command_led = LED_ALL_OFF;

      if (gps_status == STATUS_OK){
        gps_led = LED_G;
      } else {
        gps_led = LED_R;
      }

      for (uint8_t i = 0; i < 5; i++){
        if (SIS_state[i].FE_state == RELAY_OPEN || SIS_state[i].CT_state == RELAY_OPEN){
          SIS_leds[i] = LED_R;  
        } else{
          SIS_leds[i] = LED_G;  
        }        
      }      
                     
  } else if (salt_mode == MODO_LIMITADO){
      mode_MAL_led = LED_G;
      mode_MAT_led = LED_ALL_OFF;


      for (uint8_t i = 0; i < 5; i++){
        if (chop_profile == i){
          chop_profile_leds[i] = LED_ON;  
        } else{
          chop_profile_leds[i] = LED_OFF;
        }        
      }      

      if (remote_command_active == COMMAND_ACTIVE){
        active_command_led = LED_G;
      } else {
        active_command_led = LED_ALL_OFF;
      }

      if (gps_status == STATUS_OK){
        gps_led = LED_G;
      } else {
        gps_led = LED_R;
      }

      // SIS_leds[i] remains unchanged from normal mode to show SIS failing

      // TO BE IMPLEMENTED
      CT_led = LED_RG; // GET STATE FROM ACTIVATION OR NOT
      FE_led = LED_RG; // GET_STATE FROM ACTIVATION OR NOT
            
  } else if(salt_mode == MODO_TOTAL){

      mode_MAL_led = LED_ALL_OFF;
      mode_MAT_led = LED_G;

      for (uint8_t i = 0; i < 5; i++){
        chop_profile_leds[i] = LED_OFF;
      }


      if (remote_command_active == COMMAND_ACTIVE){
        active_command_led = LED_G;
      } else {
        active_command_led = LED_ALL_OFF;
      }

      if (gps_status == STATUS_OK){
        gps_led = LED_G;
      } else {
        gps_led = LED_R;
      }

      // SIS_leds[i] remains unchanged from normal mode to show SIS failing
  }

   switch (current_zone){
      case ZONE_1:
        zone_led = LED_G;
        break;
      case ZONE_2:
        zone_led = LED_RG;
        break;
      case ZONE_3:
        zone_led = LED_R;
        break;
      default:
        zone_led = LED_ALL_OFF;
        break;
   }

  setDigit4_regValue();
  setDigit5_regValue();
  setDigit6_regValue();
  setDigit7_regValue();
}

uint8_t extractBit(rgb_led_state_t state, uint8_t bitPos) {
    return (state >> bitPos) & 0x01;
}

void setOrClearBit(uint8_t *byteVal, uint8_t bitPos, uint8_t value) {
    if (value) {
        *byteVal |= (1 << bitPos);  // Set the bit
    } else {
        *byteVal &= ~(1 << bitPos); // Clear the bit
    }
}

void setDigit4_regValue(){
    uint8_t byteVal = 0x00;
  
    uint8_t D7_DP = chop_profile_leds[0];               // LED14      - CC_4 - A_DP - CHOP_1
    uint8_t D6_A = extractBit(mode_MAL_led, 1);         // LED2_GREEN - CC_4 - A_A  - MODO LIMITADO
    uint8_t D5_B = extractBit(mode_MAT_led, 1);         // LED3_GREEN - CC_4 - A_B  - MODO TOTAL
    uint8_t D4_C = extractBit(active_command_led, 1);   // LED4_GREEN - CC_4 - A_C  - COMANDO REMOTO
    uint8_t D3_D = extractBit(gps_led, 1);              // LED5_GREEN - CC_4 - A_D  - GPS
    uint8_t D2_E = extractBit(zone_led, 1);             // LED6_GREEN - CC_4 - A_E  - ZONA
    uint8_t D1_F = extractBit(SIS_leds[0], 1);          // LED7_GREEN - CC_4 - A_F  - SIS_1
    uint8_t D0_G = chop_profile_leds[1];                // LED15      - CC_4 - A_G  - CHOP_2
    
    uint8_t vars[8] = {D0_G, D1_F, D2_E, D3_D, D4_C, D5_B, D6_A, D7_DP};

    
    for (uint8_t i = 0; i < 8; i++) {
        setOrClearBit(&byteVal, i, vars[i]);
    }

    digit4_reg_value = byteVal;
}

void setDigit5_regValue(){
    uint8_t byteVal = 0x00; 

    uint8_t D7_DP = power_led;                          //  LED1      - CC_5 - A_DP - POWER
    uint8_t D6_A = extractBit(mode_MAL_led, 2);         //  LED2_RED  - CC_5 - A_A  - MODO LIMITADO
    uint8_t D5_B = extractBit(mode_MAT_led, 2);         //  LED3_RED  - CC_5 - A_B  - MODO TOTAL
    uint8_t D4_C = extractBit(active_command_led, 2);   //  LED4_RED  - CC_5 - A_C  - COMANDO REMOTO
    uint8_t D3_D = extractBit(gps_led, 2);              //  LED5_RED  - CC_5 - A_D  - GPS
    uint8_t D2_E = extractBit(zone_led, 2);             //  LED6_RED  - CC_5 - A_E  - ZONA
    uint8_t D1_F = extractBit(SIS_leds[0], 2);          //  LED7_RED  - CC_5 - A_F  - SIS_1
    uint8_t D0_G = chop_profile_leds[2];                //  LED16     - CC_5 - A_G  - CHOP_3
    
    uint8_t vars[8] = {D0_G, D1_F, D2_E, D3_D, D4_C, D5_B, D6_A, D7_DP};

    for (uint8_t i = 0; i < 8; i++) {
        setOrClearBit(&byteVal, i, vars[i]);
    }

    digit5_reg_value = byteVal;
}

void setDigit6_regValue(){
    uint8_t byteVal = 0x00; 

    uint8_t D7_DP = CT_led;                             // LED12       - CC_6 - A_DP - CT
    uint8_t D6_A = extractBit(SIS_leds[1], 1);          // LED8_GREEN  - CC_6 - A_A  - SIS_2
    uint8_t D5_B = extractBit(SIS_leds[2], 1);          // LED9_GREEN  - CC_6 - A_B  - SIS_3
    uint8_t D4_C = extractBit(SIS_leds[3], 1);          // LED10_GREEN - CC_6 - A_C  - SIS_4
    uint8_t D3_D = extractBit(SIS_leds[4], 1);          // LED11_GREEN - CC_6 - A_D  - SIS_5
    uint8_t D2_E = 0;                                   // NC          - CC_6 - A_E  - NC
    uint8_t D1_F = extractBit(SIS_leds[0], 0);          // LED7_BLUE   - CC_6 - A_F  - SIS_1 --> ZONA
    uint8_t D0_G = chop_profile_leds[3];                // LED17       - CC_6 - A_G  - CHOP_4
    
    uint8_t vars[8] = {D0_G, D1_F, D2_E, D3_D, D4_C, D5_B, D6_A, D7_DP};

    for (uint8_t i = 0; i < 8; i++) {
        setOrClearBit(&byteVal, i, vars[i]);
    }

    digit6_reg_value = byteVal;
}

void setDigit7_regValue(){
    uint8_t byteVal = 0x00; 

    uint8_t D7_DP = FE_led;                             // LED13     - CC_7 - A_DP - FE
    uint8_t D6_A = extractBit(SIS_leds[1], 2);          // LED8_RED  - CC_7 - A_A  - SIS_2
    uint8_t D5_B = extractBit(SIS_leds[2], 2);          // LED9_RED  - CC_7 - A_B  - SIS_3
    uint8_t D4_C = extractBit(SIS_leds[3], 2);          // LED10_RED - CC_7 - A_C  - SIS_4
    uint8_t D3_D = extractBit(SIS_leds[4], 2);          // LED11_RED - CC_7 - A_D  - SIS_5
    uint8_t D2_E = 0;                                   // NC        - CC_7 - A_E  - NC
    uint8_t D1_F = 0;                                   // NC        - CC_7 - A_F  - NC
    uint8_t D0_G = chop_profile_leds[4];                // LED18     - CC_7 - A_G  - CHOP_5
    
    uint8_t vars[8] = {D0_G, D1_F, D2_E, D3_D, D4_C, D5_B, D6_A, D7_DP};

    for (uint8_t i = 0; i < 8; i++) {
        setOrClearBit(&byteVal, i, vars[i]);
    }

    digit7_reg_value = byteVal;
}


void Send_SystemStatus(void){

  for (uint8_t digit=0; digit < 4 ; digit++){
      result = LedDriver_WriteReg(digit + 1, digit7Segment[digit], &I2C_HANDLE);
  }
  result = LedDriver_WriteReg(0x05, digit4_reg_value, &I2C_HANDLE);
  result = LedDriver_WriteReg(0x06, digit5_reg_value, &I2C_HANDLE);
  result = LedDriver_WriteReg(0x07, digit6_reg_value, &I2C_HANDLE);
  result = LedDriver_WriteReg(0x08, digit7_reg_value, &I2C_HANDLE);
}


void Activate_ZoneRelay(void){
    if (current_zone == ZONE_3){
    	zone_relay = RELAY_CLOSED;
    }
}


void Save_LocalLogs(void){
  // TO BE IMPLEMENTED
  // What should I log here? 
	sprintf(local_log_buffer, "this is my local log");
	//log_event(local_log_file_name, log_timestamp, local_log_buffer);
	printf("%s: %s wrote in file: %s\r\n", log_timestamp, local_log_buffer, local_log_file_name);
}


void Transmit_RemoteEvents(void){
  // TO BE IMPLEMENTED
  // What should I send here? 
	sprintf(remote_events_buffer, "This is my event\n");
	HAL_UART_Transmit(&WIFI_UART_HANDLE, (uint8_t*)remote_events_buffer, strlen(remote_events_buffer), HAL_MAX_DELAY);
}

void Activate_Buzzer(void){
  HAL_GPIO_WritePin(BUZZER_C_GPIO_Port, BUZZER_C_Pin, BUZZER_ON);
}

void Deactivate_Buzzer(void){
  HAL_GPIO_WritePin(BUZZER_C_GPIO_Port, BUZZER_C_Pin, BUZZER_OFF);
}


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
  MX_DMA_Init();
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

  // Start GPS Callback
  HAL_UART_RegisterCallback(&GPS_UART_HANDLE, HAL_UART_RX_COMPLETE_CB_ID, GPS_UART_Callback);
  HAL_UART_Receive_IT(&GPS_UART_HANDLE, &GPScharRead, 1);

  // Start WIFI Callback
  HAL_UART_RegisterCallback(&WIFI_UART_HANDLE, HAL_UART_RX_COMPLETE_CB_ID, WIFI_UART_Callback);
  HAL_UART_Receive_IT(&WIFI_UART_HANDLE, &WIFIcharRead, 1);

  // Start first ADC conversion
  HAL_ADC_Start_DMA(&ADC_HANDLE, (uint32_t*) adc_results_dma, adcChannelCount);

    
  I2C_Reset(&hi2c1);
  i2c_state = HAL_I2C_GetState(&I2C_HANDLE);
  printf("i2c_state is %d", i2c_state);
  result = LedDriver_Init(&I2C_HANDLE);
  printf("Led driver initializated");

  Led_Init();

  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


    Handle_SaltMode_Transition();


	if (salt_mode == MODO_NORMAL){
    // TO BE IMPLEMENTED
		// Transaction btw mode should be configured
		// Deactivate_Buzzer();
    // Deactivate_SISBypass();
    // Deactivate_MQTTCommands();
    // Release_CriticalSignals();


		Read_SystemStatus();
		Display_SystemStatus();
		Activate_ZoneRelay();
		Transmit_RemoteEvents();
		Save_LocalLogs();
	} else if (salt_mode == MODO_LIMITADO){
		// TO BE IMPLEMENTED
		// Transaction btw mode should be configured
		// Activate_Buzzer();
    // Activate_SISBypass();
    
    Read_RemoteCommand(); 
    // Display_ActiveMQTTCommands();
    // Control_CriticalSignals();
    // Display_CriticalSignals();
    // Read_ChoppProfileSelector();
    // Display_ActiveChoppProfile();
    

		;
	} else if (salt_mode == MODO_TOTAL){
		// TO BE IMPLEMENTED
		// Transaction btw mode should be configured
		// Activate_Buzzer();
    // Activate_SISBypass();
    // Activate_MQTTCommands();
    // Release_CriticalSignals();

		;
	}

	HAL_Delay(1000);



	/*
	if (ADC_ConvCplt){
		for (int i=0; i<10;i++){
			printf("Read value %d: 0x%.3x\r\n", i, adc_results_dma[i]);
		}
		ADC_ConvCplt = 0;

	}

	printf("Speed is: %f according to source %d \r\n", speed, speed_source);
	for (int i=0; i<4;i++){
		printf("Speed digit %d is: %d with decimal %d\r\n",i, speed_display[i].digit, speed_display[i].decimal_point);
	}*/


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
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 10;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 10;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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

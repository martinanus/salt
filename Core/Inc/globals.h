
#include <stdio.h>
#include <stdlib.h>
#include "fatfs.h"
#include "stdio.h"
#include "string.h"
#include "gps.h"
#include "sd.h"
#include "led_driver.h"

extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc3;
extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart3;
extern salt_mode_t salt_mode;
extern salt_mode_t prev_salt_mode;
extern char *salt_mode_labels[];
extern status_t internal_error;
extern uint32_t modo_limitado_activationMillis;
extern uint8_t report_status_period_s;
extern uint8_t command_validity_s;
extern uint8_t speed_configs[4][4];
extern uint8_t chop_profile_config[5][4];
extern char WIFIrxBuff[MAX_BUFFER_LENGTH];
extern uint8_t WIFIcharRead;
extern uint8_t WIFIidx;
extern char GPSrxBuff[MAX_COMMAND_LENGTH];
extern uint8_t GPScharRead;
extern uint8_t GPSidx;
extern uint32_t gps_dataMillis;
extern char rs485_1_rxBuff[MAX_BUFFER_LENGTH];
extern uint8_t rs485_1_charRead;
extern uint8_t rs485_1_idx;
extern uint32_t rs485_1_dataMillis;
extern char rs485_2_rxBuff[MAX_BUFFER_LENGTH];
extern uint8_t rs485_2_charRead;
extern uint8_t rs485_2_idx;
extern uint32_t rs485_2_dataMillis;
extern char localSerial_rxBuff[MAX_BUFFER_LENGTH];
extern uint8_t localSerial_charRead;
extern uint8_t localSerial_idx;
extern char GPSsentence[];
extern struct GPRMC gprms;
extern dd_location_d origin_point;
extern FATFS fs;
extern const char *local_log_file_name ;
extern volatile uint16_t adc_results_dma[10];
extern volatile uint8_t ADC_ConvCplt;
extern const int adcChannelCount;
extern float speed;
extern float hasler_speed;
extern float pulse_generator_speed;
extern float gps_speed;
extern float prev_speed;
extern speed_source_t speed_source;
extern speed_source_t prev_speed_source;
extern char *speed_source_labels[];
extern zones_t current_zone;
extern zones_t prev_zone;
extern relay_state_t zone_relay;
extern relay_state_t prev_zone_relay;
extern status_t gps_status;
extern status_t prev_gps_status;
extern SIS_state_t SIS_state[5];
extern SIS_state_t prev_SIS_state[5];
extern critical_signal_state_t CT_signal;
extern critical_signal_state_t prev_CT_signal;
extern critical_signal_state_t FE_signal;
extern critical_signal_state_t prev_FE_signal;
extern char *critical_signal_state_labels[];
extern switch_state_t MAL_switch_state_1;
extern switch_state_t MAL_switch_state_2;
extern switch_state_t MAT_switch_state_1;
extern switch_state_t MAT_switch_state_2;
extern char remote_command[MAX_COMMAND_LENGTH];
extern command_states_t remote_command_active;
extern uint32_t remote_command_Millis;
extern rgb_led_state_t zone_led;
extern rgb_led_state_t SIS_leds[5];
extern rgb_led_state_t mode_MAL_led;
extern rgb_led_state_t mode_MAT_led;
extern rgb_led_state_t active_command_led;
extern rgb_led_state_t gps_led;
extern led_state_t power_led;
extern led_state_t CT_led;
extern led_state_t FE_led;
extern led_state_t chop_profile_leds[5];
extern buzzer_state_t buzzer_state;
extern uint8_t digit4_reg_value;
extern uint8_t digit5_reg_value;
extern uint8_t digit6_reg_value;
extern uint8_t digit7_reg_value;
extern uint8_t digit7Segment[4];
extern char local_log_buffer[MAX_LOG_LENGTH];
extern char remote_events_buffer[MAX_LOG_LENGTH];
extern chop_profile_t chop_profile;

void GPS_UART_Callback(UART_HandleTypeDef *huart);
void WIFI_UART_Callback(UART_HandleTypeDef *huart);
void Led_Init(void);
void Handle_SaltMode_Transition(void);
void Read_SystemStatus(void);
void Read_Speed(void);
void Process_RS485_1_line(void);
void Check_Hasler_Validity(void);
void Process_RS485_2_line(void);
void Check_PulseGenerator_Validity(void);
void Check_GPSData_Validity(void);
void Process_GPSline(void);
void Read_SISStatus(void);
void Read_ActivationSwitchState(void);
void Read_MALSwitchState(void);
void Read_MATSwitchState(void);
void Process_WIFIline(void);
void Check_RemoteCommand_Validity(void);
void Update_RTCDateTime(char *remote_command);
void Update_ReportStatusPeriod(char *remote_command);
void Update_CommandValidity(char *remote_command);
void Update_SpeedConfig(char *remote_command);
void Update_ChopConfig(char *remote_command);
void Select_ChopProfile(char *remote_command);
void Process_LocalSerialLine(void);
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
void Log_Event(const char *event);
void Send_CommandAcknoledge(int id);
void Transmit_RemoteEvents(const char *buffer);
void Control_Buzzer(void);
void Activate_SISBypass(void);
void Deactivate_SISBypass(void);
void Reset_GPS_Power(void);
void Test_ActivateRelays(void);
void Test_WriteCheckRelays(void);
void Set_CriticalSignals_State(void);
void Activate_ChopRoutine(void);
void Control_CriticalSignals(void);
void Control_CTsignal(void);
void Control_FEsignal(void);
void Report_SystemStatus(void);
void WriteCheck_RelayState(GPIO_TypeDef *WritecGPIOx, uint16_t Write_GPIO_Pin, GPIO_PinState PinState,
                           GPIO_TypeDef *Read_GPIOx, uint16_t Read_GPIO_Pin);

#include <stdio.h>
#include <stdlib.h>
#include "fatfs.h"
#include "string.h"
#include "gps.h"
#include "sd.h"
#include "led_driver.h"

extern osSemaphoreId commsSemHandle;

extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart3;

extern salt_mode_t salt_mode;
extern const char *salt_mode_labels[];
extern status_t internal_error;
extern uint32_t modo_limitado_activationMillis;
extern uint8_t report_status_period_s;
extern uint8_t command_validity_s;
extern uint8_t speed_configs[4][4];
extern uint8_t chop_profile_config[5][4];

extern comm_t commToProcess;
extern char WIFIline[MAX_BUFFER_LENGTH];
extern char GPSline[MAX_COMMAND_LENGTH];
extern char rs485_1_line[MAX_BUFFER_LENGTH];
extern char rs485_2_line[MAX_BUFFER_LENGTH];
extern char localSerial_line[MAX_BUFFER_LENGTH];
extern uint32_t gps_dataMillis;
extern uint32_t rs485_1_dataMillis;
extern uint32_t rs485_2_dataMillis;

extern const char *local_log_file_name ;
extern volatile uint16_t adc_results_dma[10];
extern float speed;
extern speed_source_t speed_source;
extern zones_t current_zone;
extern status_t gps_status;
extern SIS_state_t SIS_state[5];
extern critical_signal_state_t CT_signal;
extern critical_signal_state_t FE_signal;
extern switch_state_t MAL_switch_state_1;
extern switch_state_t MAL_switch_state_2;
extern switch_state_t MAT_switch_state_1;
extern switch_state_t MAT_switch_state_2;
extern char remote_command[MAX_COMMAND_LENGTH];
extern command_states_t remote_command_active;
extern buzzer_state_t buzzer_state;

extern char local_log_buffer[MAX_LOG_LENGTH];
extern char remote_events_buffer[MAX_LOG_LENGTH];
extern chop_profile_t chop_profile;

void Init_Communications(void);
void Process_WIFIline(void);
void Process_LocalSerialLine(void);
void Process_RS485_1_line(void);
void Process_RS485_2_line(void);
void Process_GPSline(void);
void Led_Init(void);
void Handle_SaltMode_Transition(void);
void Read_SystemStatus(void);
void Display_SystemStatus(void);

void Check_RemoteCommand_Validity(void);
void Log_Event(const char *event);
void Transmit_RemoteEvents(const char *buffer);
void Activate_SISBypass(void);
void Deactivate_SISBypass(void);
void Set_CriticalSignals_State(void);
void Control_CriticalSignals(void);
void Report_SystemStatus(void);
void WriteCheck_RelayState(GPIO_TypeDef *WritecGPIOx, uint16_t Write_GPIO_Pin, GPIO_PinState PinState,
                           GPIO_TypeDef *Read_GPIOx, uint16_t Read_GPIO_Pin);
#include "globals.h"

/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
void GPS_UART_Callback(UART_HandleTypeDef *huart);
void WIFI_UART_Callback(UART_HandleTypeDef *huart);
void Transmit_RemoteEvents(const char *buffer);
void Reset_GPS_Power(void);
void Log_Event(const char *event);
void Report_SystemStatus(void);
*/

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC3)
    {
        ADC_ConvCplt = 1;
        HAL_ADC_Start_DMA(&ADC_HANDLE, (uint32_t *)adc_results_dma, adcChannelCount);
    }
}

void GPS_UART_Callback(UART_HandleTypeDef *huart)
{
    if (GPScharRead == '$' || GPSidx == 100)
    {
        GPSidx = 0;
        GPSrxBuff[GPSidx++] = '$';
    }
    else if (GPScharRead == '\r' || GPScharRead == '\n')
    {

        if (strncmp((char *)GPSrxBuff, GPSsentence, strlen(GPSsentence)) == 0)
        {
            GPSrxBuff[GPSidx++] = '\r';
            GPSrxBuff[GPSidx++] = '\n';
            GPSrxBuff[GPSidx++] = '\0';
            memcpy(GPSline, GPSrxBuff, GPSidx);
            GPSnew_line = 1;
            GPSrxBuff[0] = 0;
            gps_dataMillis = HAL_GetTick();
            // printf("GPS: %s\r\n", GPSline);
        }
        else
        {
            GPSidx = 0;
        }
    }
    else
    {
        GPSrxBuff[GPSidx++] = GPScharRead;
    }

    HAL_UART_Receive_IT(&GPS_UART_HANDLE, &GPScharRead, 1);
}

void WIFI_UART_Callback(UART_HandleTypeDef *huart)
{
    if (WIFIcharRead == '\r' || WIFIcharRead == '\n')
    {
        if (WIFIidx > 0)
        {

            memcpy(WIFIline, WIFIrxBuff, WIFIidx);
            WIFIline[WIFIidx] = '\0';
            WIFInew_line = 1;
            WIFIrxBuff[0] = '\0';
            WIFIidx = 0;
            // printf("WIFI: %s\r\n", WIFIline);
        }
    }
    else
    {
        WIFIrxBuff[WIFIidx++] = WIFIcharRead;
    }

    HAL_UART_Receive_IT(&WIFI_UART_HANDLE, &WIFIcharRead, 1);
}

void clearBuffer(char *buffer, size_t size)
{
    memset(buffer, 0, size);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == RS485_1_UART_HANDLE.Instance)
    {
        rs485_1_rxBuff[rs485_1_idx++] = rs485_1_charRead;

        if (rs485_1_charRead == 0x7E)
        {
            if (rs485_1_idx == 31)
            {
                memcpy(rs485_1_line, rs485_1_rxBuff, rs485_1_idx);
                rs485_1_line[rs485_1_idx] = '\0';
                rs485_1_new_line = 1;
                rs485_1_rxBuff[0] = '\0';
                rs485_1_new_line = 1;
                rs485_1_dataMillis = HAL_GetTick();
                rs485_1_idx = 0;
                // printf("rs485_1: %s\r\n", rs485_1_line);
            }
            else if (rs485_1_idx > 31)
            {
                rs485_1_rxBuff[0] = '\0';
                rs485_1_idx = 0;
            }
        }

        rs485_1_charRead = 0;
        HAL_UART_Receive_IT(&RS485_1_UART_HANDLE, &rs485_1_charRead, 1);
    }
    else if (huart->Instance == RS485_2_UART_HANDLE.Instance)
    {
        if (rs485_2_charRead == '\r' || rs485_2_charRead == '\n')
        {
            if (rs485_2_idx > 0)
            {
                memcpy(rs485_2_line, rs485_2_rxBuff, rs485_2_idx);
                rs485_2_line[rs485_2_idx] = '\0';
                rs485_2_new_line = 1;
                rs485_2_rxBuff[0] = '\0';
                rs485_2_idx = 0;
                rs485_2_dataMillis = HAL_GetTick();
                // printf("rs485_2: %s\r\n", rs485_2_line);
            }
        }
        else
        {
            rs485_2_rxBuff[rs485_2_idx++] = rs485_2_charRead;
        }

        rs485_2_charRead = 0;
        HAL_UART_Receive_IT(&RS485_2_UART_HANDLE, &rs485_2_charRead, 1);
    }
    else if (huart->Instance == LOCAL_SERIAL_UART_HANDLE.Instance)
    {
        {
            if (localSerial_charRead == '\r' || localSerial_charRead == '\n')
            {
                if (localSerial_idx > 0)
                {
                    memcpy(localSerial_line, localSerial_rxBuff, localSerial_idx);
                    localSerial_line[localSerial_idx] = '\0';
                    localSerial_new_line = 1;

                    localSerial_rxBuff[0] = '\0';
                    localSerial_idx = 0;
                    // printf("localSerial: %s\r\n", localSerial_line);
                }
            }
            else
            {
                localSerial_rxBuff[localSerial_idx++] = localSerial_charRead;
            }

            localSerial_charRead = 0;
            HAL_UART_Receive_IT(&LOCAL_SERIAL_UART_HANDLE, &localSerial_charRead, 1);
        }
    }
}



void Transmit_RemoteEvents(const char *buffer)
{
    HAL_UART_Transmit(&WIFI_UART_HANDLE, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}


void Reset_GPS_Power(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPS_PW_ON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPS_PW_ON_GPIO_Port, &GPIO_InitStruct);

    // TO BE IMPLEMENTED - check if this can be async delay
    // Using timer or freeRTOS
    HAL_Delay(300);

    GPIO_InitStruct.Pin = GPS_PW_ON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPS_PW_ON_GPIO_Port, &GPIO_InitStruct);
}


void Log_Event(const char *event)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    char timestamp[20];
    char buffer[MAX_LOG_LENGTH];

    HAL_RTC_GetTime(&RTC_HANDLE, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&RTC_HANDLE, &sDate, RTC_FORMAT_BIN);

    sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d",
            sDate.Year + 2000,
            sDate.Month,
            sDate.Date,
            sTime.Hours,
            sTime.Minutes,
            sTime.Seconds);

    snprintf(buffer, sizeof(buffer), "%s: %s\r\n", timestamp, event);

    // TODO - uncomment this
    // write_in_file(local_log_file_name, buffer);
    Transmit_RemoteEvents(buffer);
    printf(buffer);
}


void Report_SystemStatus(void)
{
    static uint32_t last_reportSystemStatusMillis;
    char buffer[MAX_BUFF_SIZE];
    uint32_t currentMillis = HAL_GetTick();

    if (currentMillis - last_reportSystemStatusMillis > report_status_period_s * 1000)
    {

        if (salt_mode == MODO_LIMITADO)
        {
            snprintf(buffer, sizeof(buffer), "STATUS: MODO LIMITADO%s\r\n", speed_source ? "" : " - INTERMITENTE");
        }
        else
        {
            snprintf(buffer, sizeof(buffer), "STATUS: %s%s\r\n", salt_mode_labels[salt_mode], internal_error == STATUS_ERROR ? " - INTERNAL_ERROR" : "");
        }

        Transmit_RemoteEvents(buffer);
        printf("%s", buffer);

        last_reportSystemStatusMillis = currentMillis;
    }
}
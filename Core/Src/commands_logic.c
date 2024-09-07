#include "globals.h"

/*
void Read_LocalCommand(void);
void Read_RemoteCommand(void);
void Update_RTCDateTime(char *remote_command);
void Update_ReportStatusPeriod(char *remote_command);
void Update_CommandValidity(char *remote_command);
void Update_SpeedConfig(char *remote_command);
void Update_ChopConfig(char *remote_command);
void Select_ChopProfile(char *remote_command);
void Send_CommandAcknoledge(int id);
*/


void Read_LocalCommand(void)
{
    if (localSerial_new_line)
    {
        if (strcmp(localSerial_line, "DOWNLOAD_LOGS") == 0)
        {
            Log_Event("LOCAL_LOG_DOWNLOAD_START");
            read_file_line_by_line(local_log_file_name, &LOCAL_SERIAL_UART_HANDLE);
            Log_Event("LOCAL_LOG_DOWNLOAD_FINISH");
        }

        localSerial_new_line = 0;
    }
}

void Read_RemoteCommand(void)
{
    uint32_t currentMillis;
    int id;

    char *command_values;

    if (WIFInew_line)
    {

        sscanf(WIFIline, "%d|%[^:]", &id, remote_command);
        command_values = strchr(WIFIline, ':');

        if (id)
        {
            snprintf(local_log_buffer, sizeof(local_log_buffer), "COMMAND_RECEIVED: %s", remote_command);
            Log_Event(local_log_buffer);
            Send_CommandAcknoledge(id);
        }

        if (strcmp(remote_command, "AISLADO_TOTAL") == 0)
        {
            remote_command_active = COMMAND_ACTIVE;
            remote_command_Millis = HAL_GetTick();
        }
        else if (strcmp(remote_command, "PARADA_TOTAL") == 0)
        {
            remote_command_active = COMMAND_ACTIVE;
            remote_command_Millis = HAL_GetTick();
        }
        else if (strcmp(remote_command, "COCHE_DERIVA") == 0)
        {
            remote_command_active = COMMAND_ACTIVE;
            remote_command_Millis = HAL_GetTick();
        }
        else if (strcmp(remote_command, "INTERMITENTE") == 0)
        {
            Select_ChopProfile(command_values + 1);
            remote_command_active = COMMAND_ACTIVE;
            remote_command_Millis = HAL_GetTick();
        }
        else if (strcmp(remote_command, "CANCEL") == 0)
        {
            remote_command_active = COMMAND_INACTIVE;
            Log_Event("REMOTE_COMMANDS_CANCELED");
        }
        else if (strcmp(remote_command, "DATETIME") == 0)
        {
            Update_RTCDateTime(command_values + 1);
        }
        else if (strcmp(remote_command, "REPORT_STATUS_PERIOD_CONFIG") == 0)
        {
            Update_ReportStatusPeriod(command_values + 1);
        }
        else if (strcmp(remote_command, "COMMAND_VALIDITY_CONFIG") == 0)
        {
            Update_CommandValidity(command_values + 1);
        }
        else if (strcmp(remote_command, "SPEED_CONFIG") == 0)
        {
            Update_SpeedConfig(command_values + 1);
        }
        else if (strcmp(remote_command, "INTERMITENTE_CONFIG") == 0)
        {
            Update_ChopConfig(command_values + 1);
        }
        else if (strcmp(remote_command, "DOWNLOAD_LOGS") == 0)
        {
            Log_Event("REMOTE_LOG_DOWNLOAD_START");
            read_file_line_by_line(local_log_file_name, &WIFI_UART_HANDLE);
            Log_Event("REMOTE_LOG_DOWNLOAD_FINISH");
        }

        WIFInew_line = 0;
    }
    else
    {
        currentMillis = HAL_GetTick();
        if (currentMillis - remote_command_Millis > command_validity_s * 1000)
        {
            if (remote_command_active == COMMAND_ACTIVE)
            {
                remote_command_active = COMMAND_INACTIVE;
                Log_Event("REMOTE_COMMAND_EXPIRED");
            }
        }
    }
}

void Update_RTCDateTime(char *remote_command)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    int day, month, year, hours, minutes, seconds;

    sscanf(remote_command, "%02d/%02d/%04d %02d:%02d:%02d",
           &day, &month, &year, &hours, &minutes, &seconds);

    sTime.Hours = hours;
    sTime.Minutes = minutes;
    sTime.Seconds = seconds;

    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    sDate.Month = month;
    sDate.Date = day;
    sDate.Year = year - 2000;

    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
}

void Update_ReportStatusPeriod(char *remote_command)
{
    int new_reportStatusPeriod;

    sscanf(remote_command, "%d", &new_reportStatusPeriod);

    report_status_period_s = new_reportStatusPeriod;
    sprintf(local_log_buffer, "REPORT_STATUS_PERIOD: %d", report_status_period_s);
    Log_Event(local_log_buffer);
}

void Update_CommandValidity(char *remote_command)
{
    int new_command_validity;

    sscanf(remote_command, "%d", &new_command_validity);

    command_validity_s = new_command_validity;
    sprintf(local_log_buffer, "COMMAND_VALIDITY: %d", command_validity_s);
    Log_Event(local_log_buffer);
}

void Update_SpeedConfig(char *remote_command)
{
    int zone;
    int speeds[4];

    if (sscanf(remote_command, "%d,%d,%d,%d,%d", &zone, &speeds[0], &speeds[1], &speeds[2], &speeds[3]) == 5)
    {
        memcpy(speed_configs, speeds, sizeof(speed_configs));
        speed_configs[zone][0] = speeds[0];
        speed_configs[zone][1] = speeds[1];
        speed_configs[zone][2] = speeds[2];
        speed_configs[zone][3] = speeds[3];
        sprintf(local_log_buffer, "SPEED_CONFIG_ZONE_%u: %u,%u,%u,%u", zone, speeds[0], speeds[1], speeds[2], speeds[3]);
        Log_Event(local_log_buffer);
    }
}

void Update_ChopConfig(char *remote_command)
{
    int profile;
    int chop_parameters[4];

    if (sscanf(remote_command, "%d,%d,%d,%d,%d", &profile, &chop_parameters[0], &chop_parameters[1], &chop_parameters[2], &chop_parameters[3]) == 5)
    {
        profile -= 1; // offset by starting array in idx 0
        chop_profile_config[profile][0] = chop_parameters[0];
        chop_profile_config[profile][1] = chop_parameters[1];
        chop_profile_config[profile][2] = chop_parameters[2];
        chop_profile_config[profile][3] = chop_parameters[3];
        sprintf(local_log_buffer, "CHOP_CONFIG_PROF_%u: %u,%u,%u,%u", profile, chop_parameters[0], chop_parameters[1], chop_parameters[2], chop_parameters[3]);
        Log_Event(local_log_buffer);
    }
}

void Select_ChopProfile(char *remote_command)
{
    int profile;

    if (sscanf(remote_command, "%d", &profile) == 1)
    {
        if (profile >= 1 && profile <= 5)
        {
            chop_profile = profile - 1;
        }
    }
}



void Send_CommandAcknoledge(int id)
{
    char buffer[MAX_BUFF_SIZE];
    snprintf(buffer, sizeof(buffer), "ACK: %d\r\n", id);
    Transmit_RemoteEvents(buffer);
    printf(buffer);
}

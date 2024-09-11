#include "globals.h"

/*
void Read_SystemStatus(void);
void Read_Speed(void);
void Process_RS485_1_line(void)
void Check_Hasler_Validity(void)
void Process_RS485_2_line(void);
void Check_PulseGenerator_Validity(void);
void Check_GPSData_Validity(void);
void Process_GPSline(void);
void Read_SISStatus(void);
void Read_ActivationSwitchState(void);
void Read_MALSwitchState(void);
void Read_MATSwitchState(void);
*/

void Read_SystemStatus(void)
{
    Check_GPSData_Validity();
    Read_Speed();
    Read_SISStatus();
}

void Read_Speed(void)
{
    Check_Hasler_Validity();
    Check_PulseGenerator_Validity();

    prev_speed_source = speed_source;
    prev_speed = speed;

    if (hasler_speed != -1)
    {
        speed = hasler_speed;
        speed_source = HASLER;
    }
    else if (pulse_generator_speed != -1)
    {
        speed = pulse_generator_speed;
        speed_source = PULSE_GENERATOR;
    }
    else if (gps_speed != -1)
    {
        speed = gps_speed;
        speed_source = GPS;
    }
    else
    {
        speed = -1;
        speed_source = SPEED_NONE;
    }

    if (speed_source != prev_speed_source)
    {
        sprintf(local_log_buffer, "SPEED_SOURCE: %s", speed_source_labels[speed_source]);
        Log_Event(local_log_buffer);
    }

    if (speed != prev_speed)
    {
        if (speed_source != SPEED_NONE)
        {
            sprintf(local_log_buffer, "SPEED: %.2f", speed);
            Log_Event(local_log_buffer);
        }
    }
}
void Process_RS485_1_line(void)
{
    hasler_speed = rs485_1_rxBuff[HASLER_SPEED_BYTE];
}

void Check_Hasler_Validity(void)
{
    uint32_t currentMillis;

    currentMillis = HAL_GetTick();
    if (currentMillis - rs485_1_dataMillis > SPEED_READ_VALIDITY_S * 1000)
    {
        hasler_speed = -1;
    }
}


void Process_RS485_2_line(void)
{
    char *pEnd;

    pulse_generator_speed = strtof(rs485_2_rxBuff, &pEnd);

    if (pEnd == rs485_2_rxBuff)
    {
        pulse_generator_speed = -1;
    }
}

void Check_PulseGenerator_Validity(void)
{
    uint32_t currentMillis;

    currentMillis = HAL_GetTick();
    if (currentMillis - rs485_2_dataMillis > SPEED_READ_VALIDITY_S * 1000)
    {
        pulse_generator_speed = -1;
    }
}

void Process_GPSline(void)
{
    double distance_from_origin;
    dd_location_d current_point;

    prev_gps_status = gps_status;
    prev_zone = current_zone;

    parse_GPRMC((char *)GPSrxBuff, &gprms);
    if (gprms.status == 'A')
    {
        gps_status = STATUS_OK;
        gps_speed = gprms.speed * KNOT_TO_KM_H_FACTOR;

        current_point = convert_dms_to_decimal(gprms.latitude, gprms.longitude);
        distance_from_origin = haversine_distance(current_point, origin_point);

        if (distance_from_origin > DISTANCE_LIMIT_ZONE_2)
        {
            current_zone = ZONE_3;
        }
        else if (distance_from_origin > DISTANCE_LIMIT_ZONE_1)
        {
            current_zone = ZONE_2;
        }
        else
        {
            current_zone = ZONE_1;
        }
    }
    else
    {
        gps_status = STATUS_ERROR;
        gps_speed = -1;
        current_zone = NO_ZONE;
    }

    if (gps_status != prev_gps_status)
    {
        sprintf(local_log_buffer, "GPS_STATUS: %s ", gps_status ? "NOT_CONNECTED" : "OK");
        Log_Event(local_log_buffer);
    }

    if (current_zone != prev_zone)
    {
        sprintf(local_log_buffer, "ZONE: %d", current_zone);
        Log_Event(local_log_buffer);
    }
}

void Check_GPSData_Validity(void)
{
    uint32_t currentMillis;

    currentMillis = HAL_GetTick();
    if (currentMillis - gps_dataMillis > SPEED_READ_VALIDITY_S * 1000)
    {
        gps_status = STATUS_ERROR;
        gps_speed = -1;
        current_zone = NO_ZONE;
    }
}

void Read_SISStatus(void)
{
    for (int i = 0; i < 5; i++)
    {

        prev_SIS_state[i].FE_state = SIS_state[i].FE_state;
        prev_SIS_state[i].CT_state = SIS_state[i].CT_state;

        if (adc_results_dma[2 * i] > SIS_FAIL_THRESHOLD)
        {
            SIS_state[i].FE_state = SIGNAL_ERROR;
        }
        else
        {
            SIS_state[i].FE_state = SIGNAL_OK;
        }

        if (adc_results_dma[2 * i + 1] > SIS_FAIL_THRESHOLD)
        {
            SIS_state[i].CT_state = SIGNAL_ERROR;
        }
        else
        {
            SIS_state[i].CT_state = SIGNAL_OK;
        }
    }

    for (int i = 0; i < 5; i++)
    {
        if (SIS_state[i].FE_state != prev_SIS_state[i].FE_state)
        {
            sprintf(local_log_buffer, "SIS_%d_FE_state: %s ", i + 1, SIS_state[i].FE_state ? "FAIL" : "OK");
            Log_Event(local_log_buffer);
        }
        if (SIS_state[i].CT_state != prev_SIS_state[i].CT_state)
        {
            sprintf(local_log_buffer, "SIS_%d_CT_state: %s ", i + 1, SIS_state[i].CT_state ? "FAIL" : "OK");
            Log_Event(local_log_buffer);
        }
    }
}

void Read_ActivationSwitchState(void)
{
    Read_MALSwitchState();
    Read_MATSwitchState();
}

void Read_MALSwitchState(void)
{
    MAL_switch_state_1 = HAL_GPIO_ReadPin(ON_SW_MAL_1_GPIO_Port, ON_SW_MAL_1_Pin);

    MAL_switch_state_2 = HAL_GPIO_ReadPin(ON_SW_MAL_2_GPIO_Port, ON_SW_MAL_2_Pin);
}

void Read_MATSwitchState(void)
{
    MAT_switch_state_1 = HAL_GPIO_ReadPin(ON_SW_MAT_1_GPIO_Port, ON_SW_MAT_1_Pin);

    MAT_switch_state_2 = HAL_GPIO_ReadPin(ON_SW_MAT_2_GPIO_Port, ON_SW_MAT_2_Pin);
}

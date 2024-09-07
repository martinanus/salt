#include "globals.h"

/*
void Activate_SISBypass(void);
void Deactivate_SISBypass(void);
void Set_CriticalSignals_State(void);
void Activate_ChopRoutine(void);
void Control_CriticalSignals(void);
void Control_CTsignal(void);
void Control_FEsignal(void);
*/


void Activate_SISBypass(void)
{
    if (SIS_state[0].CT_state == SIGNAL_ERROR)
    {
        WriteCheck_RelayState(SIS_1_CT_BP_C_GPIO_Port, SIS_1_CT_BP_C_Pin, RELAY_ENERGIZED,
                              SIS_1_CT_BP_M_GPIO_Port, SIS_1_CT_BP_M_Pin);
        Log_Event("BYPASS_ACTIVE: SIS_1_CT");
    }
    if (SIS_state[0].FE_state == SIGNAL_ERROR)
    {
        WriteCheck_RelayState(SIS_1_FE_BP_C_GPIO_Port, SIS_1_FE_BP_C_Pin, RELAY_ENERGIZED,
                              SIS_1_FE_BP_M_GPIO_Port, SIS_1_FE_BP_M_Pin);
        Log_Event("BYPASS_ACTIVE: SIS_1_FE");
    }

    if (SIS_state[1].CT_state == SIGNAL_ERROR)
    {
        WriteCheck_RelayState(SIS_2_CT_BP_C_GPIO_Port, SIS_2_CT_BP_C_Pin, RELAY_ENERGIZED,
                              SIS_2_CT_BP_M_GPIO_Port, SIS_2_CT_BP_M_Pin);
        Log_Event("BYPASS_ACTIVE: SIS_2_CT");
    }
    if (SIS_state[1].FE_state == SIGNAL_ERROR)
    {
        WriteCheck_RelayState(SIS_2_FE_BP_C_GPIO_Port, SIS_2_FE_BP_C_Pin, RELAY_ENERGIZED,
                              SIS_2_FE_BP_M_GPIO_Port, SIS_2_FE_BP_M_Pin);
        Log_Event("BYPASS_ACTIVE: SIS_2_FE");
    }

    if (SIS_state[2].CT_state == SIGNAL_ERROR)
    {
        WriteCheck_RelayState(SIS_3_CT_BP_C_GPIO_Port, SIS_3_CT_BP_C_Pin, RELAY_ENERGIZED,
                              SIS_3_CT_BP_M_GPIO_Port, SIS_3_CT_BP_M_Pin);
        Log_Event("BYPASS_ACTIVE: SIS_3_CT");
    }
    if (SIS_state[2].FE_state == SIGNAL_ERROR)
    {
        WriteCheck_RelayState(SIS_3_FE_BP_C_GPIO_Port, SIS_3_FE_BP_C_Pin, RELAY_ENERGIZED,
                              SIS_3_FE_BP_M_GPIO_Port, SIS_3_FE_BP_M_Pin);
        Log_Event("BYPASS_ACTIVE: SIS_3_FE");
    }

    if (SIS_state[3].CT_state == SIGNAL_ERROR)
    {
        WriteCheck_RelayState(SIS_4_CT_BP_C_GPIO_Port, SIS_4_CT_BP_C_Pin, RELAY_ENERGIZED,
                              SIS_4_CT_BP_M_GPIO_Port, SIS_4_CT_BP_M_Pin);
        Log_Event("BYPASS_ACTIVE: SIS_4_CT");
    }
    if (SIS_state[3].FE_state == SIGNAL_ERROR)
    {
        WriteCheck_RelayState(SIS_4_FE_BP_C_GPIO_Port, SIS_4_FE_BP_C_Pin, RELAY_ENERGIZED,
                              SIS_4_FE_BP_M_GPIO_Port, SIS_4_FE_BP_M_Pin);
        Log_Event("BYPASS_ACTIVE: SIS_4_FE");
    }

    if (SIS_state[4].CT_state == SIGNAL_ERROR)
    {
        WriteCheck_RelayState(SIS_5_CT_BP_C_GPIO_Port, SIS_5_CT_BP_C_Pin, RELAY_ENERGIZED,
                              SIS_5_CT_BP_M_GPIO_Port, SIS_5_CT_BP_M_Pin);
        Log_Event("BYPASS_ACTIVE: SIS_5_CT");
    }
    if (SIS_state[4].FE_state == SIGNAL_ERROR)
    {
        WriteCheck_RelayState(SIS_5_FE_BP_C_GPIO_Port, SIS_5_FE_BP_C_Pin, RELAY_ENERGIZED,
                              SIS_5_FE_BP_M_GPIO_Port, SIS_5_FE_BP_M_Pin);
        Log_Event("BYPASS_ACTIVE: SIS_5_FE");
    }
}

void Deactivate_SISBypass(void)
{
    WriteCheck_RelayState(SIS_1_CT_BP_C_GPIO_Port, SIS_1_CT_BP_C_Pin, RELAY_NORMAL,
                          SIS_1_CT_BP_M_GPIO_Port, SIS_1_CT_BP_M_Pin);
    WriteCheck_RelayState(SIS_1_FE_BP_C_GPIO_Port, SIS_1_FE_BP_C_Pin, RELAY_NORMAL,
                          SIS_1_FE_BP_M_GPIO_Port, SIS_1_FE_BP_M_Pin);
    WriteCheck_RelayState(SIS_2_CT_BP_C_GPIO_Port, SIS_2_CT_BP_C_Pin, RELAY_NORMAL,
                          SIS_2_CT_BP_M_GPIO_Port, SIS_2_CT_BP_M_Pin);
    WriteCheck_RelayState(SIS_2_FE_BP_C_GPIO_Port, SIS_2_FE_BP_C_Pin, RELAY_NORMAL,
                          SIS_2_FE_BP_M_GPIO_Port, SIS_2_FE_BP_M_Pin);
    WriteCheck_RelayState(SIS_3_CT_BP_C_GPIO_Port, SIS_3_CT_BP_C_Pin, RELAY_NORMAL,
                          SIS_3_CT_BP_M_GPIO_Port, SIS_3_CT_BP_M_Pin);
    WriteCheck_RelayState(SIS_3_FE_BP_C_GPIO_Port, SIS_3_FE_BP_C_Pin, RELAY_NORMAL,
                          SIS_3_FE_BP_M_GPIO_Port, SIS_3_FE_BP_M_Pin);
    WriteCheck_RelayState(SIS_4_CT_BP_C_GPIO_Port, SIS_4_CT_BP_C_Pin, RELAY_NORMAL,
                          SIS_4_CT_BP_M_GPIO_Port, SIS_4_CT_BP_M_Pin);
    WriteCheck_RelayState(SIS_4_FE_BP_C_GPIO_Port, SIS_4_FE_BP_C_Pin, RELAY_NORMAL,
                          SIS_4_FE_BP_M_GPIO_Port, SIS_4_FE_BP_M_Pin);
    WriteCheck_RelayState(SIS_5_CT_BP_C_GPIO_Port, SIS_5_CT_BP_C_Pin, RELAY_NORMAL,
                          SIS_5_CT_BP_M_GPIO_Port, SIS_5_CT_BP_M_Pin);
    WriteCheck_RelayState(SIS_5_FE_BP_C_GPIO_Port, SIS_5_FE_BP_C_Pin, RELAY_NORMAL,
                          SIS_5_FE_BP_M_GPIO_Port, SIS_5_FE_BP_M_Pin);
    Log_Event("BYPASS_RELEASED: ALL_SIS_CT_FE");
}

void Set_CriticalSignals_State(void)
{
    static uint32_t last_FE_signal_activation_ms = 0;
    uint32_t currentMillis;

    uint8_t speed_config_selected[4];

    memcpy(speed_config_selected, speed_configs[current_zone], sizeof(speed_config_selected));

    uint8_t speed_limit_to_decelerate = speed_config_selected[0]; // km/h
    uint8_t speed_limit_to_accelerate = speed_config_selected[1]; // km/h
    uint8_t speed_limit_to_brake = speed_config_selected[2];      // km/h
    uint8_t time_to_brake_s = speed_config_selected[3];           // s

    prev_CT_signal = CT_signal;
    prev_FE_signal = FE_signal;

    if (salt_mode == MODO_NORMAL)
    {
        CT_signal = SIGNAL_UNINTERFERED;
        FE_signal = SIGNAL_UNINTERFERED;
    }
    else if (salt_mode == MODO_LIMITADO)
    {
        currentMillis = HAL_GetTick();

        if (speed_source != SPEED_NONE)
        {

            if (speed > speed_limit_to_decelerate)
            {
                CT_signal = SIGNAL_OPEN;
                buzzer_state = BUZZER_ON_CONTINUOS;
            }
            else if (CT_signal == SIGNAL_OPEN &&
                     FE_signal == SIGNAL_UNINTERFERED &&
                     speed < speed_limit_to_accelerate)
            {

                CT_signal = SIGNAL_UNINTERFERED;
                buzzer_state = BUZZER_ON_INTERMITENT;
            }

            if (FE_signal == SIGNAL_UNINTERFERED && speed > speed_limit_to_brake)
            {

                FE_signal = SIGNAL_OPEN;
                last_FE_signal_activation_ms = HAL_GetTick();
            }
            else if (currentMillis - last_FE_signal_activation_ms > time_to_brake_s * 1000 &&
                     speed < SPEED_STOP)
            {
                FE_signal = SIGNAL_UNINTERFERED;
            }
        }
        else
        {
            if (currentMillis - modo_limitado_activationMillis < MODO_LIMITADO_STARTING_BREAK_S * 1000)
            {
                CT_signal = SIGNAL_OPEN;
                FE_signal = SIGNAL_OPEN;
            }
            else
            {
                Activate_ChopRoutine();
            }
        }
    }
    else if (salt_mode == MODO_TOTAL)
    {
        CT_signal = SIGNAL_BYPASSED;
        FE_signal = SIGNAL_BYPASSED;
    }
    else if (salt_mode == MODO_PARADA)
    {
        CT_signal = SIGNAL_OPEN;
        FE_signal = SIGNAL_OPEN;
    }
    else if (salt_mode == MODO_COCHE_DERIVA)
    {
        CT_signal = SIGNAL_OPEN;
        FE_signal = SIGNAL_BYPASSED;
    }
    else if (salt_mode == MODO_INTERMITENTE)
    {
        Activate_ChopRoutine();
    }
}

void Activate_ChopRoutine(void)
{
    static uint32_t acceleration_start_ms = 0;
    static uint32_t deceleration_start_ms = 0;
    static uint8_t cycles_run = 0;
    static uint32_t brake_start_ms = 0;
    static chop_state_t chop_state = CHOP_READY_TO_START;

    uint32_t currentMillis;
    uint8_t chop_profile_selected[4];

    memcpy(chop_profile_selected, chop_profile_config[chop_profile], sizeof(chop_profile_selected));

    uint8_t time_to_accelerate = chop_profile_selected[0];
    uint8_t time_to_decelerate = chop_profile_selected[1];
    uint8_t number_of_cycles_before_break = chop_profile_selected[2];
    uint8_t time_to_brake = chop_profile_selected[3];

    critical_signal_state_t active_state;

    if (salt_mode == MODO_LIMITADO)
    {
        active_state = SIGNAL_UNINTERFERED;
    }
    else
    {
        active_state = SIGNAL_BYPASSED;
    }

    currentMillis = HAL_GetTick();

    switch (chop_state)
    {

    case CHOP_DEACTIVATED:
        CT_signal = SIGNAL_UNINTERFERED;
        FE_signal = SIGNAL_UNINTERFERED;
        break;

    case CHOP_READY_TO_START:
        CT_signal = active_state;
        FE_signal = active_state;
        acceleration_start_ms = HAL_GetTick();
        chop_state = CHOP_ACCELERATING;
        break;

    case CHOP_ACCELERATING:

        if (currentMillis - acceleration_start_ms > time_to_accelerate * 1000)
        {
            CT_signal = SIGNAL_OPEN;
            FE_signal = active_state;
            deceleration_start_ms = HAL_GetTick();
            chop_state = CHOP_DECELERATING;
        }
        break;

    case CHOP_DECELERATING:

        if (currentMillis - deceleration_start_ms > time_to_decelerate * 1000)
        {
            cycles_run++;

            if (cycles_run == number_of_cycles_before_break)
            {
                cycles_run = 0;
                CT_signal = SIGNAL_OPEN;
                FE_signal = SIGNAL_OPEN;
                brake_start_ms = HAL_GetTick();
                chop_state = CHOP_BRAKING;
            }
            else
            {
                chop_state = CHOP_READY_TO_START;
            }
        }
        break;

    case CHOP_BRAKING:
        if (currentMillis - brake_start_ms > time_to_brake * 1000)
        {
            chop_state = CHOP_READY_TO_START;
        }
        break;

    default:
        break;
    }
}

void Control_CriticalSignals(void)
{
    Control_CTsignal();
    Control_FEsignal();
}

void Control_CTsignal(void)
{
    if (CT_signal == SIGNAL_OPEN)
    {
        WriteCheck_RelayState(CT_C_GPIO_Port, CT_C_Pin, RELAY_ENERGIZED,
                              CT_M_GPIO_Port, CT_M_Pin);
        WriteCheck_RelayState(CT_DES_1_GPIO_Port, CT_DES_1_Pin, RELAY_ENERGIZED,
                              CT_EN_1_M_GPIO_Port, CT_EN_1_M_Pin);
        WriteCheck_RelayState(CT_DES_2_GPIO_Port, CT_DES_2_Pin, RELAY_ENERGIZED,
                              CT_EN_2_M_GPIO_Port, CT_EN_2_M_Pin);

        WriteCheck_RelayState(REG_CT_C_GPIO_Port, REG_CT_C_Pin, RELAY_ENERGIZED,
                              REG_CT_M_GPIO_Port, REG_CT_M_Pin);
    }
    else if (CT_signal == SIGNAL_BYPASSED)
    {
        WriteCheck_RelayState(CT_C_GPIO_Port, CT_C_Pin, RELAY_NORMAL,
                              CT_M_GPIO_Port, CT_M_Pin);
        WriteCheck_RelayState(CT_DES_1_GPIO_Port, CT_DES_1_Pin, RELAY_ENERGIZED,
                              CT_EN_1_M_GPIO_Port, CT_EN_1_M_Pin);
        WriteCheck_RelayState(CT_DES_2_GPIO_Port, CT_DES_2_Pin, RELAY_NORMAL,
                              CT_EN_2_M_GPIO_Port, CT_EN_2_M_Pin);

        WriteCheck_RelayState(REG_CT_C_GPIO_Port, REG_CT_C_Pin, RELAY_NORMAL,
                              REG_CT_M_GPIO_Port, REG_CT_M_Pin);
    }
    else
    {
        WriteCheck_RelayState(CT_C_GPIO_Port, CT_C_Pin, RELAY_NORMAL,
                              CT_M_GPIO_Port, CT_M_Pin);
        WriteCheck_RelayState(CT_DES_1_GPIO_Port, CT_DES_1_Pin, RELAY_NORMAL,
                              CT_EN_1_M_GPIO_Port, CT_EN_1_M_Pin);
        WriteCheck_RelayState(CT_DES_2_GPIO_Port, CT_DES_2_Pin, RELAY_NORMAL,
                              CT_EN_2_M_GPIO_Port, CT_EN_2_M_Pin);

        WriteCheck_RelayState(REG_CT_C_GPIO_Port, REG_CT_C_Pin, RELAY_NORMAL,
                              REG_CT_M_GPIO_Port, REG_CT_M_Pin);
    }

    if (CT_signal != prev_CT_signal)
    {
        sprintf(local_log_buffer, "CT_SIGNAL_STATE: %s", critical_signal_state_labels[CT_signal]);
        Log_Event(local_log_buffer);
    }
}

void Control_FEsignal(void)
{
    if (FE_signal == SIGNAL_OPEN)
    {
        WriteCheck_RelayState(FE_C_GPIO_Port, FE_C_Pin, RELAY_ENERGIZED,
                              FE_M_GPIO_Port, FE_M_Pin);
        WriteCheck_RelayState(FE_DES_1_GPIO_Port, FE_DES_1_Pin, RELAY_ENERGIZED,
                              FE_EN_1_M_GPIO_Port, FE_EN_1_M_Pin);
        WriteCheck_RelayState(FE_DES_2_GPIO_Port, FE_DES_2_Pin, RELAY_ENERGIZED,
                              FE_EN_2_M_GPIO_Port, FE_EN_2_M_Pin);

        WriteCheck_RelayState(REG_FE_C_GPIO_Port, REG_FE_C_Pin, RELAY_ENERGIZED,
                              REG_FE_M_GPIO_Port, REG_FE_M_Pin);
    }
    else if (FE_signal == SIGNAL_BYPASSED)
    {
        WriteCheck_RelayState(FE_C_GPIO_Port, FE_C_Pin, RELAY_NORMAL,
                              FE_M_GPIO_Port, FE_M_Pin);
        WriteCheck_RelayState(FE_DES_1_GPIO_Port, FE_DES_1_Pin, RELAY_ENERGIZED,
                              FE_EN_1_M_GPIO_Port, FE_EN_1_M_Pin);
        WriteCheck_RelayState(FE_DES_2_GPIO_Port, FE_DES_2_Pin, RELAY_NORMAL,
                              FE_EN_2_M_GPIO_Port, FE_EN_2_M_Pin);

        WriteCheck_RelayState(REG_FE_C_GPIO_Port, REG_FE_C_Pin, RELAY_NORMAL,
                              REG_FE_M_GPIO_Port, REG_FE_M_Pin);
    }
    else
    {
        WriteCheck_RelayState(FE_C_GPIO_Port, FE_C_Pin, RELAY_NORMAL,
                              FE_M_GPIO_Port, FE_M_Pin);
        WriteCheck_RelayState(FE_DES_1_GPIO_Port, FE_DES_1_Pin, RELAY_NORMAL,
                              FE_EN_1_M_GPIO_Port, FE_EN_1_M_Pin);
        WriteCheck_RelayState(FE_DES_2_GPIO_Port, FE_DES_2_Pin, RELAY_NORMAL,
                              FE_EN_2_M_GPIO_Port, FE_EN_2_M_Pin);

        WriteCheck_RelayState(REG_FE_C_GPIO_Port, REG_FE_C_Pin, RELAY_NORMAL,
                              REG_FE_M_GPIO_Port, REG_FE_M_Pin);
    }

    if (FE_signal != prev_FE_signal)
    {
        sprintf(local_log_buffer, "FE_SIGNAL_STATE: %s", critical_signal_state_labels[FE_signal]);
        Log_Event(local_log_buffer);
    }
}

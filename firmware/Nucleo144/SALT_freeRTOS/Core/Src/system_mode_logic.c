#include "globals.h"


void Handle_SaltMode_Transition(void);




void Handle_SaltMode_Transition(void)
{
    static salt_mode_t prev_salt_mode;

    prev_salt_mode = salt_mode;

    Check_RemoteCommand_Validity();

    if (internal_error == STATUS_ERROR)
    {
        salt_mode = MODO_NORMAL;
    }
    else
    {
        if (remote_command_active == COMMAND_ACTIVE)
        {
            if (strcmp(remote_command, "AISLADO_TOTAL") == 0)
            {
                salt_mode = MODO_TOTAL;
            }
            else if (strcmp(remote_command, "PARADA_TOTAL") == 0)
            {
                salt_mode = MODO_PARADA;
            }
            else if (strcmp(remote_command, "COCHE_DERIVA") == 0)
            {
                salt_mode = MODO_COCHE_DERIVA;
            }
            else if (strcmp(remote_command, "INTERMITENTE") == 0)
            {
                salt_mode = MODO_INTERMITENTE;
            }
        }
        else
        {
            if (MAT_switch_state_1 == SWITCH_ON && MAT_switch_state_2 == SWITCH_ON)
            {
                salt_mode = MODO_TOTAL;
            }
            else if (MAL_switch_state_1 == SWITCH_ON && MAL_switch_state_2 == SWITCH_ON)
            {
                salt_mode = MODO_LIMITADO;
            }
            else
            {
                salt_mode = MODO_NORMAL;
            }
        }
    }

    if (salt_mode != prev_salt_mode)
    {

        if (prev_salt_mode == MODO_LIMITADO)
        {
            Deactivate_SISBypass();
            buzzer_state = BUZZER_OFF;
            WriteCheck_RelayState(REG_MODO_LIMITADO_C_GPIO_Port, REG_MODO_LIMITADO_C_Pin, RELAY_NORMAL,
                                  REG_MODO_LIMITADO_M_GPIO_Port, REG_MODO_LIMITADO_M_Pin);
        }

        if (salt_mode == MODO_LIMITADO)
        {
            modo_limitado_activationMillis = HAL_GetTick();
            Activate_SISBypass();
            buzzer_state = BUZZER_ON_INTERMITENT;
            WriteCheck_RelayState(REG_MODO_LIMITADO_C_GPIO_Port, REG_MODO_LIMITADO_C_Pin, RELAY_ENERGIZED,
                                  REG_MODO_LIMITADO_M_GPIO_Port, REG_MODO_LIMITADO_M_Pin);
        }

        sprintf(local_log_buffer, "SALT_MODE_TRANSITION: %s --> %s", salt_mode_labels[prev_salt_mode], salt_mode_labels[salt_mode]);
        Log_Event(local_log_buffer);
    }
}

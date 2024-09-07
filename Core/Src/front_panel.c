#include "globals.h"

/*
void Led_Init(void);
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
void Control_Buzzer(void);
void Send_SystemStatus(void);
void Activate_ZoneRelay(void);
*/



void Led_Init()
{
    // LED RGB
    zone_led = LED_G;
    SIS_leds[0] = LED_G;
    SIS_leds[1] = LED_G;
    SIS_leds[2] = LED_G;
    SIS_leds[3] = LED_G;
    SIS_leds[4] = LED_G;
    active_command_led = LED_ALL_OFF;
    gps_led = LED_G;
    mode_MAL_led = LED_ALL_OFF;
    mode_MAT_led = LED_ALL_OFF;

    // Green LED
    power_led = LED_ON;
    CT_led = LED_OFF;
    FE_led = LED_OFF;
    chop_profile_leds[0] = LED_OFF;
    chop_profile_leds[1] = LED_OFF;
    chop_profile_leds[2] = LED_OFF;
    chop_profile_leds[3] = LED_OFF;
    chop_profile_leds[4] = LED_OFF;
}

void Display_SystemStatus(void)
{
    Build_SystemStatus();
    Send_SystemStatus();
}

void Build_SystemStatus(void)
{
    Build_SpeedDisplay();
    Build_LedIndicators();
}

void Build_SpeedDisplay(void)
{
    seven_segment_t speed_display[4];
    int integer_part = (int)speed;
    int decimal_part = (int)((speed - integer_part) * 10);
    if (speed_source == SPEED_NONE)
    {
        for (int i = 0; i < 4; i++)
        {
            speed_display[i].digit = DASH;
            speed_display[i].decimal_point = 0;
        }
    }
    else
    {
        speed_display[0].digit = (integer_part / 100) % 10;
        speed_display[0].decimal_point = 0;

        speed_display[1].digit = (integer_part / 10) % 10;
        speed_display[1].decimal_point = 0;

        speed_display[2].digit = integer_part % 10;
        speed_display[2].decimal_point = 1;

        speed_display[3].digit = decimal_part;
        speed_display[3].decimal_point = 0;
    }

    if (speed_display[0].digit == ZERO)
    {
        speed_display[0].digit = NONE_DIGIT;
        if (speed_display[1].digit == ZERO)
        {
            speed_display[1].digit = NONE_DIGIT;
        }
    }

    for (uint8_t digit = 0; digit < 4; digit++)
    {
        digit7Segment[digit] = digitTo7Segment(speed_display[digit].digit);

        if (speed_display[digit].decimal_point)
        {
            digit7Segment[digit] |= DECIMAL_POINT_MASK;
        }
    }
}

void Build_LedIndicators()
{
    if (salt_mode == MODO_NORMAL)
    {
        mode_MAL_led = LED_ALL_OFF;
        mode_MAT_led = LED_ALL_OFF;

        CT_led = LED_OFF;
        FE_led = LED_OFF;

        for (uint8_t i = 0; i < 5; i++)
        {
            chop_profile_leds[i] = LED_OFF;
        }

        if (remote_command_active == COMMAND_ACTIVE)
        {
            active_command_led = LED_G;
        }
        else
        {
            active_command_led = LED_ALL_OFF;
        }

        for (uint8_t i = 0; i < 5; i++)
        {
            if (SIS_state[i].FE_state == SIGNAL_ERROR || SIS_state[i].CT_state == SIGNAL_ERROR)
            {
                SIS_leds[i] = LED_R;
            }
            else
            {
                SIS_leds[i] = LED_G;
            }
        }
    }
    else if (salt_mode == MODO_LIMITADO)
    {
        mode_MAL_led = LED_G;
        mode_MAT_led = LED_ALL_OFF;

        for (uint8_t i = 0; i < 5; i++)
        {
            if (chop_profile == i)
            {
                chop_profile_leds[i] = LED_ON;
            }
            else
            {
                chop_profile_leds[i] = LED_OFF;
            }
        }

        if (remote_command_active == COMMAND_ACTIVE)
        {
            active_command_led = LED_G;
        }
        else
        {
            active_command_led = LED_ALL_OFF;
        }

        // SIS_leds[i] remains unchanged from normal mode to show SIS failing

        if (CT_signal == SIGNAL_OPEN)
        {
            CT_led = LED_ON;
        }
        else
        {
            CT_led = LED_OFF;
        }

        if (FE_signal == SIGNAL_OPEN)
        {
            FE_led = LED_ON;
        }
        else
        {
            FE_led = LED_OFF;
        }
    }
    else if (salt_mode == MODO_TOTAL)
    {

        mode_MAL_led = LED_ALL_OFF;
        mode_MAT_led = LED_G;

        CT_led = LED_OFF;
        FE_led = LED_OFF;

        for (uint8_t i = 0; i < 5; i++)
        {
            chop_profile_leds[i] = LED_OFF;
        }

        if (remote_command_active == COMMAND_ACTIVE)
        {
            active_command_led = LED_G;
        }
        else
        {
            active_command_led = LED_ALL_OFF;
        }

        for (uint8_t i = 0; i < 5; i++)
        {
            SIS_leds[i] = LED_R;
        }
    }
    else if (salt_mode == MODO_PARADA)
    {
        mode_MAL_led = LED_ALL_OFF;
        mode_MAT_led = LED_ALL_OFF;

        CT_led = LED_ON;
        FE_led = LED_ON;

        for (uint8_t i = 0; i < 5; i++)
        {
            chop_profile_leds[i] = LED_OFF;
        }

        active_command_led = LED_G;

        for (uint8_t i = 0; i < 5; i++)
        {
            SIS_leds[i] = LED_R;
        }
    }
    else if (salt_mode == MODO_COCHE_DERIVA)
    {
        mode_MAL_led = LED_ALL_OFF;
        mode_MAT_led = LED_ALL_OFF;

        CT_led = LED_OFF;
        FE_led = LED_ON;

        for (uint8_t i = 0; i < 5; i++)
        {
            chop_profile_leds[i] = LED_OFF;
        }

        active_command_led = LED_G;

        for (uint8_t i = 0; i < 5; i++)
        {
            SIS_leds[i] = LED_R;
        }
    }
    else if (salt_mode == MODO_INTERMITENTE)
    {
        mode_MAL_led = LED_ALL_OFF;
        mode_MAT_led = LED_ALL_OFF;

        if (CT_signal == SIGNAL_OPEN)
        {
            CT_led = LED_ON;
        }
        else
        {
            CT_led = LED_OFF;
        }

        if (FE_signal == SIGNAL_OPEN)
        {
            FE_led = LED_ON;
        }
        else
        {
            FE_led = LED_OFF;
        }

        for (uint8_t i = 0; i < 5; i++)
        {
            if (chop_profile == i)
            {
                chop_profile_leds[i] = LED_ON;
            }
            else
            {
                chop_profile_leds[i] = LED_OFF;
            }
        }

        active_command_led = LED_G;

        for (uint8_t i = 0; i < 5; i++)
        {
            SIS_leds[i] = LED_R;
        }
    }

    switch (current_zone)
    {
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

    if (gps_status == STATUS_OK)
    {
        gps_led = LED_G;
    }
    else
    {
        gps_led = LED_R;
    }

    setDigit4_regValue();
    setDigit5_regValue();
    setDigit6_regValue();
    setDigit7_regValue();
}

uint8_t extractBit(rgb_led_state_t state, uint8_t bitPos)
{
    return (state >> bitPos) & 0x01;
}

void setOrClearBit(uint8_t *byteVal, uint8_t bitPos, uint8_t value)
{
    if (value)
    {
        *byteVal |= (1 << bitPos); // Set the bit
    }
    else
    {
        *byteVal &= ~(1 << bitPos); // Clear the bit
    }
}

void setDigit4_regValue()
{
    uint8_t byteVal = 0x00;

    uint8_t D7_DP = chop_profile_leds[0];             // LED14      - CC_4 - A_DP - CHOP_1
    uint8_t D6_A = extractBit(mode_MAL_led, 1);       // LED2_GREEN - CC_4 - A_A  - MODO LIMITADO
    uint8_t D5_B = extractBit(mode_MAT_led, 1);       // LED3_GREEN - CC_4 - A_B  - MODO TOTAL
    uint8_t D4_C = extractBit(active_command_led, 1); // LED4_GREEN - CC_4 - A_C  - COMANDO REMOTO
    uint8_t D3_D = extractBit(gps_led, 1);            // LED5_GREEN - CC_4 - A_D  - GPS
    uint8_t D2_E = extractBit(zone_led, 1);           // LED6_GREEN - CC_4 - A_E  - ZONA
    uint8_t D1_F = extractBit(SIS_leds[0], 1);        // LED7_GREEN - CC_4 - A_F  - SIS_1
    uint8_t D0_G = chop_profile_leds[1];              // LED15      - CC_4 - A_G  - CHOP_2

    uint8_t vars[8] = {D0_G, D1_F, D2_E, D3_D, D4_C, D5_B, D6_A, D7_DP};

    for (uint8_t i = 0; i < 8; i++)
    {
        setOrClearBit(&byteVal, i, vars[i]);
    }

    digit4_reg_value = byteVal;
}

void setDigit5_regValue()
{
    uint8_t byteVal = 0x00;

    uint8_t D7_DP = power_led;                        //  LED1      - CC_5 - A_DP - POWER
    uint8_t D6_A = extractBit(mode_MAL_led, 2);       //  LED2_RED  - CC_5 - A_A  - MODO LIMITADO
    uint8_t D5_B = extractBit(mode_MAT_led, 2);       //  LED3_RED  - CC_5 - A_B  - MODO TOTAL
    uint8_t D4_C = extractBit(active_command_led, 2); //  LED4_RED  - CC_5 - A_C  - COMANDO REMOTO
    uint8_t D3_D = extractBit(gps_led, 2);            //  LED5_RED  - CC_5 - A_D  - GPS
    uint8_t D2_E = extractBit(zone_led, 2);           //  LED6_RED  - CC_5 - A_E  - ZONA
    uint8_t D1_F = extractBit(SIS_leds[0], 2);        //  LED7_RED  - CC_5 - A_F  - SIS_1
    uint8_t D0_G = chop_profile_leds[2];              //  LED16     - CC_5 - A_G  - CHOP_3

    uint8_t vars[8] = {D0_G, D1_F, D2_E, D3_D, D4_C, D5_B, D6_A, D7_DP};

    for (uint8_t i = 0; i < 8; i++)
    {
        setOrClearBit(&byteVal, i, vars[i]);
    }

    digit5_reg_value = byteVal;
}

void setDigit6_regValue()
{
    uint8_t byteVal = 0x00;

    uint8_t D7_DP = CT_led;                    // LED12       - CC_6 - A_DP - CT
    uint8_t D6_A = extractBit(SIS_leds[1], 1); // LED8_GREEN  - CC_6 - A_A  - SIS_2
    uint8_t D5_B = extractBit(SIS_leds[2], 1); // LED9_GREEN  - CC_6 - A_B  - SIS_3
    uint8_t D4_C = extractBit(SIS_leds[3], 1); // LED10_GREEN - CC_6 - A_C  - SIS_4
    uint8_t D3_D = extractBit(SIS_leds[4], 1); // LED11_GREEN - CC_6 - A_D  - SIS_5
    uint8_t D2_E = 0;                          // NC          - CC_6 - A_E  - NC
    uint8_t D1_F = extractBit(SIS_leds[0], 0); // LED7_BLUE   - CC_6 - A_F  - SIS_1-->ZONA
    uint8_t D0_G = chop_profile_leds[3];       // LED17       - CC_6 - A_G  - CHOP_4

    uint8_t vars[8] = {D0_G, D1_F, D2_E, D3_D, D4_C, D5_B, D6_A, D7_DP};

    for (uint8_t i = 0; i < 8; i++)
    {
        setOrClearBit(&byteVal, i, vars[i]);
    }

    digit6_reg_value = byteVal;
}

void setDigit7_regValue()
{
    uint8_t byteVal = 0x00;

    uint8_t D7_DP = FE_led;                    // LED13     - CC_7 - A_DP - FE
    uint8_t D6_A = extractBit(SIS_leds[1], 2); // LED8_RED  - CC_7 - A_A  - SIS_2
    uint8_t D5_B = extractBit(SIS_leds[2], 2); // LED9_RED  - CC_7 - A_B  - SIS_3
    uint8_t D4_C = extractBit(SIS_leds[3], 2); // LED10_RED - CC_7 - A_C  - SIS_4
    uint8_t D3_D = extractBit(SIS_leds[4], 2); // LED11_RED - CC_7 - A_D  - SIS_5
    uint8_t D2_E = 0;                          // NC        - CC_7 - A_E  - NC
    uint8_t D1_F = 0;                          // NC        - CC_7 - A_F  - NC
    uint8_t D0_G = chop_profile_leds[4];       // LED18     - CC_7 - A_G  - CHOP_5

    uint8_t vars[8] = {D0_G, D1_F, D2_E, D3_D, D4_C, D5_B, D6_A, D7_DP};

    for (uint8_t i = 0; i < 8; i++)
    {
        setOrClearBit(&byteVal, i, vars[i]);
    }

    digit7_reg_value = byteVal;
}

void Send_SystemStatus(void)
{
    for (uint8_t digit = 0; digit < 4; digit++)
    {
        LedDriver_WriteReg(digit + 1, digit7Segment[digit], &I2C_HANDLE);
    }
    LedDriver_WriteReg(0x05, digit4_reg_value, &I2C_HANDLE);
    LedDriver_WriteReg(0x06, digit5_reg_value, &I2C_HANDLE);
    LedDriver_WriteReg(0x07, digit6_reg_value, &I2C_HANDLE);
    LedDriver_WriteReg(0x08, digit7_reg_value, &I2C_HANDLE);

    Activate_ZoneRelay();
    Control_Buzzer();
}

void Control_Buzzer(void)
{
    static uint32_t buzzer_toggle_ms = 0;
    uint32_t currentMillis;

    if (buzzer_state == BUZZER_OFF)
    {
        HAL_GPIO_WritePin(BUZZER_C_GPIO_Port, BUZZER_C_Pin, 0);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
    }
    else if (buzzer_state == BUZZER_ON_CONTINUOS)
    {
        HAL_GPIO_WritePin(BUZZER_C_GPIO_Port, BUZZER_C_Pin, 1);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
    }
    else if (buzzer_state == BUZZER_ON_INTERMITENT)
    {
        currentMillis = HAL_GetTick();
        if (currentMillis - buzzer_toggle_ms > BUZZER_SOUND_PERIOD_S * 1000)
        {
            HAL_GPIO_TogglePin(BUZZER_C_GPIO_Port, BUZZER_C_Pin);
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

            buzzer_toggle_ms = currentMillis;
        }
    }
}


void Activate_ZoneRelay(void)
{
    prev_zone_relay = zone_relay;

    if (current_zone == ZONE_3)
    {
        zone_relay = RELAY_ENERGIZED;
        WriteCheck_RelayState(ZONA_C_GPIO_Port, ZONA_C_Pin, RELAY_ENERGIZED,
                              ZONA_M_GPIO_Port, ZONA_M_Pin);
    }
    else
    {
        zone_relay = RELAY_NORMAL;
        WriteCheck_RelayState(ZONA_C_GPIO_Port, ZONA_C_Pin, RELAY_NORMAL,
                              ZONA_M_GPIO_Port, ZONA_M_Pin);
    }

    if (zone_relay != prev_zone_relay)
    {
        sprintf(local_log_buffer, "ZONE_RELAY: %s", zone_relay ? "RELAY_ENERGIZED" : "RELAY_NORMAL");
        Log_Event(local_log_buffer);
    }
}

#include "globals.h"

/*
void WriteCheck_RelayState(GPIO_TypeDef *WritecGPIOx, uint16_t Write_GPIO_Pin, GPIO_PinState PinState,
                           GPIO_TypeDef *Read_GPIOx, uint16_t Read_GPIO_Pin);
void Test_ActivateRelays(void);
void Test_WriteCheckRelays(void);
*/

void WriteCheck_RelayState(GPIO_TypeDef *WritecGPIOx, uint16_t Write_GPIO_Pin, GPIO_PinState WritePinState,
                           GPIO_TypeDef *Read_GPIOx, uint16_t Read_GPIO_Pin)
{
    GPIO_PinState readPinState;
    HAL_GPIO_WritePin(WritecGPIOx, Write_GPIO_Pin, WritePinState);
    HAL_Delay(100);
    readPinState = HAL_GPIO_ReadPin(Read_GPIOx, Read_GPIO_Pin);

    // Read pin are connected with inverted values
    if (WritePinState == readPinState)
    {
        internal_error = STATUS_ERROR;
        HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
        Log_Event("ERROR INTERNO");
    }
}

void Test_ActivateRelays(void)
{
    HAL_GPIO_WritePin(SIS_1_CT_BP_C_GPIO_Port, SIS_1_CT_BP_C_Pin, RELAY_ENERGIZED);
    HAL_GPIO_WritePin(SIS_1_FE_BP_C_GPIO_Port, SIS_1_FE_BP_C_Pin, RELAY_ENERGIZED);
    HAL_GPIO_WritePin(SIS_2_CT_BP_C_GPIO_Port, SIS_2_CT_BP_C_Pin, RELAY_ENERGIZED);
    HAL_GPIO_WritePin(SIS_2_FE_BP_C_GPIO_Port, SIS_2_FE_BP_C_Pin, RELAY_ENERGIZED);
    HAL_GPIO_WritePin(SIS_3_CT_BP_C_GPIO_Port, SIS_3_CT_BP_C_Pin, RELAY_ENERGIZED);
    HAL_GPIO_WritePin(SIS_3_FE_BP_C_GPIO_Port, SIS_3_FE_BP_C_Pin, RELAY_ENERGIZED);
    HAL_GPIO_WritePin(SIS_4_CT_BP_C_GPIO_Port, SIS_4_CT_BP_C_Pin, RELAY_ENERGIZED);
    HAL_GPIO_WritePin(SIS_4_FE_BP_C_GPIO_Port, SIS_4_FE_BP_C_Pin, RELAY_ENERGIZED);
    HAL_GPIO_WritePin(SIS_5_CT_BP_C_GPIO_Port, SIS_5_CT_BP_C_Pin, RELAY_ENERGIZED);
    HAL_GPIO_WritePin(SIS_5_FE_BP_C_GPIO_Port, SIS_5_FE_BP_C_Pin, RELAY_ENERGIZED);

    HAL_Delay(2000);

    HAL_GPIO_WritePin(SIS_1_CT_BP_C_GPIO_Port, SIS_1_CT_BP_C_Pin, RELAY_NORMAL);
    HAL_GPIO_WritePin(SIS_1_FE_BP_C_GPIO_Port, SIS_1_FE_BP_C_Pin, RELAY_NORMAL);
    HAL_GPIO_WritePin(SIS_2_CT_BP_C_GPIO_Port, SIS_2_CT_BP_C_Pin, RELAY_NORMAL);
    HAL_GPIO_WritePin(SIS_2_FE_BP_C_GPIO_Port, SIS_2_FE_BP_C_Pin, RELAY_NORMAL);
    HAL_GPIO_WritePin(SIS_3_CT_BP_C_GPIO_Port, SIS_3_CT_BP_C_Pin, RELAY_NORMAL);
    HAL_GPIO_WritePin(SIS_3_FE_BP_C_GPIO_Port, SIS_3_FE_BP_C_Pin, RELAY_NORMAL);
    HAL_GPIO_WritePin(SIS_4_CT_BP_C_GPIO_Port, SIS_4_CT_BP_C_Pin, RELAY_NORMAL);
    HAL_GPIO_WritePin(SIS_4_FE_BP_C_GPIO_Port, SIS_4_FE_BP_C_Pin, RELAY_NORMAL);
    HAL_GPIO_WritePin(SIS_5_CT_BP_C_GPIO_Port, SIS_5_CT_BP_C_Pin, RELAY_NORMAL);
    HAL_GPIO_WritePin(SIS_5_FE_BP_C_GPIO_Port, SIS_5_FE_BP_C_Pin, RELAY_NORMAL);

    HAL_Delay(2000);

    HAL_GPIO_WritePin(REG_1_C_GPIO_Port, REG_1_C_Pin, RELAY_ENERGIZED);
    HAL_GPIO_WritePin(REG_2_C_GPIO_Port, REG_2_C_Pin, RELAY_ENERGIZED);
    HAL_GPIO_WritePin(REG_3_C_GPIO_Port, REG_3_C_Pin, RELAY_ENERGIZED);
    HAL_GPIO_WritePin(REG_4_C_GPIO_Port, REG_4_C_Pin, RELAY_ENERGIZED);
    HAL_GPIO_WritePin(ZONA_C_GPIO_Port, ZONA_C_Pin, RELAY_ENERGIZED);

    HAL_Delay(2000);

    HAL_GPIO_WritePin(REG_1_C_GPIO_Port, REG_1_C_Pin, RELAY_NORMAL);
    HAL_GPIO_WritePin(REG_2_C_GPIO_Port, REG_2_C_Pin, RELAY_NORMAL);
    HAL_GPIO_WritePin(REG_3_C_GPIO_Port, REG_3_C_Pin, RELAY_NORMAL);
    HAL_GPIO_WritePin(REG_4_C_GPIO_Port, REG_4_C_Pin, RELAY_NORMAL);
    HAL_GPIO_WritePin(ZONA_C_GPIO_Port, ZONA_C_Pin, RELAY_NORMAL);

    HAL_Delay(2000);

    HAL_GPIO_WritePin(CT_C_GPIO_Port, CT_C_Pin, RELAY_ENERGIZED);
    HAL_GPIO_WritePin(CT_DES_1_GPIO_Port, CT_DES_1_Pin, RELAY_ENERGIZED);
    HAL_GPIO_WritePin(CT_DES_2_GPIO_Port, CT_DES_2_Pin, RELAY_ENERGIZED);

    HAL_GPIO_WritePin(FE_C_GPIO_Port, FE_C_Pin, RELAY_ENERGIZED);
    HAL_GPIO_WritePin(FE_DES_1_GPIO_Port, FE_DES_1_Pin, RELAY_ENERGIZED);
    HAL_GPIO_WritePin(FE_DES_2_GPIO_Port, FE_DES_2_Pin, RELAY_ENERGIZED);

    HAL_Delay(2000);

    HAL_GPIO_WritePin(CT_C_GPIO_Port, CT_C_Pin, RELAY_NORMAL);
    HAL_GPIO_WritePin(CT_DES_1_GPIO_Port, CT_DES_1_Pin, RELAY_NORMAL);
    HAL_GPIO_WritePin(CT_DES_2_GPIO_Port, CT_DES_2_Pin, RELAY_NORMAL);

    HAL_GPIO_WritePin(FE_C_GPIO_Port, FE_C_Pin, RELAY_NORMAL);
    HAL_GPIO_WritePin(FE_DES_1_GPIO_Port, FE_DES_1_Pin, RELAY_NORMAL);
    HAL_GPIO_WritePin(FE_DES_2_GPIO_Port, FE_DES_2_Pin, RELAY_NORMAL);

    HAL_Delay(2000);
}

void Test_WriteCheckRelays(void)
{
    WriteCheck_RelayState(SIS_1_CT_BP_C_GPIO_Port, SIS_1_CT_BP_C_Pin, RELAY_ENERGIZED,
                          SIS_1_CT_BP_M_GPIO_Port, SIS_1_CT_BP_M_Pin);
    WriteCheck_RelayState(SIS_1_FE_BP_C_GPIO_Port, SIS_1_FE_BP_C_Pin, RELAY_ENERGIZED,
                          SIS_1_FE_BP_M_GPIO_Port, SIS_1_FE_BP_M_Pin);
    WriteCheck_RelayState(SIS_2_CT_BP_C_GPIO_Port, SIS_2_CT_BP_C_Pin, RELAY_ENERGIZED,
                          SIS_2_CT_BP_M_GPIO_Port, SIS_2_CT_BP_M_Pin);
    WriteCheck_RelayState(SIS_2_FE_BP_C_GPIO_Port, SIS_2_FE_BP_C_Pin, RELAY_ENERGIZED,
                          SIS_2_FE_BP_M_GPIO_Port, SIS_2_FE_BP_M_Pin);
    WriteCheck_RelayState(SIS_3_CT_BP_C_GPIO_Port, SIS_3_CT_BP_C_Pin, RELAY_ENERGIZED,
                          SIS_3_CT_BP_M_GPIO_Port, SIS_3_CT_BP_M_Pin);
    WriteCheck_RelayState(SIS_3_FE_BP_C_GPIO_Port, SIS_3_FE_BP_C_Pin, RELAY_ENERGIZED,
                          SIS_3_FE_BP_M_GPIO_Port, SIS_3_FE_BP_M_Pin);
    WriteCheck_RelayState(SIS_4_CT_BP_C_GPIO_Port, SIS_4_CT_BP_C_Pin, RELAY_ENERGIZED,
                          SIS_4_CT_BP_M_GPIO_Port, SIS_4_CT_BP_M_Pin);
    WriteCheck_RelayState(SIS_4_FE_BP_C_GPIO_Port, SIS_4_FE_BP_C_Pin, RELAY_ENERGIZED,
                          SIS_4_FE_BP_M_GPIO_Port, SIS_4_FE_BP_M_Pin);
    WriteCheck_RelayState(SIS_5_CT_BP_C_GPIO_Port, SIS_5_CT_BP_C_Pin, RELAY_ENERGIZED,
                          SIS_5_CT_BP_M_GPIO_Port, SIS_5_CT_BP_M_Pin);
    WriteCheck_RelayState(SIS_5_FE_BP_C_GPIO_Port, SIS_5_FE_BP_C_Pin, RELAY_ENERGIZED,
                          SIS_5_FE_BP_M_GPIO_Port, SIS_5_FE_BP_M_Pin);

    HAL_Delay(2000);

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

    HAL_Delay(2000);

    WriteCheck_RelayState(REG_1_C_GPIO_Port, REG_1_C_Pin, RELAY_ENERGIZED,
                          REG_1_M_GPIO_Port, REG_1_M_Pin);
    WriteCheck_RelayState(REG_2_C_GPIO_Port, REG_2_C_Pin, RELAY_ENERGIZED,
                          REG_2_M_GPIO_Port, REG_2_M_Pin);
    WriteCheck_RelayState(REG_3_C_GPIO_Port, REG_3_C_Pin, RELAY_ENERGIZED,
                          REG_3_M_GPIO_Port, REG_3_M_Pin);
    WriteCheck_RelayState(REG_4_C_GPIO_Port, REG_4_C_Pin, RELAY_ENERGIZED,
                          REG_4_M_GPIO_Port, REG_4_M_Pin);
    WriteCheck_RelayState(ZONA_C_GPIO_Port, ZONA_C_Pin, RELAY_ENERGIZED,
                          ZONA_M_GPIO_Port, ZONA_M_Pin);

    HAL_Delay(2000);

    WriteCheck_RelayState(REG_1_C_GPIO_Port, REG_1_C_Pin, RELAY_NORMAL,
                          REG_1_M_GPIO_Port, REG_1_M_Pin);
    WriteCheck_RelayState(REG_2_C_GPIO_Port, REG_2_C_Pin, RELAY_NORMAL,
                          REG_2_M_GPIO_Port, REG_2_M_Pin);
    WriteCheck_RelayState(REG_3_C_GPIO_Port, REG_3_C_Pin, RELAY_NORMAL,
                          REG_3_M_GPIO_Port, REG_3_M_Pin);
    WriteCheck_RelayState(REG_4_C_GPIO_Port, REG_4_C_Pin, RELAY_NORMAL,
                          REG_4_M_GPIO_Port, REG_4_M_Pin);
    WriteCheck_RelayState(ZONA_C_GPIO_Port, ZONA_C_Pin, RELAY_NORMAL,
                          ZONA_M_GPIO_Port, ZONA_M_Pin);

    HAL_Delay(2000);

    WriteCheck_RelayState(CT_C_GPIO_Port, CT_C_Pin, RELAY_ENERGIZED,
                          CT_M_GPIO_Port, CT_M_Pin);
    WriteCheck_RelayState(CT_DES_1_GPIO_Port, CT_DES_1_Pin, RELAY_ENERGIZED,
                          CT_EN_1_M_GPIO_Port, CT_EN_1_M_Pin);
    WriteCheck_RelayState(CT_DES_2_GPIO_Port, CT_DES_2_Pin, RELAY_ENERGIZED,
                          CT_EN_2_M_GPIO_Port, CT_EN_2_M_Pin);

    WriteCheck_RelayState(FE_C_GPIO_Port, FE_C_Pin, RELAY_ENERGIZED,
                          FE_M_GPIO_Port, FE_M_Pin);
    WriteCheck_RelayState(FE_DES_1_GPIO_Port, FE_DES_1_Pin, RELAY_ENERGIZED,
                          FE_EN_1_M_GPIO_Port, FE_EN_1_M_Pin);
    WriteCheck_RelayState(FE_DES_2_GPIO_Port, FE_DES_2_Pin, RELAY_ENERGIZED,
                          FE_EN_2_M_GPIO_Port, FE_EN_2_M_Pin);

    HAL_Delay(2000);

    WriteCheck_RelayState(CT_C_GPIO_Port, CT_C_Pin, RELAY_NORMAL,
                          CT_M_GPIO_Port, CT_M_Pin);
    WriteCheck_RelayState(CT_DES_1_GPIO_Port, CT_DES_1_Pin, RELAY_NORMAL,
                          CT_EN_1_M_GPIO_Port, CT_EN_1_M_Pin);
    WriteCheck_RelayState(CT_DES_2_GPIO_Port, CT_DES_2_Pin, RELAY_NORMAL,
                          CT_EN_2_M_GPIO_Port, CT_EN_2_M_Pin);

    WriteCheck_RelayState(FE_C_GPIO_Port, FE_C_Pin, RELAY_NORMAL,
                          FE_M_GPIO_Port, FE_M_Pin);
    WriteCheck_RelayState(FE_DES_1_GPIO_Port, FE_DES_1_Pin, RELAY_NORMAL,
                          FE_EN_1_M_GPIO_Port, FE_EN_1_M_Pin);
    WriteCheck_RelayState(FE_DES_2_GPIO_Port, FE_DES_2_Pin, RELAY_NORMAL,
                          FE_EN_2_M_GPIO_Port, FE_EN_2_M_Pin);

    HAL_Delay(2000);
}

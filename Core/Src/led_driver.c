/*
 * led_driver.c
 *
 *  Created on: Jul 28, 2024
 *      Author: manus
 */

#include "led_driver.h"

void I2C_Reset(I2C_HandleTypeDef *hi2c)
{
    __HAL_RCC_I2C1_FORCE_RESET();
    HAL_Delay(1); // Small delay
    __HAL_RCC_I2C1_RELEASE_RESET();

    // Reinitialize the I2C peripheral
    HAL_I2C_Init(hi2c);
}

HAL_StatusTypeDef LedDriver_WriteReg(uint8_t reg_address, uint8_t reg_value, I2C_HandleTypeDef *hi2c)
{
    uint8_t data[2] = {reg_address, reg_value};
    return HAL_I2C_Master_Transmit(hi2c, AS1115_I2C_ADDRESS << 1, data, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef LedDriver_Init(I2C_HandleTypeDef *hi2c)
{
    LedDriver_WriteReg(SHUTDOWN_REG_ADDR, NORMAL_OPERATION_MODE, hi2c);
    LedDriver_WriteReg(SCAN_LIMIT_REG_ADDR, DISPLAY_7_DIGITS, hi2c);
    return LedDriver_WriteReg(GLOBAL_INTENSITY_REG_ADDR, MID_INTENSITY, hi2c);
}

uint8_t digitTo7Segment(uint8_t digit)
{
    static const uint8_t segmentMap[11] = {
        0b01111110, // 0
        0b00110000, // 1
        0b01101101, // 2
        0b01111001, // 3
        0b00110011, // 4
        0b01011011, // 5
        0b01011111, // 6
        0b01110000, // 7
        0b01111111, // 8
        0b01111011, // 9
        0b00000001  // -
    };

    if (digit > 10)
    {
        return 0b00000000; // Return no digit
    }

    return segmentMap[digit];
}

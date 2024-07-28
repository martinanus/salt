
/*
 * led_driver.h
 *
 *  Created on: Jul 28, 2024
 *      Author: manus
 */

#ifndef __LED_DRIVER_H
#define __LED_DRIVER_H


#include "stm32f4xx_hal.h"


#define AS1115_I2C_ADDRESS 0x00
#define SHUTDOWN_REG_ADDR 0x0C
#define SHUTDOWN_MODE 0x00
#define NORMAL_OPERATION_MODE 0x01
#define SCAN_LIMIT_REG_ADDR 0x0B
#define DISPLAY_7_DIGITS 0x07
#define GLOBAL_INTENSITY_REG_ADDR 0x0A
#define MAX_INTENSITY 0x0F
#define MID_INTENSITY 0x09
#define MIN_INTENSITY 0x01
#define FEATURE_REG_ADDR 0x0E
#define DECODE_REG_MASK 0x04  // 0: Code-B | 1:HEX
#define DECODE_ENABLE_REG_ADDR 0x09
#define NO_DECODE_MODE 0x00
#define DECODE_DIGIT_0_TO_5_MODE 0x3F

void I2C_Reset(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef LedDriver_WriteReg(uint8_t reg_address, uint8_t reg_value, I2C_HandleTypeDef *hi2c);
void LedDriver_Init(I2C_HandleTypeDef *hi2c);
uint8_t digitTo7Segment(uint8_t digit);



#endif /* __LED_DRIVER_H */

/*
 * VCNL4200.c
 *
 *  Created on: Jun 9, 2019
 *      Author: kevin
 */

#include "VCNL4200.h"
#include <stdio.h>

static I2C_HandleTypeDef *_hi2c;

void VCNL4200_initialize(I2C_HandleTypeDef *h) {
	_hi2c = h;

    // ALS Shutdown, Why does the high bit need to be 1?
    VCNL4200_WriteRegister(VCNL4200_ALS_CONF_REG, 0b00000001,0b00000001);
    // PS 4 Pulses, Disable Smart Persistance, Force Mode Enable 
    VCNL4200_WriteRegister(VCNL4200_PS_CONF3_MS_REG,0b11000000,0b00001000);
    // PS 1/1280 Duty Cycle 1T PS On, 16 Bit output
    VCNL4200_WriteRegister(VCNL4200_PS_CONF1_CONF2_REG,0b01001000,0b00000000);
 
}

uint8_t VCNL4200_exists()
{
	uint8_t retval[2];

    HAL_I2C_Mem_Read(_hi2c, VCNL4200_ADDR<<1, VCNL4200_DeviceID_REG, 1, retval, 2, 1000);

    if((retval[0] == 0x58) && retval[1] == 0x10)
        return 1;
    else
	    return 0;
}

uint16_t VCNL4200_getProximity() {
    // Trigger one Cycle
    //VCNL4200_WriteRegister(VCNL4200_PS_CONF3_MS_REG,0b01001100,0b00000000);

    return VCNL4200_ReadRegister(VCNL4200_PROXIMITY_REG);
}

uint8_t VCNL4200_WriteRegister(uint8_t reg, uint8_t low, uint8_t high) {
    uint8_t buffer[2];
    buffer[0] = low;
    buffer[1] = high;
    HAL_StatusTypeDef r = HAL_I2C_Mem_Write(_hi2c, VCNL4200_ADDR<<1, reg, 1, buffer, 2, 1000);

    if (r == HAL_OK)
        return 1; 
    else
        return 0;
    
}

uint16_t VCNL4200_ReadRegister(uint8_t reg) {
    uint8_t buffer[2];
    HAL_I2C_Mem_Read(_hi2c, VCNL4200_ADDR<<1, reg, 1, buffer, 2, 1000);

    uint16_t retval = buffer[0] + (uint16_t)(buffer[1]) * 256;

    return retval;
}

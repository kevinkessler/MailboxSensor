/*
 * VCNL4200.h
 *
 *  Created on: Jun 9, 2019
 *      Author: kevin
 */
#include "main.h"
#include <stdio.h>

#ifndef VCNL4200_H_
#define VCNL4200_H_

#define VCNL4200_ADDR 0x51
#define VCNL4200_ALS_CONF_REG 0x00
#define VCNL4200_ALS_THDH_REG 0x01 //Ambient Light Sensor Threshold Data High
#define VCNL4200_ALS_THDL_REG 0x02 //Ambient Light Sensor Threshold Data Low
#define VCNL4200_PS_CONF1_CONF2_REG 0x03
#define VCNL4200_PS_CONF3_MS_REG 0x04 //Conf3 and Mode Select
#define VCNL4200_PS_CANC_REG 0x05
#define VCNL4200_PS_THDL_REG 0x06 //Proximity Sensor Threshold Data Low
#define VCNL4200_PS_THDH_REG 0x07 //Proximity Sensor Threshold Data High
#define VCNL4200_PROXIMITY_REG 0x08
#define VCNL4200_AMBIENT_REG 0x09
#define VCNL4200_WHITE_REG 0x0A
#define VCNL4200_INT_FLAG_REG 0x0D
#define VCNL4200_DeviceID_REG 0x0E

uint8_t VCNL4200_exists(void);
void VCNL4200_initialize(I2C_HandleTypeDef *h);
uint8_t VCNL4200_WriteRegister(uint8_t register, uint8_t low, uint8_t high);
uint16_t VCNL4200_ReadRegister(uint8_t);
uint16_t VCNL4200_getProximity(void);

#endif /* VCNL4200_H_ */
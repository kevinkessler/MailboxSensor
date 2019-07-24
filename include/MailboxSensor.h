/**
 *  @filename   :   MailboxSensor.h
 *  @brief      :   Application Logic for the Mailbox Sensor
 *
 *  @author     :   Kevin Kessler
 *
 * Copyright (C) 2019 Kevin Kessler
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#ifndef INC_MAILBOXSENSOR_H_
#define INC_MAILBOXSENSOR_H_
#include "main.h"

#define VREFINT_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF80078))
#define TS_CAL1_ADDR     ((uint16_t*) ((uint32_t) 0x1FF8007A))
#define TS_CAL2_ADDR     ((uint16_t*) ((uint32_t) 0x1FF8007E))

void pollMailbox(void);
void initMailbox(UART_HandleTypeDef *hu, I2C_HandleTypeDef *hi, ADC_HandleTypeDef *ha);
#endif /* INC_MAILBOXSENSOR_H_ */
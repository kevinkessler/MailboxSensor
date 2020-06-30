/**
 *  @filename   :   MailboxSensor.c
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

#include "MailboxSensor.h"
#include "E32Lora.h"
#include "VCNL4200.h"

static UART_HandleTypeDef *_huart;
static I2C_HandleTypeDef *_hi2c;
static ADC_HandleTypeDef *_hadc;

volatile uint8_t adcConversionComplete=0;

// Todo: ADC Low power mode 
static uint16_t* readADC(uint16_t *adcCount){
    
	//Wait until ADC is fully powered up, probably not necessary
	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VREFINTRDY));
    
    uint32_t adcValues[4];

    // Open Drain on to provide ground for the Flag Sensor and Battery sensor current to flow
    HAL_GPIO_WritePin(ADC_GND_GPIO_Port, ADC_GND_Pin, GPIO_PIN_RESET);
 
    adcConversionComplete = 0;

	/*if(HAL_ADC_Start_DMA(_hadc, adcValues, 1)!=HAL_OK) {
        adcCount[0] = 0xFFFA;
        adcCount[1] = 0xFFFA;
        adcCount[2] =  0xFFFA;
        return adcCount;
    }
    uint32_t timeoutCounter = 0;
    uint8_t timeoutError = 0;
    // At 2.097Mhz, each loop takes about 27 uS and the ADC normally completes in 300uS
    // Timeout occurs in about 140mS
	while(!adcConversionComplete)
        if(++timeoutCounter > 500000){
            timeoutError = 1;
            break;
        };*/

    uint8_t timeoutError=0;
    HAL_ADC_Start(_hadc);
    HAL_ADC_PollForConversion(_hadc,10000);
    adcValues[0] = HAL_ADC_GetValue(_hadc);

    HAL_ADC_PollForConversion(_hadc,1000);
    adcValues[2] = HAL_ADC_GetValue(_hadc);

    HAL_ADC_PollForConversion(_hadc,1000);
    adcValues[3] = HAL_ADC_GetValue(_hadc);


    if(timeoutError){
        adcCount[0] = 0xFFFF;
        adcCount[1] = 0xFFFF;
        adcCount[2] =  0xFFFF;

    } else {
        //adcCount[0] = adcValues[3] & 0xff;
        //adcCount[1] = adcValues[3] >> 8;
        
        uint16_t batt = (3000 * (*VREFINT_CAL_ADDR))/adcValues[2];
    
        // Send battery Voltage in hundredths of V, with 2V subtracted off to keep it in one byte. Actual Voltage = adcCount[1] / 100 + 2V
        // External battey is measure through a resistor divider so the uC ADC gets 1/2 the battery voltage
        // Each adc Count  = battery voltage /4096 (max 2^12) * total counts on the adc. This returns the value in .001V steps
        // Divide that by 10 to get .01V steps and multiply by 2 to get the actual voltage because of the divider, so divide by 5.
        // divide by 4096 is the same as >>12, and the +5 is crude rounding, subtract 150 to create a measureable voltage range of 
        // 2V to 4.55V, a range you would expect a Li Ion battery to be in.
        adcCount[1] = ((((batt * adcValues[0]) >> 12) ) / 5);
        if (adcCount[1] > 200)
            adcCount[1] -= 200;
        else
            adcCount[1] = 0;
        
        

        // Scale the temperature count to the actual battery voltage to 3 V the factory calibration occured at
        // Multiply by 1000 to avoid having to use floating point and running out of memory
        // and calculate slope at 1000X as well. Since I'm sending the value in .5 degree increments,
        // just divide by 500
        // Actual temperature = adcCount[2] / 2 - 30
        uint16_t adjTempCount = (1000*batt*adcValues[3])/(3000000);
        uint32_t slope = (1000 * (130 - 30))/(*(TS_CAL2_ADDR)- *(TS_CAL1_ADDR));
        int16_t temp = ((slope * (adjTempCount-*(TS_CAL1_ADDR))) +30000) / 500;

        // -30 in 1/2 degrees
        
        if(temp < -60)
        {
            temp=0;
        } else if(temp > 195) {
        // 97.5 in 1/2 degree increments
            temp=0xfE;
        } else {
        //Make the number unsigned
            temp+=60;
        }
        adcCount[2] =  (uint16_t) temp; 
    }

	return adcCount;
}

void pollMailbox() {
    uint16_t adcCount[3];
    VCNL4200_initialize(_hi2c);
    VCNL4200_WriteRegister(VCNL4200_PS_CONF3_MS_REG,0b01001100,0b00000000);
    
    // Read voltage does not take long enough for the proximity sensor to complete, so a delay has to be added
    HAL_Delay(200);
    E32_SetMode(NORMAL_MODE);
    readADC(adcCount);

    //uint16_t proximity_count = 6;
    uint16_t proximity_count = VCNL4200_getProximity();
    
    // Shutdown
    //VCNL4200_WriteRegister(VCNL4200_ALS_CONF_REG,0b00000001, 0b00000000);
    VCNL4200_WriteRegister(VCNL4200_PS_CONF1_CONF2_REG,0b01001001,0b00000000); 
    //VCNL4200_WriteRegister(VCNL4200_PS_CONF3_MS_REG,0b00000000,0b00000000);

    uint8_t buffer[4];
    buffer[0] = proximity_count & 0xff;
    buffer[1] = (uint8_t)(proximity_count  >> 8);
    buffer[2] = (adcCount[1] & 0xff);
    buffer[3] = (adcCount[2] & 0xff);
   
    E32_Transmit(buffer,4);

    E32_SetMode(SLEEP_MODE);

    HAL_PWREx_EnableUltraLowPower();
    //HAL_PWREx_EnableFastWakeUp();

    if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
    HAL_PWR_EnterSTANDBYMode();
}

void initMailbox(UART_HandleTypeDef *hu, I2C_HandleTypeDef *hi, ADC_HandleTypeDef *ha) {
    _huart = hu;
    _hi2c = hi;
    _hadc = ha;

    
    E32_Init(M0_GPIO_Port, M0_Pin, M1_GPIO_Port, M1_Pin, AUX_GPIO_Port, AUX_Pin, _huart);

    E32_SetTargetAddress(0x00);
	E32_SetTargetChannel(0xf);
	/*uint8_t config[]={0xc0,0x00,0x02,0x1b,0x0f,0xC6};
	E32_SetConfig(config);
	E32_SetParams();
    E32_SetMode(NORMAL_MODE);
    E32_Reset(); */


    // Add delay to allow the E32 module to power up and to wait for the door to comply close
    // Powering up the E32 run at 13.5mA for about 2 seconds, and then setting E32 to sleep drops the 
    // current consumption to .5 mA for the remaining 2 seconds of the delay
    E32_SetMode(SLEEP_MODE);
    HAL_Delay(4000);
	
}
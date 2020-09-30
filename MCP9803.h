 /*
 * mcp9803.h
 *
 *  Created on: 30.09.2020
 *      Author: SimpleMethod
 *
 *Copyright 2020 SimpleMethod
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy of
 *this software and associated documentation files (the "Software"), to deal in
 *the Software without restriction, including without limitation the rights to
 *use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *of the Software, and to permit persons to whom the Software is furnished to do
 *so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *THE SOFTWARE.
 ******************************************************************************
 */


#ifndef INC_MCP9803_H_
#define INC_MCP9803_H_

#include "main.h"

/*!
 * ADDRESS REGISTER
 */

#define MCP9803_DEFAULT_ADDRESS 0x48<<1
#define MCP9803_CONFIG_REGISTER 0x01 //00000001U
#define MCP9803_AMBIENT_TEMPERATURE_REGISTER 0x00 //00000000U
#define MCP9803_HYSTERESIS_REGISTER 0x02 //00000010U
#define MCP9803_LIMIT_SET_REGISTER 0x03 //00000010U
/*!
 *  CONFIG REGISTER
 */
#define MCP9803_ONE_SHOT_ENABLED 0x80
#define MCP9803_ONE_SHOT_DISABLED 0x00

#define MCP9803_FAULT_QUEUE_1 0x00
#define MCP9803_FAULT_QUEUE_2 0x08
#define MCP9803_FAULT_QUEUE_4 0x10
#define MCP9803_FAULT_QUEUE_6 0x18

#define MCP9803_ADC_RESOLUTION_9_BIT 0x00
#define MCP9803_ADC_RESOLUTION_10_BIT 0x20
#define MCP9803_ADC_RESOLUTION_11_BIT 0x40
#define MCP9803_ADC_RESOLUTION_12_BIT 0x60

#define MCP9803_ALERT_POLARITY_ENABLED 0x04
#define MCP9803_ALERT_POLARITY_DISABLED 0x00

#define MCP9803_COMPARATOR_MODE 0x00
#define MCP9803_INTERPUT_MODE 0x02

#define MCP9803_SHUTDOWN_ENABLE 0x01
#define MCP9803_SHUTDOWN_DISABLE 0x00


typedef enum
{
	ADC_RESOLUTION_9_BIT    = 0x00,//!< ADC_RESOLUTION_9_BIT
	ADC_RESOLUTION_10_BIT   = 0x20,//!< ADC_RESOLUTION_10_BIT
	ADC_RESOLUTION_11_BIT   = 0x40,//!< ADC_RESOLUTION_11_BIT
	ADC_RESOLUTION_12_BIT  	 = 0x60//!< ADC_RESOLUTION_12_BIT
} ADCResolution_t;

typedef struct {
	I2C_HandleTypeDef* i2c_handle;
	uint8_t device_address;
	uint8_t payload[2];
	uint8_t config;
} MCP9803_HandleTypeDef;


MCP9803_HandleTypeDef mcp9803_Init(I2C_HandleTypeDef* i2c_handle, uint8_t device_address, uint8_t comp_int_mode, uint8_t alert_polarity, uint8_t fault_queue, ADCResolution_t adc_resolution, uint8_t one_shot_mode);
uint8_t mcp9803_Read8Bit(MCP9803_HandleTypeDef *mcp9803, uint8_t memory_address);
uint8_t mcp9803_Write8Bit(MCP9803_HandleTypeDef *mcp9803, uint8_t memory_address, uint8_t data);
uint8_t mcp9803_ReadRawTemperature(MCP9803_HandleTypeDef *mcp9803);
uint8_t MCP9803_ReadLimitSetRegister(MCP9803_HandleTypeDef *mcp9803);
uint8_t MCP9803_ReadHysteriesRegister(MCP9803_HandleTypeDef *mcp9803);
float MCP9803_TakeOneShotMeasurement(MCP9803_HandleTypeDef *mcp9803);
float mcp9803_GetTemperature(MCP9803_HandleTypeDef *mcp9803);
void MCP9803_GetShutdownMode(MCP9803_HandleTypeDef *mcp9803);
void MCP9803_SetADCResolution(MCP9803_HandleTypeDef *mcp9803, ADCResolution_t ADCResolution);
void MCP9803_WriteHysteriesRegister(MCP9803_HandleTypeDef *mcp9803, uint8_t data);
void mcp9803_LoadConfig(MCP9803_HandleTypeDef *mcp9803);
void MCP9803_WriteLimitSetRegister(MCP9803_HandleTypeDef *mcp9803, uint8_t data);
#endif /* INC_MCP9803_H_ */

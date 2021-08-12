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

#include "MCP9803.h"

/*!
 * Main configuration of the IC.
 * @param i2c_handle  Handle for I2C protocol.
 * @param device_address Address of the IC. Default is 0x48.
 * @param comp_int_mode Comparator or Interrupt mode.
 * @param alert_polarity Alert Polarity.
 * @param fault_queue Fault Queue.
 * @param adc_resolution ADC resolution. It takes 30 ms (typical) for 9-bit data and 240 ms (typical) for 12-bit data.
 * @param one_shot_mode One-shot mode performs a single temperature measurement and returns to deep sleep mode.
 * @return
 */
MCP9803_HandleTypeDef mcp9803_Init(I2C_HandleTypeDef* i2c_handle, uint8_t device_address, uint8_t comp_int_mode, uint8_t alert_polarity, uint8_t fault_queue, ADCResolution_t adc_resolution, uint8_t one_shot_mode)
{
	uint8_t config=comp_int_mode|alert_polarity|fault_queue|adc_resolution|one_shot_mode;
	MCP9803_HandleTypeDef mcp9803;
	mcp9803.i2c_handle = i2c_handle;
	mcp9803.device_address = device_address;
	mcp9803.config=config;
	return mcp9803;
}

/*!
 * Reading 8 bits from internal memory.
 * @param mcp9803 Handle do main struct.
 * @param memory_address Internal memory address.
 * @return Read 8 bits from internal memory.
 */
uint8_t mcp9803_Read8Bit(MCP9803_HandleTypeDef *mcp9803, uint8_t memory_address)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(mcp9803->i2c_handle, mcp9803->device_address, memory_address, 0x00000001U, &tmp, 1, HAL_MAX_DELAY);
	return tmp;
}
/*!
 * Sending 8 bits to internal memory.
 * @param mcp9803 Handle do main struct.
 * @param memory_address Internal memory address.
 * @param data Data to be sent into IC.
 * @return Status of operation. Look at HAL_OK.
 */
uint8_t mcp9803_Write8Bit(MCP9803_HandleTypeDef *mcp9803, uint8_t memory_address, uint8_t data)
{
	return HAL_I2C_Mem_Write(mcp9803->i2c_handle, mcp9803->device_address, memory_address, 0x00000001U, &data, 1, HAL_MAX_DELAY);
}
/*!
 * Loading the configuration into IC.
 * @param mcp9803 Handle do main struct.
 */
void mcp9803_LoadConfig(MCP9803_HandleTypeDef *mcp9803)
{
	 HAL_I2C_Mem_Write(mcp9803->i2c_handle, mcp9803->device_address, MCP9803_CONFIG_REGISTER, 0x00000001U, &mcp9803->config, 0x00000001U, HAL_MAX_DELAY);
}
/*!
 * Status of data reading. Function debugging a connection.
 * @param mcp9803 Handle do main struct.
 * @return Status of data reading. Look at HAL_OK.
 */
uint8_t mcp9803_ReadRawTemperature(MCP9803_HandleTypeDef *mcp9803)
{
	uint8_t data[2];
	uint8_t hal_status=HAL_I2C_Mem_Read(mcp9803->i2c_handle, mcp9803->device_address, MCP9803_AMBIENT_TEMPERATURE_REGISTER, 0x00000001U, data, 2, HAL_MAX_DELAY);
	mcp9803->payload[0]=data[0];
	mcp9803->payload[1]=data[1];
	return hal_status;
}
/*!
 * Returns the current temperature;
 * @param mcp9803 Handle do main struct.
 * @return Temperature as float value.
 */
float mcp9803_GetTemperature(MCP9803_HandleTypeDef *mcp9803)
{
	if(mcp9803_ReadRawTemperature(mcp9803)==HAL_OK)
	{
		uint16_t temp = (mcp9803->payload[0] * 0x100 + (mcp9803->payload[1] & 0xF0));
		 if (temp & 0b1000000000000000) {
			 temp = (~temp) + 16;
			 return ((temp >> 4)*(-0.0625));
		 }
		 else
		 {
			return ((temp >> 4)*(0.0625));
		 }
	}
	return -1;
}
/*!
 * Switching to deep sleep mode.
 * @param mcp9803 Handle do main struct.
 */
void MCP9803_GetShutdownMode(MCP9803_HandleTypeDef *mcp9803)
{
	mcp9803_Write8Bit(mcp9803,MCP9803_CONFIG_REGISTER,MCP9803_SHUTDOWN_ENABLE);
}
/*!
 *  Sets a new mode of measurement resolution.
 * @param mcp9803 Handle do main struct.
 * @param adc_resolution Enum with new resolution.
 */
void MCP9803_SetADCResolution(MCP9803_HandleTypeDef *mcp9803, ADCResolution_t adc_resolution)
{
	mcp9803_Write8Bit(mcp9803,MCP9803_CONFIG_REGISTER,adc_resolution);
}
/*!
 * Triggers a one-time measurement and switches to deep sleep mode.
 * @param mcp9803 Handle do main struct.
 * @return Read data from register.
 */
float MCP9803_TakeOneShotMeasurement(MCP9803_HandleTypeDef *mcp9803)
{
	mcp9803_Write8Bit(mcp9803,MCP9803_CONFIG_REGISTER,mcp9803->config|MCP9803_CONFIG_REGISTER);
	HAL_Delay(2);
	return mcp9803_GetTemperature(mcp9803);
}
/*!
 *  Reads limit temperature from the register.
 * @param mcp9803 Handle do main struct.
 * @return Read data from register.
 */
uint8_t MCP9803_ReadHysteriesRegister(MCP9803_HandleTypeDef *mcp9803)
{
	return mcp9803_Read8Bit(mcp9803,MCP9803_HYSTERESIS_REGISTER);
}
/*!
 *  Sends data to register used to trigger an interruption when a temperature oscillates around sets temperature. Only most significant bit data is required.
 * @param mcp9803 Handle do main struct.
 * @param data Temperature written on 8 bit.
 */
void MCP9803_WriteHysteriesRegister(MCP9803_HandleTypeDef *mcp9803, uint8_t data)
{
	 mcp9803_Write8Bit(mcp9803,MCP9803_HYSTERESIS_REGISTER,data);
}
/*!
 *  Reads limit temperature from the register.
 * @param mcp9803 Handle do main struct.
 * @return Read data from register.
 */
uint8_t MCP9803_ReadLimitSetRegister(MCP9803_HandleTypeDef *mcp9803)
{
return	 mcp9803_Read8Bit(mcp9803,MCP9803_LIMIT_SET_REGISTER);
}
/*!
 *  Sends data to register used to trigger an interruption when a temperature is exceeded. Only most significant bit data is required.
 * @param mcp9803 Handle do main struct.
 * @param data Temperature written on 8 bit.
 */
void MCP9803_WriteLimitSetRegister(MCP9803_HandleTypeDef *mcp9803, uint8_t data)
{
	 mcp9803_Write8Bit(mcp9803,MCP9803_LIMIT_SET_REGISTER,data);
}

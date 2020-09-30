# MCP9800 1/2/3

## STM32 library for high-accuracy temperature sensor using HAL and I2C.

![proof of concept](https://raw.githubusercontent.com/SimpleMethod/STM32-MCP9803/master/doc/PoC.png)

# Example:

    /* USER CODE BEGIN 2 */
    mcp9803 = mcp9803_Init(&hi2c2, MCP9803_DEFAULT_ADDRESS, MCP9803_COMPARATOR_MODE,
                           MCP9803_ALERT_POLARITY_DISABLED, MCP9803_FAULT_QUEUE_1,
                           ADC_RESOLUTION_12_BIT, MCP9803_ONE_SHOT_DISABLED);
    mcp9803_LoadConfig(&mcp9803);
    /* USER CODE END 2 */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
      printf("Temperature: %d \r\n", (int)mcp9803_GetTemperature(&mcp9803));
      HAL_Delay(2000);
      /* USER CODE END WHILE */
      /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */

# Successfully tested on:
- [x] H723ZG
- [x] H743ZI
- [x] F767ZI
- [x] F411RE
- [x] G431KB

# Address register:

    MCP9803_DEFAULT_ADDRESS 0x48<<1
    MCP9803_CONFIG_REGISTER 0x01 //00000001U
    MCP9803_AMBIENT_TEMPERATURE_REGISTER 0x00 //00000000U
    MCP9803_HYSTERESIS_REGISTER 0x02 //00000010U
    MCP9803_LIMIT_SET_REGISTER 0x03 //00000010U

#  Config register:

    MCP9803_ONE_SHOT_ENABLED 0x80
    MCP9803_ONE_SHOT_DISABLED 0x00
    
    MCP9803_FAULT_QUEUE_1 0x00
    MCP9803_FAULT_QUEUE_2 0x08
    MCP9803_FAULT_QUEUE_4 0x10
    MCP9803_FAULT_QUEUE_6 0x18
    
    MCP9803_ADC_RESOLUTION_9_BIT 0x00
    MCP9803_ADC_RESOLUTION_10_BIT 0x20
    MCP9803_ADC_RESOLUTION_11_BIT 0x40
    MCP9803_ADC_RESOLUTION_12_BIT 0x60
    
    MCP9803_ALERT_POLARITY_ENABLED 0x04
    MCP9803_ALERT_POLARITY_DISABLED 0x00
    
    MCP9803_COMPARATOR_MODE 0x00
    MCP9803_INTERPUT_MODE 0x02
    
    MCP9803_SHUTDOWN_ENABLE 0x01
    MCP9803_SHUTDOWN_DISABLE 0x00
    
    typedef enum
    {
    	ADC_RESOLUTION_9_BIT    = 0x00,//!< ADC_RESOLUTION_9_BIT
    	ADC_RESOLUTION_10_BIT   = 0x20,//!< ADC_RESOLUTION_10_BIT
    	ADC_RESOLUTION_11_BIT   = 0x40,//!< ADC_RESOLUTION_11_BIT
    	ADC_RESOLUTION_12_BIT  	 = 0x60//!< ADC_RESOLUTION_12_BIT
    } ADCResolution_t;

# Pins configuration:

| PERIPHERALS | MODES                 | FUNCTIONS      | PINS              |
|-------------|-----------------------|----------------|-------------------|
| I2C1        | I2C                   | I2C1_SCL       | PB8               |
| I2C1        | I2C                   | I2C1_SDA       | PB9               |
| SYS         | Trace Asynchronous Sw | SYS_JTMS-SWDIO | PA13              |
| SYS         | Trace Asynchronous Sw | SYS_JTCK-SWCLK | PA14              |
| SYS         | Trace Asynchronous Sw | SYS_JTDO-SWO   | PB3               |
| SYS         | SysTick               | SYS_VS_Systick | VP_SYS_VS_Systick |

### **For using printf with float should be add flag -u _printf_float in C compiler**

![enter image description here](https://raw.githubusercontent.com/SimpleMethod/STM32-AM2320/master/Images/am2320_Atolic_TrueSTUDIO.png)

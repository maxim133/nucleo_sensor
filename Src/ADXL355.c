/**
  ******************************************************************************
  * @file           ADXL355.c
  * @brief          ADXL355 driver
  ******************************************************************************
  */

/***************************** Include Files **********************************/
#include <stdio.h>

#include "ADXL355.h"
#include "spi_communication.h"

/****************************** Global Data ***********************************/

/**
   @brief Schematic spi chip select
**/
CSPinDesc_t ADXL355_CS_pins [ADXL355_COUNT] =
{
	{0, GPIOA, GPIO_PIN_4},
	{0, GPIOC, GPIO_PIN_10},
	{0, GPIOC, GPIO_PIN_11},
	{0, GPIOC, GPIO_PIN_12},
	{0, GPIOF, GPIO_PIN_6},
	{0, GPIOF, GPIO_PIN_7},
	{0, GPIOA, GPIO_PIN_15},
	{0, GPIOC, GPIO_PIN_2},
	{0, GPIOC, GPIO_PIN_3},
	{0, GPIOD, GPIO_PIN_4},
	{0, GPIOD, GPIO_PIN_5},
	{0, GPIOD, GPIO_PIN_6},
	{0, GPIOD, GPIO_PIN_7},
	{0, GPIOE, GPIO_PIN_3},
	{0, GPIOF, GPIO_PIN_1},
	{0, GPIOF, GPIO_PIN_0},
	{0, GPIOG, GPIO_PIN_0},
	{0, GPIOE, GPIO_PIN_1},
	{0, GPIOG, GPIO_PIN_9},
	{0, GPIOG, GPIO_PIN_12},
};

/************************* Global scope functions *****************************/
/**
   @brief Turns on accelerometer measurement mode.

   @return none

**/
void ADXL355_Start_Sensor(void)
{
	uint8_t ui8temp;

	for (int channel = 0; channel < ADXL355_COUNT; channel++)
	{
		SPI_ReadData(ADXL355, &ADXL355_CS_pins[channel], POWER_CTL, &ui8temp, 1);       /* Read POWER_CTL register, before modifying it */

		ui8temp = ui8temp & 0xFE;                                          /* Set measurement bit in POWER_CTL register */

		SPI_WriteData(ADXL355, &ADXL355_CS_pins[channel], POWER_CTL, ui8temp);                    /* Write the new value to POWER_CTL register */
	}
}

/**
   @brief Reset the accelerometer sensor

   @param cs Chip select number

   @return none

**/
void ADXL355_Reset(CSPinDesc_t *cs)
{
	SPI_WriteData(ADXL355, cs, RESET_G, POR_RESET);
	HAL_Delay(10UL);
}

/**
   @brief Initialization the accelerometer sensor

   @return none

**/
void ADXL355_Init(void)
{
	uint8_t devid;

	for (int channel = 0; channel < ADXL355_COUNT; channel++)
	{
		ADXL355_CS_pins[channel].spi_channel = &hspi3;
	}

   /* Quick verification test for boards */
	for (int channel = 0; channel < ADXL355_COUNT; channel++)
	{
		ADXL355_Reset(&ADXL355_CS_pins[channel]);

		SPI_ReadData(ADXL355, &ADXL355_CS_pins[channel], DEVID_AD, &devid, 1);

#if ADXL_RANGE == 2
		SPI_WriteData(ADXL355, &ADXL355_CS_pins[channel],  RANGE, 0x81);          /* Set sensor range within RANGE register */
#endif

#if ADXL_RANGE == 4
		SPI_WriteData(ADXL355, ADXL355_CS_pins[channel],  RANGE, 0x82);          /* Set sensor range within RANGE register */
#endif

#if ADXL_RANGE == 8
		SPI_WriteData(ADXL355, ADXL355_CS_pins[channel],  RANGE, 0x83);          /* Set sensor range within RANGE register */
#endif
	}

	ADXL355_Start_Sensor();
}

/**
   @brief Puts the accelerometer into standby mode.

   @return none

**/
void ADXL355_Stop_Sensor(void)
{
	uint8_t ui8temp;

	for (int channel = 0; channel < ADXL355_COUNT; channel++)
	{
		SPI_ReadData(ADXL355, &ADXL355_CS_pins[channel], POWER_CTL, &ui8temp, 1);        /*Read POWER_CTL register, before modifying it */

		ui8temp = ui8temp | 0x01;                                      /* Clear measurement bit in POWER_CTL register */

		SPI_WriteData(ADXL355, &ADXL355_CS_pins[channel], POWER_CTL, ui8temp);                 /* Write the new value to POWER_CTL register */
	}
}

/**
   @brief Convert the two's complement data in X,Y,Z registers to signed integers

   @param ui32SensorData - raw data from register

   @return int32_t - signed integer data

**/
int32_t ADXL355_Acceleration_Data_Conversion (uint32_t ui32SensorData)
{
   int32_t volatile i32Conversion = 0;

   ui32SensorData = (ui32SensorData  >> 4);
   ui32SensorData = (ui32SensorData & 0x000FFFFF);

   if((ui32SensorData & 0x00080000)  == 0x00080000){

         i32Conversion = (ui32SensorData | 0xFFF00000);

   }
   else{
         i32Conversion = ui32SensorData;
   }

   return i32Conversion;
}

/**
   @brief Reads the accelerometer data.

   @param self Data structure

   @param count Devices count

   @return none

**/
void ADXL355_Data_Scan(ADXL355Device *self, uint8_t count)
{
	uint8_t buffer[3];

	for (uint8_t channel = 0; channel < count; channel++)
	{
		SPI_ReadData(ADXL355, &ADXL355_CS_pins[0], XDATA3, buffer, 3);
		self[channel].X = ((buffer[0] << 16) | (buffer[1] << 8) | buffer[2]);
		SPI_ReadData(ADXL355, &ADXL355_CS_pins[0], YDATA3, buffer, 3);
		self[channel].Y = ((buffer[0] << 16) | (buffer[1] << 8) | buffer[2]);
		SPI_ReadData(ADXL355, &ADXL355_CS_pins[0], ZDATA3, buffer, 3);
		self[channel].Z = ((buffer[0] << 16) | (buffer[1] << 8) | buffer[2]);
		SPI_ReadData(ADXL355, &ADXL355_CS_pins[0], TEMP2, buffer, 2);
		self[channel].T = ((buffer[0] << 8) | buffer[1]);

		// for debug
//		float temp = ((((float)self[channel].T - ADXL355_TEMP_BIAS)) / ADXL355_TEMP_SLOPE) + 25.0;
	}
}

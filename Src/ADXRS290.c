/**
  ******************************************************************************
  * @file           ADXRS290.c
  * @brief          ADXRS290 driver
  ******************************************************************************
*/

/***************************** Include Files **********************************/
#include "spi_communication.h"
#include "ADXRS290.h"
#include "stdbool.h"

/****************************** Global Data ***********************************/

/**
   @brief Schematic spi chip select
**/
CSPinDesc_t ADXRS290_CS_pins[ADXRS290_COUNT] =
{
	{0 ,GPIOD, GPIO_PIN_14},
	{0, GPIOC, GPIO_PIN_6},
	{0, GPIOB, GPIO_PIN_15},
	{0, GPIOB, GPIO_PIN_8},
};

/************************* Global scope functions *****************************/

/**
   @brief Schematic selector routine

   @param number A number (see schematic)

   @param select Select A or not
**/
void selectA(uint16_t number, bool select)
{
	if (select == true)
		GPIOD->BSRR = number;
	else
		GPIOD->BSRR = (uint32_t)number << 16;
}

/**
   @brief Initialization the gyro sensor

   @return none

**/
void ADXRS290_Init(void)
{
	uint8_t devid;

	ADXRS290_CS_pins[0].spi_channel = &hspi1;
	ADXRS290_CS_pins[1].spi_channel = &hspi1;
	ADXRS290_CS_pins[2].spi_channel = &hspi4;
	ADXRS290_CS_pins[3].spi_channel = &hspi1;

	for (int channel = 0; channel < ADXRS290_COUNT; channel++)
	{
		for (int A = 0; A < ADXRS290_A_COUNT; A++)
		{
			selectA(A, true);
			SPI_ReadData(ADXRS290, &ADXRS290_CS_pins[channel], ADXRS290_DEV_ID, &devid, 1);

			if (devid != 0x92)
				HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);

			/*Set measurement mode. Temperature sensor is enable*/
			SPI_WriteData(ADXRS290, &ADXRS290_CS_pins[channel], ADXRS290_POW_CTRL_REG, ADXRS290_POW_CTRL_STDBY_MASK);
			SPI_ReadData(ADXRS290, &ADXRS290_CS_pins[channel], ADXRS290_POW_CTRL_REG, &devid, 1);
			selectA(A, false);
		}
	}

}

/**
   @brief Reads the gyro data.

   @param self Data structure

   @param count Devices count

   @return none

**/
void ADXRS290_Data_Scan(ADXRS290Device *self, uint8_t count)
{
	uint8_t buffer[4];
	int16_t i16x, i16y;
	uint16_t data = 0;

	for (uint8_t channel = 0; channel < count; channel++)
	{
		for (uint8_t A = 0; A < ADXRS290_A_COUNT; A++)
		{
			uint8_t index = channel * ADXRS290_A_COUNT + A;

			selectA(A, true);
			SPI_ReadData(ADXRS290, &ADXRS290_CS_pins[channel], ADXRS290_TEMP_L, buffer, 2);

			int16_t temp = ((buffer[1] << 8) | buffer[0]);

			self[index].T = temp;

			SPI_ReadData(ADXRS290, &ADXRS290_CS_pins[channel], ADXRS290_GYR_X_L, buffer, 4);
			selectA(A, false);

			data = buffer[3];
			data |= (0xFF & buffer[2]) << 8; // MSB
			i16x = (int16_t)data;

			self[index].X = data;

			data = buffer[1];
			data |= (0xFF & buffer[0]) << 8; // MSB
			i16y = (int16_t)data;

			self[index].Y = data;
		}
	}
}

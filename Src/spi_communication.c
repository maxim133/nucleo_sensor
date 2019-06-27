/*
 * spi_communication.c
 *
 *  Created on: 2 θών. 2019 γ.
 *      Author: Petrov
 */

#include "stm32f7xx_hal.h"
#include "spi_communication.h"

void ChipSelect(CSPinDesc_t *cs)
{
	HAL_GPIO_TogglePin(cs->cs_gpio, cs->cs_pin);
}

void SPI_WriteData(Channel ch, CSPinDesc_t *cs, uint8_t ui8address, uint8_t ui8Data)
{
	uint8_t ui8writeAddress;

	if (ch == ADXL355)
	{
		ui8writeAddress = ((ui8address <<1) | SPI_WRITE);
	}
	else if (ch == ADXRS290)
	{
		ui8writeAddress = ui8address | (SPI_WRITE << 7);
	}

	ChipSelect(cs);

	HAL_SPI_Transmit(cs->spi_channel, &ui8writeAddress, 1, 1);
	HAL_SPI_Transmit(cs->spi_channel, &ui8Data, 1, 1);

	ChipSelect(cs);
}

void SPI_ReadData(Channel ch, CSPinDesc_t *cs, uint8_t ui8address, uint8_t *buffer, uint8_t size)
{
	uint8_t DummyBite = 0xAA;
	uint8_t ui8writeAddress;

	if (ch == ADXL355)
	{
		ui8writeAddress = ((ui8address <<1) | SPI_READ);
	}
	else if (ch == ADXRS290)
	{
		ui8writeAddress = ui8address | (SPI_READ << 7);
	}

	ChipSelect(cs);

	HAL_SPI_Transmit(cs->spi_channel, &ui8writeAddress, 1, 1);

	HAL_SPI_TransmitReceive(cs->spi_channel, &DummyBite, buffer, size, 2);

	ChipSelect(cs);
}

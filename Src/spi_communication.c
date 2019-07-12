/**
  ******************************************************************************
  * @file           spi_communication.c
  * @brief          SPI HAL
  ******************************************************************************
*/

#include "stm32f7xx_hal.h"
#include "spi_communication.h"

/**
   @brief SPI Chip Select

   @param cs A Number of CS pin

   @param PinState CS turn on/off
**/
void ChipSelect(CSPinDesc_t *cs, GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(cs->cs_gpio, cs->cs_pin, PinState);
}

/**
   @brief Write data to the SPI channel

   @param ch SPI channel

   @param cs A Number of CS pin

   @param address Register address

   @param data Register data
**/
void SPI_WriteData(Channel ch, CSPinDesc_t *cs, uint8_t address, uint8_t data)
{
	uint8_t ui8writeAddress;

	if (ch == ADXL355)
	{
		ui8writeAddress = ((address <<1) | SPI_WRITE);
	}
	else if (ch == ADXRS290)
	{
		ui8writeAddress = address | (SPI_WRITE << 7);
	}

	ChipSelect(cs, GPIO_PIN_RESET);

	HAL_SPI_Transmit(cs->spi_channel, &ui8writeAddress, 1, 3);
	HAL_SPI_Transmit(cs->spi_channel, &data, 1, 3);

	ChipSelect(cs, GPIO_PIN_SET);
}

/**
   @brief Read data from the SPI channel

   @param ch SPI channel

   @param cs A Number of CS pin

   @param address Register address

   @param data Pointer to the buffer

   @param size Buffer size
**/
void SPI_ReadData(Channel ch, CSPinDesc_t *cs, uint8_t address, uint8_t *data, uint8_t size)
{
	uint8_t DummyBite = 0xAA;
	uint8_t ui8writeAddress;

	if (ch == ADXL355)
	{
		ui8writeAddress = ((address <<1) | SPI_READ);
	}
	else if (ch == ADXRS290)
	{
		ui8writeAddress = address | (SPI_READ << 7);
	}

	ChipSelect(cs, GPIO_PIN_RESET);

	HAL_SPI_Transmit(cs->spi_channel, &ui8writeAddress, 1, 3);

	HAL_SPI_TransmitReceive(cs->spi_channel, &DummyBite, data, size, 3);

	ChipSelect(cs, GPIO_PIN_SET);
}

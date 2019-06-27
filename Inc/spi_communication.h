/*
 * spi_communication.h
 *
 *  Created on: 2 θών. 2019 γ.
 *      Author: Petrov
 */

#ifndef SPI_COMMUNICATION_H_
#define SPI_COMMUNICATION_H_

#include "main.h"
#include "pin_desc_t.h"

typedef enum
{
	ADXL355 = 1,
	ADXRS290,
} Channel;

/* Accelerometer write command */
#define SPI_WRITE         0x0

/* Accelerometer read command */
#define SPI_READ          0x1

void SPI_ReadData(Channel ch, CSPinDesc_t *cs, uint8_t ui8address, uint8_t *buffer, uint8_t size);
void SPI_WriteData(Channel ch, CSPinDesc_t *cs, uint8_t ui8address, uint8_t ui8Data);

#endif /* SPI_COMMUNICATION_H_ */

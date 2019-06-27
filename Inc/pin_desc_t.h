#ifndef __PINDESC_T_H__
#define __PINDESC_T_H__

#include "stm32f7xx_hal.h"

typedef struct
{
	SPI_HandleTypeDef *spi_channel;
	GPIO_TypeDef* cs_gpio;
	uint16_t cs_pin;
} CSPinDesc_t;

#endif

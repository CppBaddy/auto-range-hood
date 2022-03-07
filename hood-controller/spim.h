/*
 * spim.h
 *
 *  Created on: 2020-08-21
 *      Author: yulay
 */

#ifndef SPIM_H_
#define SPIM_H_

#include <avr/io.h>
#include "PortConfig.h"


inline void setCSN()
{
    SPI_PORT &= ~_BV(SPI_CSN); //activate CSN
}

inline void clearCSN()
{
    SPI_PORT |= _BV(SPI_CSN); //deactivate CSN
}

void SPI_Init();

uint8_t SPI_Transfer(uint8_t v);


#endif /* SPIM_H_ */

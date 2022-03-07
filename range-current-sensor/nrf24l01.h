/*
 * nrf24l01.h
 *
 *  Created on: 2020-08-21
 *      Author: yulay
 */

#ifndef NRF24L01_H_
#define NRF24L01_H_

#include <stdbool.h>
#include <avr/io.h>

#include "PortConfig.h"

inline void setCE()
{
    NRF_PORT |= _BV(NRF_CE); //activate CE
}

inline void clearCE()
{
    NRF_PORT &= ~_BV(NRF_CE); //deactivate CE
}


void nrf24l01_Init();

uint8_t registerRead(uint8_t addr);

uint8_t registerWrite(uint8_t addr, uint8_t v);
uint8_t registerWriteS(uint8_t addr, const uint8_t* data, uint8_t size);

bool nrf24l01_Send( const uint8_t* buffer, uint8_t size);


#endif /* NRF24L01_H_ */

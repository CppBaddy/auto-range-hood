/*
 * spim.c
 *
 *  Created on: 2020-08-21
 *      Author: yulay
 */
#include <stdbool.h>
#include "spim.h"

// hardware assisted SPI: MISO != DI && MOSI != DO
// need to cross wires to make this work
//void SPI_Init()
//{
//    SPI_DDR |= _BV(SPI_SCK) | _BV(SPI_MOSI) | _BV(SPI_CSN); //outputs: SCK, MOSI, CSN
//    SPI_PORT |= _BV(SPI_CSN) | _BV(SPI_MISO); //level high: CSN and enable pull-ups: MISO
//
//    USICR = _BV(USIWM0); //Three-wire mode. Uses DO, DI and USCK pins
//}
//
//uint8_t SPI_Transfer(uint8_t v)
//{
//    USIDR = v;
//    USISR = _BV(USIOIF); //clear overflow flag
//
//    do
//    {
//        USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK) | _BV(USITC);
//    }
//    while(0 == (USISR & _BV(USIOIF)));
//
//    return USIDR;
//}

void SPI_Init()
{
    SPI_DDR |= _BV(SPI_SCK) | _BV(SPI_MOSI) | _BV(SPI_CSN); //outputs: SCK, MOSI, CSN
    SPI_PORT |= _BV(SPI_CSN) | _BV(SPI_MISO); //level high: CSN and enable pull-ups: MISO
}

//doing bit-banging solution in order to keep PCBs as is, ie using MISO/MOSI pins

inline void sckUp()
{
    SPI_PORT |= _BV(SPI_SCK);
}
inline void sckDown()
{
    SPI_PORT &= ~_BV(SPI_SCK);
}

inline void setMOSI(bool b)
{
    if(b)
    {
        SPI_PORT |= _BV(SPI_MOSI);
    }
    else
    {
        SPI_PORT &= ~_BV(SPI_MOSI);
    }
}

inline bool getMISO()
{
    return (SPI_PIN & _BV(SPI_MISO));
}

uint8_t SPI_Transfer(uint8_t v)
{
    sckDown();

    uint8_t di = 0;
    for(uint8_t i=0; i < 8; ++i)
    {
        setMOSI(0x80 & v);

        v <<= 1;
        di <<= 1;

        sckUp();

        if(getMISO())
        {
            di |= 1;
        }

        sckDown();
    }

    return di;
}

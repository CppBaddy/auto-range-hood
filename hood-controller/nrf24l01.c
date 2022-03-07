/*
 * nrf24l01.c
 *
 *  Created on: 2020-08-21
 *      Author: yulay
 */
#include <util/delay.h>

#include "spim.h"
#include "nrf24l01.h"
#include "nrf24l01_config.h"

/*

Running on internal 8 MHz clock

CPU Clock Freq = 1 MHz
Main prescaler = 8

PA0 - input IRQ from nRF24L01+

PA3 - CE    chip enable output

*/

void nrf24l01_Init()
{
    SPI_Init(); //USI - SPI Master

    NRF_DDR |= _BV(NRF_CE);   //output pin CE
    NRF_DDR &= ~_BV(NRF_IRQ);  //input pin nRF IRQ

    clearCE();
}

uint8_t registerWrite(uint8_t addr, uint8_t v)
{
    setCSN();

    uint8_t status = SPI_Transfer(W_REGISTER | (REGISTER_MASK & addr));
    SPI_Transfer(v);

    clearCSN();

    return status;
}

uint8_t registerWriteS(uint8_t addr, const uint8_t* data, uint8_t size)
{
    setCSN();

    uint8_t status = SPI_Transfer(W_REGISTER | (REGISTER_MASK & addr));

    for(uint8_t i=0; i < size; ++i)
    {
        SPI_Transfer(data[i]);
    }

    clearCSN();

    return status;
}

uint8_t registerRead(uint8_t addr)
{
    setCSN();

    SPI_Transfer(R_REGISTER | (REGISTER_MASK & addr));
    uint8_t result = SPI_Transfer(0);

    clearCSN();

    return result;
}

uint8_t setReceiveAddress( const uint8_t *address, uint8_t size)
{
    return registerWriteS( RX_ADDR_P0, address, size);
}

uint8_t setTransmitAddress( const uint8_t *address, uint8_t size)
{
    return registerWriteS( TX_ADDR, address, size);
}

void startListen()
{
    setCE();
}

void flushTx()
{
    setCSN();

    SPI_Transfer( FLUSH_TX);

    clearCSN();
}

void flushRx()
{
    setCSN();

    SPI_Transfer( FLUSH_RX);

    clearCSN();
}

uint8_t getRxPayloadWidth()
{
    setCSN();

    SPI_Transfer( R_RX_PL_WIDTH);

    uint8_t result = SPI_Transfer(0);

    clearCSN();

    return result;
}

uint8_t getStatus()
{
    setCSN();

    uint8_t status = SPI_Transfer( 0xff);

    clearCSN();

    return status;
}

bool isReadyToSend()
{
    uint8_t status = getStatus();

    return (status & _BV( TX_FULL)) == 0;
}

bool isDataReady()
{
    const uint8_t status = registerRead( FIFO_STATUS);

    return (status & _BV( RX_EMPTY )) == 0;
}

bool nrf24l01_Send( const uint8_t* buffer, uint8_t size)
{
    clearCE();

    setCSN();

    uint8_t status = SPI_Transfer( W_TX_PAYLOAD);

    if ( status & _BV( TX_FULL))
    {
        clearCSN();

        return false;
    }

    for(uint8_t i=0; i < size; ++i)
    {
        SPI_Transfer(buffer[i]);
    }

    clearCSN();

    setCE();

    _delay_us( 11);

    clearCE();

    return true;
}

void nrf24l01_Receive( uint8_t* buffer, uint8_t size)
{
    setCSN();

    SPI_Transfer(R_RX_PAYLOAD);

    for(uint8_t i=0; i < size; ++i)
    {
        buffer[i] = SPI_Transfer(0xff);
    }

    clearCSN();
}





















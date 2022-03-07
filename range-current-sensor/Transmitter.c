/*
 * Transmitter.c
 *
 *  Created on: 2020-08-30
 *      Author: yulay
 */

#include <util/delay.h>

#include "Transmitter.h"
#include "nrf24l01.h"
#include "nrf24l01_config.h"

#define CONFIG_TX_POWER_DOWN    (_BV(MASK_RX_DR) | _BV(MASK_MAX_RT) | _BV(EN_CRC) | _BV(CRCO))
#define CONFIG_TX_POWER_UP      (_BV(PWR_UP) | _BV(MASK_RX_DR) | _BV(MASK_MAX_RT) | _BV(EN_CRC) | _BV(CRCO))

static const uint8_t NodeAddress[] = { 'R', 'a', 'n', 'g', 'e' };

void RF_Init()
{
    nrf24l01_Init();

    _delay_ms(100);

    RF_PowerDown();

    //registerWrite(EN_AA, _BV(ENAA_P0)); //enable ACK for pipe 0
    registerWrite(EN_AA, 0); //disable ACK for all pipes

    //registerWrite(EN_RXADDR, _BV(ERX_P0)); //enable Rx addresses

    registerWrite(SETUP_AW, 3);         //address width 5 bytes

    //registerWrite(SETUP_RETR, 15);      // auto-retries
    registerWrite(SETUP_RETR, 0);      // disable auto-retransmits

    registerWrite(RF_CH, NRF_CHANNEL); // radio channel #

    //registerWrite(RF_SETUP, _BV(RF_DR_LOW) | _BV(RF_PWR_HIGH) | _BV(RF_PWR_LOW));
    registerWrite(RF_SETUP, _BV(RF_DR_LOW) | _BV(RF_PWR_LOW));

    //TODO    registerWrite(RF_STATUS, ?); //clear flags

    //registerWriteS(RX_ADDR_P0, NodeAddress, 5);
    registerWriteS(TX_ADDR, NodeAddress, 5);

    //registerWrite(RX_PW_P0, 32);         //rcv payload size in bytes
}

void RF_PowerUp()
{
    registerWrite(NRF_CONFIG, CONFIG_TX_POWER_UP); //PWR up, enable CRC, disable Rx

    _delay_ms(2); //power up duration
}

void RF_PowerDown()
{
    registerWrite(NRF_CONFIG, CONFIG_TX_POWER_DOWN); //PWR down, enable CRC, disable Rx
}

void RF_Transmit(uint8_t* data, uint8_t size)
{
    RF_PowerUp();

    nrf24l01_Send(data, size);
}

void RF_ClearIRQ()
{
    registerWrite(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
}

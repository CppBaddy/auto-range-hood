/*
 * Receiver.c
 *
 *  Created on: 2020-08-30
 *      Author: yulay
 */

#include <util/delay.h>

#include "nrf24l01.h"
#include "nrf24l01_config.h"
#include "Receiver.h"

#include "SensorModel.hpp"

#define CONFIG_RX_POWER_DOWN    (_BV(MASK_TX_DS) | _BV(MASK_MAX_RT) | _BV(EN_CRC) | _BV(CRCO))
#define CONFIG_RX_POWER_UP      (_BV(PWR_UP) | _BV(MASK_TX_DS) | _BV(MASK_MAX_RT) | _BV(EN_CRC) | _BV(CRCO))

static const uint8_t NodeAddress[] = { 'R', 'a', 'n', 'g', 'e' };

void RF_Init()
{
    nrf24l01_Init();

    _delay_ms(100);

    RF_PowerDown();

    //registerWrite(EN_AA, _BV(ENAA_P0)); //enable ACK for pipe 0
    registerWrite(EN_AA, 0); //disable ACK for all pipes

    registerWrite(EN_RXADDR, _BV(ERX_P0)/* | _BV(ERX_P1) | _BV(ERX_P2) | _BV(ERX_P3) | _BV(ERX_P4) | _BV(ERX_P5)*/); //enable Rx addresses

    registerWrite(SETUP_AW, 3);         //address width 5 bytes

    //registerWrite(SETUP_RETR, 15);      // auto-retries
    registerWrite(SETUP_RETR, 0);      // disable auto-retransmits

    registerWrite(RF_CH, NRF_CHANNEL); // radio channel #

    //registerWrite(RF_SETUP, _BV(RF_DR_LOW) | _BV(RF_PWR_HIGH) | _BV(RF_PWR_LOW));
    registerWrite(RF_SETUP, _BV(RF_DR_LOW) | _BV(RF_PWR_LOW));

    RF_ClearIRQ(); //clear flags

    registerWriteS(RX_ADDR_P0, NodeAddress, 5);
    //registerWriteS(TX_ADDR, NodeAddress, 5);

    registerWrite(RX_PW_P0, NRF_PAYLOAD_SIZE);  //rcv payload size in bytes

    RF_PowerUp();
}

void RF_PowerUp()
{
    registerWrite(NRF_CONFIG, CONFIG_RX_POWER_UP); //PWR up, enable CRC, disable Rx mode

    _delay_ms(2); //power up duration

    registerWrite(NRF_CONFIG, _BV(PRIM_RX) | CONFIG_RX_POWER_UP); //enable Rx mode

    setCE(); //going into Rx state

    _delay_us(130);
}

void RF_PowerDown()
{
    registerWrite(NRF_CONFIG, CONFIG_RX_POWER_DOWN); //PWR down, enable CRC, disable Rx
}

void RF_Receive(uint8_t* data, uint8_t size)
{
    nrf24l01_Receive(data, size);
}

void RF_Transmit(uint8_t* data, uint8_t size)
{
    RF_PowerUp();

    nrf24l01_Send(data, size);
}

uint8_t RF_ClearIRQ()
{
    return registerWrite(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
}

/*
 * PortConfig.hpp
 *
 * Copyright (c) 2018 Yulay Rakhmangulov.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef WIRELESS_BASE_PORTCONFIG_H
#define WIRELESS_BASE_PORTCONFIG_H

#include <avr/io.h>

/*

Running on internal 8 MHz clock

CPU Clock Freq = 1 MHz
Main prescaler = 8

PA0 - input IRQ from nRF24L01+

PA1 - output: light low
PA2 - input: user IO (ADC button/resistor divider)

nFR24L01+ interface
PA3 - CE    chip enable output
PA4 - SCK   master clock output
PA5 - MISO  master input
PA6 - MOSI  master output
PA7 - CSN   chip select negative output

PB0 - output: fan high speed, engage winding 1
PB1 - output: fan low speed, engage winding 2
PB2 - output: light high

PB3 - /Reset


TIMER0 - delays and ADC driver


TIMER1 - time and timeouts driver

*/
#define TIMER0_PRESCALER    256
#define LINE_FREQ           60
#define TIMER0_RELOAD       (F_CPU/TIMER0_PRESCALER/LINE_FREQ)

#if (TIMER0_RELOAD > 0xff)
    #error "Timer0 reload value is greater then 8 bits. Consider increasing prescaler value."
#elif (TIMER0_RELOAD == 0)
    #error "Timer0 reload value is zero. Consider decreasing prescaler value."
#endif



#define TIMER1_PRESCALER    256
#define TICK_FREQ		    1
#define TIMER1_RELOAD       (F_CPU/TIMER1_PRESCALER/TICK_FREQ)


#if (TIMER1_RELOAD > 0xffff)
    #error "Timer1 reload value is greater then 16 bits. Consider increasing prescaler value."
#elif (TIMER1_RELOAD == 0)
    #error "Timer1 reload value is zero. Consider decreasing prescaler value."
#endif


//
//Hardware assisted SPI
//
#define     SPI_PORT    PORTA
#define     SPI_DDR     DDRA
#define     SPI_PIN     PINA

#define     SPI_SCK     PA4
#define     SPI_MISO    PA5
#define     SPI_MOSI    PA6
#define     SPI_CSN     PA7

//
//nRF24L01+ radio chip
//
#define     NRF_PORT    PORTA
#define     NRF_DDR     DDRA
#define     NRF_PIN     PINA

#define     NRF_CE      PA3
#define     NRF_IRQ     PA0

#define     NRF_CHANNEL         2

#define     NRF_PAYLOAD_SIZE    sizeof(SensorData)




#endif //WIRELESS_BASE_PORTCONFIG_H

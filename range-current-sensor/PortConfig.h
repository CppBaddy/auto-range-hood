/*
 * PortConfig.hpp
 *
 * Copyright (c) 2021 Yulay Rakhmangulov.
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

#ifndef WIRELESS_SENSOR_PORTCONFIG_H
#define WIRELESS_SENSOR_PORTCONFIG_H

#include <avr/io.h>
#include "SensorModel.h"

/*

Running on internal 8 MHz clock

CPU Clock Freq = 1 MHz
Main prescaler = 8

PA0 - input IRQ from nRF24L01+

PA1 - input (reserved)

PA2 - Average Voltage from Current Sensor (1.8 - 3.3)/3, max 1.1V

ADC VCC - valid range 1.8V to 3.3V (6A -> 2.1V, 15A -> 3.1V)

ADC TEMP - chip temperature

PA3 - CE    chip enable output
PA4 - SCK   master clock output
PA5 - MISO  master input
PA6 - MOSI  master output
PA7 - CSN   chip select negative output

PB0 - input (reserved)
PB1 - input (reserved)
PB2 - input (reserved)

PB3 - Reset negative

*/
//#define TIMER0_PRESCALER    8
#define TIMER1_PRESCALER    1024
#define TIMER1_FREQ			  1

#define TIMER1_RELOAD       (F_CPU/TIMER1_PRESCALER/TIMER1_FREQ)


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

#define     NRF_PAYLOAD_SIZE    sizeof(SensorModel)

#endif //WIRELESS_SENSOR_PORTCONFIG_H

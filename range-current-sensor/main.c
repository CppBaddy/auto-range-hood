
/*
 * Current Sensor
 *
 * main.c
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

#include <util/delay.h>
#include <stdlib.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#include "PortConfig.h"
//#include "Uart.h"
#include "Transmitter.h"


inline void ADC_Setup()
{
    ADCSRA |= _BV(ADEN) | _BV(ADIE) | _BV(ADPS2); //prescaler 16, enable interrupt
    ADCSRB |= _BV(ADLAR);
    DIDR0  |= _BV(ADC2D); //disable digital input on ADC2
}

inline void AdcIn_Current()
{
    //input ADC2
    //internal bandgap 1.1V as reference
    ADMUX = _BV(REFS1) | _BV(MUX1);
}

inline void AdcIn_VBat()
{
    //input internal bandgap 1.1V voltage reference
    //vbat as reference
    ADMUX = _BV(MUX5) | _BV(MUX0);
}

inline void AdcIn_Temp()
{
    //input internal temperature sensor
    //1.1V as reference
    ADMUX = _BV(REFS1) | _BV(MUX5) | _BV(MUX1);
}

inline void ADC_Start()
{
    ADCSRA |= _BV(ADSC); //start conversion
}

void SendState();
void setup();

volatile uint8_t time;
volatile bool nrfIRQ; //RF data sent flag
volatile bool requestStatus;

volatile SensorModel model;

uint8_t prevPinA;


int main()
{
    setup();

    prevPinA = PINA;

    model.sensorId = SENSOR_ID;

    for(;;)
    {
    	wdt_reset();

        if(requestStatus /*&& 19 < model.voltage*/) //TODO check if voltage higher than minimum 1.9V for NRF
        {
        	requestStatus = false;

        	SendState();
        }


        if(nrfIRQ)
        {
            nrfIRQ = false;

            RF_ClearIRQ();

            RF_PowerDown();
        }

        set_sleep_mode(SLEEP_MODE_IDLE);

        sleep_enable();

            sleep_cpu();

        sleep_disable();
    }

    return 0;
}

void setup()
{
    cli();

    //setup port A
    //PA0 - input IRQ from nRF24L01+
    //PA1 - input (reserved)
    //PA2 - input current draw
    //PA3 - CE chip enable for nRF24L01+
    //PA4 - SCK output for nRF24L01+
    //PA5 - MISO input from nRF24L01+
    //PA6 - MOSI output for nRF24L01+
    //PA7 - CSN chip select negative nRF24L01+
    DDRA  |= _BV(PA3) | _BV(PA4) | _BV(PA6) | _BV(PA7); /* set PAx as output */
    PORTA |= _BV(PA1) | _BV(PA7); //pullup on PA1 and disable CSN

    //setup port B
    //PB0 - input (reserved)
    //PB1 - input (reserved)
    //PB2 - input (reserved)
    //PB3 - Reset
    PORTB |= _BV(PB0) | _BV(PB1) | _BV(PB2); //enable pull-ups

    //Interrupt inputs
    PCMSK0 = _BV(PCINT0); //enable mask for PA0 input
    GIFR  |= _BV(PCIF0);  //clear pin change interrupt flags
    GIMSK |= _BV(PCIE0);  //enable PortA pin change interrupt

    //Timer1 state machine driver
    TCCR1A = 0;                                  //CTC mode, TOP is OCR1A
    TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS10); //prescaler clk/1024
    TCCR1C = 0;
    OCR1A = TIMER1_RELOAD;
    TIMSK1 |= _BV(OCIE1A); //enable CompA interrupt

	ADC_Setup();
    AdcIn_VBat();

	PRR |= _BV(PRTIM0) | _BV(PRUSI); //disable timer0, usi clocks

	wdt_enable(WDTO_8S); // 8 seconds timeout

    sei(); //Enable interrupts
}

void SendState()
{
    RF_Init(); //just in case Vcc dropped below 1.9V

    RF_Transmit((uint8_t*)&model, sizeof(model));
}


/* External input interrupt handler */
//ISR( INT0_vect )
//{}

/* Pin change interrupt handler */
ISR( PCINT0_vect )
{
    uint8_t pinA = PINA;
    uint8_t changed = pinA ^ prevPinA;

    prevPinA = pinA;

    if((changed & _BV(NRF_IRQ)) & PCMSK0) //enabled IRQ from nRF
    {
        if((pinA & _BV(NRF_IRQ)) == 0) //active low
        {
            nrfIRQ = true;
        }
    }
}

/* Pin change interrupt handler */
//ISR( PCINT1_vect )
//implemented in Uart.c

/* Timer1 interrupt handler */
ISR( TIM1_COMPA_vect )
{
	++time;

	switch((time & 3)) // at ~1Hz
	{
	case 0:
	case 1:
	case 2:
		 //acquire VBat, Current Draw, Temperature
		ADC_Start();
		break;
	default:
		requestStatus = true;
		break;
	}
}

/* Timer1 interrupt handler */
//ISR( TIMER1_OVF_vect )
//{}

/* Timer1 interrupt handler */
//ISR( TIM1_OVF_vect )
//{}


// ADC interrupt service routine
ISR( ADC_vect )
{
	switch((time & 3))
	{
	case 0:
		model.voltage = ADCH; //TODO recalculate into volts
        AdcIn_Current();
		break;
	case 1:
		model.current = ADCH; //TODO recalculate into amps
		AdcIn_Temp();
		break;
	case 2:
		model.temperature = ADCH; //TODO recalculate into degrees
		AdcIn_VBat();
		break;
	default:
		break;
	}
}



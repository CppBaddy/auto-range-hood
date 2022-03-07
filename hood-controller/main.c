/*
 * Fan Controller
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

#include <stdlib.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>

#include <util/delay.h>

#include "PortConfig.h"
#include "RangeHood.h"
#include "Receiver.h"



inline void Timer0_Enable()
{
    PRR &= ~_BV(PRTIM0);
    TCNT0 = 0;
}
inline void Timer0_Disable()
{
    PRR |= _BV(PRTIM0);
}

inline void Timer1_Enable()
{
    PRR &= ~_BV(PRTIM1);
    TCNT1 = 0;
}
inline void Timer1_Disable()
{
    PRR |= _BV(PRTIM1);
}

inline void Fan_HighSpeed()
{
    PORTB &= ~_BV(PB1);
    PORTB |= _BV(PB0);
}
inline void Fan_LowSpeed()
{
    PORTB &= ~_BV(PB0);
    PORTB |= _BV(PB1);
}
inline void Fan_Off()
{
    PORTB &= ~(_BV(PB1) | _BV(PB0));
}

inline void Light_High()
{
    PORTA |= _BV(PA1);
    PORTB |= _BV(PB2);
}
inline void Light_Low()
{
    PORTA |= _BV(PA1);
    PORTB &= ~_BV(PB2);
}
inline void Light_Off()
{
    PORTA &= ~_BV(PA1);
    PORTB &= ~_BV(PB2);
}

inline void ADC_Setup()
{
    ADCSRA |= _BV(ADEN) | _BV(ADIE) | _BV(ADPS0) | _BV(ADPS1); //pre-scaler 8, enable interrupt
    ADCSRB |= _BV(ADLAR);
    DIDR0  |= _BV(ADC2D); //disable digital input on ADC2
}

inline void AdcIn_Buttons()
{
	//Vcc as reference
	//input ADC2
    ADMUX = _BV(MUX1);
}

inline void AdcIn_Temp()
{
    //1.1V as reference
	//input internal temperature sensor ADC8
    ADMUX = _BV(REFS1) | _BV(MUX1) | _BV(MUX5);
}

inline void ADC_Start()
{
    ADCSRA |= _BV(ADSC); //start conversion
}


void setup();

volatile bool nrfIRQ = false; //RF data received flag

volatile uint8_t inputMonitor = 0;

volatile uint8_t time;

SensorData sensor;

volatile RangeHood hood;

volatile uint8_t adcChannel;

uint8_t prevPinA;

volatile uint8_t fanSpeedChangeFlag = 0;

const uint8_t adcTable[] = {
		FanAuto_LedAuto,
		FanAuto_LedHigh,
		FanAuto_LedLow,
		FanHigh_LedAuto,
		FanHigh_LedHigh,
		FanHigh_LedLow,
		FanLow_LedAuto,
		FanLow_LedHigh,
		FanLow_LedLow,
};

const uint8_t fanSpeedMap[] = {
		FanOff,
		FanOff,
		FanOff,
		HighSpeed,
		HighSpeed,
		HighSpeed,
		LowSpeed,
		LowSpeed,
		LowSpeed,
};

const uint8_t ledLightMap[] = {
		LightOff,
		LightHigh,
		LightLow,
		LightOff,
		LightHigh,
		LightLow,
		LightOff,
		LightHigh,
		LightLow,
};

int main()
{
    setup();

    hood.fanAuto 		= true;
    hood.fanNextState 	= FanOff;
    hood.fanState 		= FanOff;

    hood.ledAuto 		= true;
    hood.ledState 		= LightOff;
    hood.ledNextState 	= LightOff;

    hood.inputState 	= FanAuto_LedAuto;

    Fan_Off();
    Light_Off();

    prevPinA = PINA;

    RF_Init();

    for(;;)
    {
    	wdt_reset();

        if(nrfIRQ) //Receive current sensor data
        {
            nrfIRQ = false;

            RF_ClearIRQ();

            RF_Receive((uint8_t*)&sensor, sizeof(SensorData));

            hood.fanTimeout = kAutoModeTimeout;
            hood.ledTimeout = kAutoModeTimeout;

            if(hood.fanAuto)
            {
                if(sensor.current >= CurrentHigh)
                {
                	hood.fanNextState = HighSpeed;
                }
                else
                {
                	hood.fanNextState = LowSpeed;
                }
            }

            if(hood.ledAuto)
            {
            	hood.ledNextState = LightLow;
            }
        }


        if(0 == hood.fanTimeout)
        {
			hood.fanNextState = FanOff;
			hood.fanAuto = true;
        }

        if(0 == hood.ledTimeout)
        {
        	hood.ledNextState = LightOff;
        	hood.ledAuto = true;
        }

        if(InputChanged == inputMonitor)
        {
        	inputMonitor = InputProcessed;

        	uint8_t state = hood.inputState;

			//find out fan and led button states
        	uint8_t i = 0;
			for(; i < 9; ++i)
			{
				if(hood.input > adcTable[i])
				{
					state = adcTable[i];
					break;
				}

				if(i == 8)
				{
					state = adcTable[i];
					break;
				}
			}

			if(state != hood.inputState)
			{
				hood.fanNextState = fanSpeedMap[i];
				hood.ledNextState = ledLightMap[i];

	            hood.fanAuto = (FanOff == hood.fanNextState);
	            hood.ledAuto = (LightOff == hood.ledNextState);

	            if(!hood.fanAuto)
	            {
	            	hood.fanTimeout = kManualModeTimeout;
	            }

	            if(!hood.ledAuto)
	            {
		            hood.ledTimeout = kManualModeTimeout;
	            }

	            hood.inputState = state;
			}
        }

        if(hood.fanState != hood.fanNextState && (SpeedSteady == fanSpeedChangeFlag))
        {
            switch(hood.fanState)
            {
            case FanOff:
            	(LowSpeed == hood.fanNextState) ? Fan_LowSpeed() : Fan_HighSpeed();
                hood.fanState = hood.fanNextState;
            	break;

            case LowSpeed:
            case HighSpeed:
            	Fan_Off();

            	if(FanOff != hood.fanNextState)
            	{   //protection against short between fan motor windings
            		//by sequencing: off + on operations for longer than mains period
            		//because triacs used have zero cross detector
					fanSpeedChangeFlag = ChangingSpeed;
					hood.fanState = FanOff;
            	}

            	hood.fanState = FanOff;
            	break;

            default:
            	break;
            }
        }

        if(hood.ledState != hood.ledNextState)
        {
        	hood.ledState = hood.ledNextState;

			switch(hood.ledState)
			{
			case LightOff:
				Light_Off();
				break;
			case LightLow:
				Light_Low();
				break;
			case LightHigh:
				Light_High();
				break;
			default:
				break;
			}
        }

        set_sleep_mode(SLEEP_MODE_IDLE); //to keep Timer0 & Timer1 running

        sleep_enable();

            sleep_cpu();

        sleep_disable();
    }

    return 0;
}

void setup()
{
    cli();

    //switch to 8 MHz clock
    //CLKPR = _BV(CLKPCE); //enable Clock Prescale Register write
    //CLKPR = 0; //change prescaler to 1, effectively set 8 MHz system clock

    //setup port A
    //PA0 - input: IRQ from nRF24L01+
    //PA1 - output: light low
    //PA2 - input: ADC2 buttons/resistor divider
    //PA3 - CE chip enable for nRF24L01+
    //PA4 - SCK output for nRF24L01+
    //PA5 - MISO input from nRF24L01+
    //PA6 - MOSI output for nRF24L01+
    //PA7 - CSN chip select negative nRF24L01+
    DDRA |= _BV(PA1) | _BV(PA3) | _BV(PA4) | _BV(PA6) | _BV(PA7); /* set PAx as output */
    PORTA |= _BV(PA7); //enable pull-ups and disable CSN

    //setup port B
    //PB0 - output: fan high speed
    //PB1 - output: fan low speed
    //PB2 - output: light high
    //PB3 - Reset
    DDRB |= _BV(PB0) | _BV(PB1) | _BV(PB2); //enable output

    //Interrupt inputs
    PCMSK0 = _BV(PCINT0); //enable mask for PA0 input
    GIFR  |= _BV(PCIF0);  //clear pin change interrupt flags
    GIMSK |= _BV(PCIE0);  //enable PortA pin change interrupt

    //Timer0 ADC driver
    TCCR0A = _BV(WGM01);   //CTC mode
    TCCR0B  = _BV(CS02);   //prescaler 256
    OCR0A = TIMER0_RELOAD;
    TIMSK0 |= _BV(OCIE0A);           //enable OCIE0A interrupt

    //Timer1 state machine driver
    TCCR1A = 0;   //CTC mode, TOP is OCR1A
    TCCR1B = _BV(WGM12) | _BV(CS12); //prescaler clk/256
    OCR1A = TIMER1_RELOAD;           //transmit out on COMPA
    TIMSK1 |= _BV(OCIE1A);           //enable OCIE1A interrupt

    PRR |= _BV(PRUSI); //disable usi clocks

    ADC_Setup();

	wdt_enable(WDTO_8S); // 8 seconds timeout

    sei(); //Enable interrupts
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
ISR( TIM1_COMPA_vect )// at ~1Hz
{
	++time;

	if(hood.fanTimeout)
	{
		--hood.fanTimeout;
	}

	if(hood.ledTimeout)
	{
		--hood.ledTimeout;
	}
}

/* Timer1 interrupt handler */
//ISR( TIM1_OVF_vect )
//{}

/* Timer0 interrupt handler */
ISR( TIM0_COMPA_vect ) //at ~16ms
{
	ADC_Start(); //acquire user input and temperature

	if(fanSpeedChangeFlag)
	{
		--fanSpeedChangeFlag;
	}
}

/* Timer0 interrupt handler */
//ISR( TIM0_OVF_vect )
//{}

// ADC interrupt service routine
ISR( ADC_vect )
{
	switch((adcChannel & 1))
	{
	case 0:
	{
		uint8_t v = ADCH;

		if(v != hood.input)
		{
			inputMonitor = InputChanging;
			hood.input = v;
		}
		else if(InputChanging == inputMonitor)
		{
			inputMonitor = InputChanged; //input stable
		}

		AdcIn_Temp();
		break;
	}
	case 1:
		hood.temperature = ADCH; //TODO recalculate into degrees
		AdcIn_Buttons();
		break;
	default:
		break;
	}

	++adcChannel;
}


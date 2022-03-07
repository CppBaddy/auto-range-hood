/*
 * RangeHood.h
 *
 *  Created on: Jan. 30, 2022
 *      Author: yulay
 */

#ifndef RANGEHOOD_H_
#define RANGEHOOD_H_

#include <stdint.h>

#include "SensorModel.hpp"

enum eCurrent
{
	CurrentHigh  = 0x70 //TODO find right value
};

/*
 * resistors used in resistor divider coder:
 * Auto +5v	2.2k
 * Fan low 	2.2k
 * Fan high 4.3k
 * Led low 	8.2k
 * Led high 15k
 *
 * if you use different resistor values
 * recalculate your coefficients in resistor-divider.ods spreadsheet
*/
enum eAdcValue //in value descending order
{
	ADC_ValueThreshold = 5,

	FanAuto_LedAuto	= (255 - ADC_ValueThreshold),
	FanAuto_LedHigh	= (FanAuto_LedAuto*8)/9,
	FanAuto_LedLow	= (FanAuto_LedAuto*4)/5,
	FanHigh_LedAuto	= (FanAuto_LedAuto*2)/3,
	FanHigh_LedHigh	= (FanAuto_LedAuto*16)/26,
	FanHigh_LedLow	= (FanAuto_LedAuto*8)/14,
	FanLow_LedAuto	= FanAuto_LedAuto/2,
	FanLow_LedHigh	= (FanAuto_LedAuto*8)/17,
	FanLow_LedLow	= (FanAuto_LedAuto*4)/9,
};

enum eMonitorState
{
	InputProcessed,
	InputChanging,
	InputChanged
};

enum eTimeouts
{
	kAutoModeTimeout       = 2*60, //seconds
	kManualModeTimeout     = 20*60,
};

enum eFanState
{
	SpeedSteady   = 0,
	ChangingSpeed = 2,

	FanOff,
	LowSpeed,
	HighSpeed,

};

enum eLedState
{
	LightOff,
	LightLow,
	LightHigh,
};

typedef struct RangeHood
{
	uint8_t temperature;

	uint8_t input;
	uint8_t inputState; //state of manual input

	uint8_t fanNextState; //state of outputs
	uint8_t fanState;

	uint8_t ledNextState; //state of outputs
	uint8_t ledState;

	bool fanAuto; //state of inputs
	bool ledAuto;

	uint16_t fanTimeout;
	uint16_t ledTimeout;
} RangeHood;

#endif /* RANGEHOOD_H_ */

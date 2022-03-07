/*
 * Transmitter.h
 *
 *  Created on: 2020-08-31
 *      Author: yulay
 */

#ifndef TRANSMITTER_H_
#define TRANSMITTER_H_

void RF_Init();
void RF_PowerUp();
void RF_PowerDown();
void RF_Transmit(uint8_t* data, uint8_t size);
void RF_ClearIRQ();

#endif /* TRANSMITTER_H_ */

/*
 * Receiver.h
 *
 *  Created on: 2020-08-31
 *      Author: yulay
 */

#ifndef RECEIVER_H_
#define RECEIVER_H_

void RF_Init();
void RF_PowerUp();
void RF_PowerDown();
void RF_Receive(uint8_t* data, uint8_t size);
void RF_Transmit(uint8_t* data, uint8_t size);
uint8_t RF_ClearIRQ();

#endif /* RECEIVER_H_ */

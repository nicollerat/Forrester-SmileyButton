/*
 * main.h
 *
 *  Created on: 13 juil. 2020
 *      Author: Marc
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include <stdbool.h>

void mTick();
void mSi115xHandler(int src);
void mDelay_us(unsigned long  us);

extern uint8_t devID[];
extern uint8_t RFsetup;
extern bool mSending;

#endif /* MAIN_H_ */

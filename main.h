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
void mStopSensors();
void mStartTest();
void mStartNormal();

typedef enum { mode_TEST, mode_NORMAL, mode_OFF } tSmileyMode;

extern tSmileyMode smileyMode;
extern uint8_t devID[];
extern uint16_t mRFsetup;
extern bool mSending;
extern uint16_t mLockTime;

#endif /* MAIN_H_ */

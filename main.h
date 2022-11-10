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
void mPushButtonHandler(int button);
void mTimerButtonHandler(int buttons);
void mHandleEX1();
void mProcTurnOff();

typedef enum { mode_TEST, mode_NORMAL, mode_OFF } tSmileyMode;

extern tSmileyMode smileyMode;
extern uint8_t devID[];
extern uint16_t mRFsetup;
extern bool mSending;
extern uint16_t mLockTime;
extern bool isProgramming;
extern int mBlankingCounter;
extern int mTurnOffDelay;

#endif /* MAIN_H_ */

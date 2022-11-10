/*
 * hw.h
 *
 *  Created on: 13 juil. 2020
 *      Author: Marc
 */

#ifndef HW_H_
#define HW_H_

#include <stdint.h>

#define LED_GREEN   1
#define LED_YELLOW  2
#define LED_RED     4
#define LED_ALL     7

void hwInit();
void hwBackground();

void hwSetLed(int mask);
void hwClearLed(int mask);
void hwSetLedFlash(int mask);
void hwFlashLed(int mask);
void hwBlinkLed(int mask);

void hwTimerStart();
void hwTimerStop();
void hwTimerSimIRStart();
void hwTimerSimIRStop();
void hwStartTimerForever();

void hwDebLedOn(uint8_t mask);
void hwDebLedOff(uint8_t mask);
bool hwLedTick();
bool hwIsButtonPressed();
void hwSetEX1(bool state);

extern uint16_t hwFlagP2Interrupt ;

#endif /* HW_H_ */

/*
 * hw.h
 *
 *  Created on: 13 juil. 2020
 *      Author: Marc
 */

#ifndef HW_H_
#define HW_H_

#define LED_GREEN   1
#define LED_YELLOW  2
#define LED_RED     4
#define LED_ALL     7

void hwInit();
void hwBackground();

void hwSetLed(int mask);
void hwClearLed(int mask);
void hwSetLedFlash(int mask);

void hwTimerStart();
void hwTimerStop();

#endif /* HW_H_ */

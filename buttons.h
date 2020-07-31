/*
 * buttons.h
 *
 *  Created on: 29 juil. 2020
 *      Author: Marc
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_


#include "stdbool.h"
#include "stdint.h"

bool bHandleButtonNormal(bool bG, bool bM, bool bD);

extern uint8_t repeatG, repeatD, repeatM;
uint8_t totalKey;

#endif /* BUTTONS_H_ */

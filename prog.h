/*
 * prog.h
 *
 *  Created on: 5 août 2020
 *      Author: Marc
 */

#ifndef PROG_H_
#define PROG_H_


#include "def.h"
#include <stdbool.h>

bool progHandleButtonState(bool BG, bool BM, bool BD);

#if 1==0
typedef enum { pmNone, pmPhase1, pmPhase2, pmPhase3, pmPhase4, pmPhase5, pmProg, pmProgFreq } tProgMode;

extern tProgMode progMode;
extern bool progTestGoProg;

void progHandleButton(bool bG, bool bM, bool bD);
void progHandleButtonFreq(bool bG, bool bM, bool bD);
void progCheckButton(bool bG, bool bM, bool bD);
void progCheckNoButton(bool bG, bool bM, bool bD);
void progShowCurrentProg();
void progHandleTimer1();
void progHandleMode();
void progInit();
void progBackground();
#endif


#endif /* PROG_H_ */

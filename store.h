/*
 * store.h
 *
 *  Created on: 14 août 2020
 *      Author: Marc
 */

#ifndef STORE_H_
#define STORE_H_

#include <stdint.h>

typedef struct {
    uint16_t RFsetup;
    uint16_t LockTime;
    uint16_t magic;
} tSetup;

void storeInit();
void storeWriteConfig();

extern tSetup setupData;

#endif /* STORE_H_ */

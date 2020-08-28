/*
 * store.c
 *
 *  Created on: 14 août 2020
 *      Author: Marc
 */

#include "store.h"
#include <driverlib.h>
#include "main.h"

#define MAGIC 0x4256

typedef struct {
    uint16_t RFsetup;
    uint16_t LockTime;
    uint16_t magic;
} tSetup;

#pragma SET_DATA_SECTION(".fram_vars")

tSetup setupData;

void storeInit()
{
    if (setupData.magic==MAGIC) {
        mRFsetup = setupData.RFsetup;
        mLockTime = setupData.LockTime;
    }
}

// Ecrit la config dans la FRAM
void storeWriteConfig()
{
    uint16_t m=MAGIC;

    FRAMCtl_write16(&mRFsetup, &setupData.RFsetup, 1);
    FRAMCtl_write16(&mLockTime, &setupData.LockTime, 1);
    FRAMCtl_write16(&m, &setupData.magic, 1);
}

/*
 * store.c
 *
 *  Created on: 14 août 2020
 *      Author: Marc
 */

#include "store.h"
#include <driverlib.h>
#include "main.h"
#include "def.h"

#define MAGIC 0x4256



#pragma SET_DATA_SECTION(".fram_vars")

// Initialize the FRAM structure with the default values
tSetup setupData = {
     .RFsetup = DEFAULT_RF_VERSION,
     .LockTime = DEFAULT_LOCK_SEC * WD_TICK_PER_SECOND,
     .magic = MAGIC
};

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

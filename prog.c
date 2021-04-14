/*
 * prog.c
 *
 *      On a une fonction principale qui traite l'état des boutons à chaque lecture (500ms)
 *
 *      Cette fonction gère un état de programmation général selon les pressions détectées.
 *
 *
 *  Created on: 5 août 2020
 *      Author: Marc
 */






#include "prog.h"
#include "hw.h"
#include "main.h"
#include "store.h"

#include <stdbool.h>
#include <driverlib.h>

#define LEDG LED_GREEN
#define LEDM LED_YELLOW
#define LEDD LED_RED

typedef enum { psNone, psShowCurrentTime, psWaitSetup, psWaitSetTime, psSetTime, psShowTime,
    psWaitToShowCurrentFreq,  psShowCurrentFreq, psSetFreq, psShowFreq, psCheckForStop } tProgState;

tProgState progState;
int progCounter=0;
int progShowTime=0;
unsigned int progTempLockTime=0;
int progLock1Second, progLock10Secs, progLock1Minute;
int8_t tempRFsetup=0;

void progStartShowTime()
{
    progTempLockTime = mLockTime / TICK_PER_SECOND; // Lock time est donné en 1/2 seconde
    progCounter=1;
    hwSetLed(LED_ALL);
}

bool progContinueShowTime()
{

    if (progCounter==0) {
        // Allume la LED la plus grande
        if (progTempLockTime>=60) {
            hwSetLed(LEDD);
            progTempLockTime-=60;
        } else if (progTempLockTime>=10) {
            hwSetLed(LEDM);
            progTempLockTime-=10;
        } else if (progTempLockTime>=1) {
            hwSetLed(LEDG);
            progTempLockTime-=1;
        }
        progCounter=1;
    } else {
        hwClearLed(LED_ALL);
        progCounter=0;
    }

    return (progTempLockTime==0) && (progCounter==0);
}

void progStartSetTime()
{
    progLock1Second=0;
    progLock10Secs=0;
    progLock1Minute=0;
    progCounter=0;
}

bool progWaitButtonReleased=false;

// Traite les boutons pour définir le temps de blocage
//   Retourne vrai quand le temps est défini
bool progContinueSetTime(bool bG, bool bM, bool bD)
{
    bool ret=false;

    if (progWaitButtonReleased) {
        if (!bG && !bM && !bD) {
            progWaitButtonReleased=false;
        } else {
            return false;
        }
    }

    // Selon le bouton, incrémente les compteurs
    if (bG) {
        if (progLock1Second<10) {
            hwSetLed(LEDG);
            progLock1Second++;
            progWaitButtonReleased=true;
            progCounter=0;
        }
    } else if (bM) {
        if (progLock10Secs<10) {
            hwSetLed(LEDM);
            progLock10Secs++;
            progWaitButtonReleased=true;
            progCounter=0;
        }
    } else if (bD) {
        if (progLock1Minute<10) {
            hwSetLed(LEDD);
            progLock1Minute++;
            progWaitButtonReleased=true;
            progCounter=0;
        }
    } else if (progCounter>=PROG_MAX_DELAY) {
        hwClearLed(LED_ALL);
        mLockTime = (progLock1Minute *60 + progLock10Secs*10 + progLock1Second) * TICK_PER_SECOND;
        if (mLockTime>0) {
            if (mLockTime<PROG_MIN_TIME) mLockTime=PROG_MIN_TIME;
            if (mLockTime>PROG_MAX_TIME) mLockTime=PROG_MAX_TIME;
            ret=true;
        }
    } else {
        progCounter++;
        hwClearLed(LED_ALL);
    }

    return ret;
}

// Prépare l'indication de la fréquence actuelle
void progStartShowFreq()
{
    hwClearLed(LED_ALL);
    progCounter=0;
}

// Continue l'affichage de la fréquence
//   eteint, rallume si fréquence étendue
bool progContinueShowFreq()
{
    bool ret=false;

    switch(progCounter) {
    case 0:
        switch (mRFsetup) {
        case EU_VERSION:
        case MIN_VERSION:
            hwSetLed(LEDG);
            break;

        case US_VERSION:
        case MID_VERSION:
            hwSetLed(LEDM);
            break;

        case ASIA_VERSION:
        case MAX_VERSION:
            hwSetLed(LEDD);
            break;
        }
        progCounter=1;
        break;

    case 1:
        hwClearLed(LED_ALL);
        progCounter=2;
        break;

    case 2:
        switch (mRFsetup) {
        case MIN_VERSION:
            hwSetLed(LEDG);
            break;

        case MID_VERSION:
            hwSetLed(LEDM);
            break;

        case MAX_VERSION:
            hwSetLed(LEDD);
            break;
        }
        progCounter=3;
        break;

    default:
        hwClearLed(LED_ALL);
        ret=true;
        break;
    }

    return ret;
}

void progStartSetFreq()
{
    tempRFsetup = -1;
    progCounter=0;
}

// Si un bouton est pressé, on allume la LED
// après le délais, on regarde si le bouton est toujours pressé, si oui la fréquence étendue est choisie
bool progContinueSetFreq(bool bG, bool bM, bool bD)
{
    bool ret=false;
    progCounter++;

    // Selon le bouton, définit la fréquence de base
    if (bG) {
        hwSetLed(LEDG);
        tempRFsetup = EU_VERSION;
    }

    if (bM) {
        hwSetLed(LEDM);
        tempRFsetup = US_VERSION;
    }

    if (bD) {
        hwSetLed(LEDD);
        tempRFsetup = ASIA_VERSION;
    }

    if (progCounter>=PROG_MAX_DELAY) {
    // Si le bouton est toujours pressé, définit la fréquence étendue

        hwClearLed(LEDD | LEDM | LEDG);
        if (bG) {
            tempRFsetup = MIN_VERSION;
        }

        if (bM) {
            tempRFsetup = MID_VERSION;
        }

        if (bD) {
            tempRFsetup = MAX_VERSION;
        }
        ret=true;
        if (tempRFsetup>=0) mRFsetup=tempRFsetup;
    }

    return ret;
}

int prog_testWD=0;

// Fonction principale. Appelée périodiquement avec le rafraîchissement de la lecture des boutons
bool progHandleButtonState(bool BG, bool BM, bool BD)
{
    bool ret=progState!=psNone;

    switch(progState) {
    case psNone:
        if (BG && !BM && BD) {
            ret=true;
            progCounter++;
            if (progCounter>2) {
                hwSetLed(LED_GREEN | LED_RED);
            }
        } else {
            if (progCounter>PROG_ENTER_LOW && progCounter<PROG_ENTER_HIGH) {
                //hwBlinkLed(LED_GREEN | LED_YELLOW | LED_RED);
                ret=true;
                progState=psShowCurrentTime;
                progStartShowTime();
                hwDebLedOn(2);
            } else {
                progCounter=0;
            }
        }
        break;

    case psShowCurrentTime:
        if (progContinueShowTime()) {
            progState=psWaitSetup;
            progCounter=0;
        } else if (BG || BM || BD) {
            progState=psNone;
        }
        break;

    case psWaitSetup: // Attend un bouton pour aller en déf de freq ou en def de temps
        if (BM) {
            progState=psWaitSetTime;
            progStartSetTime();
            hwBlinkLed(LED_ALL);
            hwDebLedOn(4);

        } else if (BG) {
            progState=psWaitToShowCurrentFreq;
            hwSetLed(LED_GREEN);
            progCounter=0;
        } else if (BD) {
            hwSetLed(LED_RED);
            progState=psCheckForStop;
            progCounter=0;
        } else {
            progCounter++;
            if (progCounter>=PROG_MAX_DELAY) {
                progState=psNone;
                hwDebLedOff(7);
            }
        }
        break;

    case psCheckForStop:
        if (BD) {
            progCounter++;
        } else {
            progState=psNone;
            if (progCounter>PROG_ENTER_LOW && progCounter<PROG_ENTER_HIGH) {
                mStopSensors();
                progState = psNone;
                progCounter=0;
                hwClearLed(LED_RED);
            } else {
                hwBlinkLed(LED_ALL);
            }
        }
        break;

    case psWaitSetTime:
        if (!BM && !BG && !BD) {
            progState=psSetTime;
        }
        break;

    case psSetTime:
        if (progContinueSetTime(BG, BM, BD)) { // Gère progCounter et indique true si on a fini
            progState=psShowTime;
            progStartShowTime();
            storeWriteConfig();
        } else if (progCounter>PROG_MAX_DELAY) { // si rien n'a été défini, sort ici
            progState=psNone;
            progCounter=0;
            hwDebLedOff(7);
        }
        break;

    case psShowTime:
        if (progContinueShowTime()) {
            progState=psNone;
            hwDebLedOff(7);
        }
        break;

    case psWaitToShowCurrentFreq:
        if (BG) {
            progCounter++;
        } else {
            hwClearLed(LED_GREEN);
            if (progCounter>=PROG_ENTER_LOW && progCounter<PROG_ENTER_HIGH) {
                progStartShowFreq();
                progState=psShowCurrentFreq;
                hwDebLedOn(1);
            } else { // On quitte sans autre
                progState=psNone;
                progCounter=0;
                hwDebLedOff(7);

            }
        }
        break;

    case psShowCurrentFreq:
        if (progContinueShowFreq()) {
            progStartSetFreq();
            progState=psSetFreq;
            hwDebLedOn(4);
        }
        break;

    case psSetFreq:
        if (progContinueSetFreq(BG, BM, BD)) {
            progStartShowFreq();
            storeWriteConfig();
            progState=psShowFreq;
        }
        break;

    case psShowFreq:
        if (progContinueShowFreq()) {
            progState=psNone;
            hwDebLedOff(7);
        }
        break;
    }

    // Si on envisage une programmation, on retourne vrai
    return ret;
}

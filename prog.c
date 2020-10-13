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
    psWaitToShowCurrentFreq,  psShowCurrentFreq, psSetFreq, psShowFreq } tProgState;

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
            switch(1) {
            case 1:
                while(1) { // Bloque pour voir
                    hwSetLed(LED_RED);

                    __bic_SR_register(GIE);
                    __bis_SR_register(LPM4_bits);
                    prog_testWD++;
                }
                break;
            case 2: // Arrête les mesures pour consommer un min
                mStopSensors();
                break;
            }

        } else {
            progCounter++;
            if (progCounter>=PROG_MAX_DELAY) {
                progState=psNone;
                hwDebLedOff(7);
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

#if 1==0
// PROJET DE LA SMILEY NORMALE
// ******************************************************************************************************

// Traitement du cas de programmation
tProgMode progMode = pmNone;
int progTimer=0;
int progCounter=0;
int progLock1Second, progLock10Secs, progLock1Minute;
bool progTestGoProg=false;
int progTempLockTime;
bool progShowUpdate;

// Frequency setup
int progFreqTimer=0;
int tempRFsetup=-1;
bool progSetNewFreqFlag=false;
#define PROG_FREQ_INIT 15
#define PROG_FREQ_OFF1 10
#define PROG_FREQ_EXT  5

#define PROG_FREQ_MAX 100
#define PROG_FREQ_MID 50

// typdef enum { fsIdle, fsWaitButtonReleased, fsCheckBlank, fs
void __progFreqDone();
void __progStartFreqProg();

void __progInit()
{
    progMode = pmNone;
    progTestGoProg=false;
    progTempLockTime=0;
}

void __progBackground()
{
    // Test si on a la condition entrée en prog
    if (progTestGoProg) {
        progCounter=0;
        progMode=pmPhase1;
        progTestGoProg=false;
        hwSetLed(LEDD | LEDG);
    }

    // Définit la nouvelle fréquence si nécessaire
    if (progSetNewFreqFlag) {
        progSetNewFreqFlag=false;
        RFsetup = tempRFsetup;
        mRFsetup=RFsetup;
        tempRFsetup=0;
    }
}

// Test les cas de programmation,
//    appelé lors du timeout watchdog
void __progHandleMode()
{

    bool bG = PORTBbits.RB4==0;
    bool bM = PORTBbits.RB5==0;
#ifdef NO_RIGHT_BUTTON
    bool bD = false;
#else
    bool bD = PORTBbits.RB6==0;
#endif

    switch(progMode) {
        case pmNone: break;

        case pmPhase1:
            // Bouton prog pressé 5s pour continuer
#ifdef NO_RIGHT_BUTTON
            if (bG && bM) {
#else
            if (bG && (!bM) && bD) {
#endif
                progCounter++;
                if (progCounter>=PROG_DELAY) {
                    progMode=pmPhase2;
                    progLock1Second=0;
                    progLock10Secs=0;
                    progLock1Minute=0;
#ifdef DEBUG
                    hwSetLed(LEDD | LEDM | LEDG);
                    mSetTimer(tmTurnLEDoff);
#endif
                }
            } else {
                hwClearLed(LEDD | LEDM | LEDG);
                progMode=pmNone;
            }
            break;

        case pmPhase2:
            // Attend que les boutons soient relâchés
            if ((!bG) && (!bM) && (!bD)) {
                progMode=pmPhase3;
                progCounter=0;
                progTimer=PROG_DELAY;
                hwClearLed(LEDD | LEDM | LEDG);
                progShowCurrentProg();
                hwEnableButtonsInterrupt();
            } else {
                // Continue d'incrémenter jusqu'à un timeout
                progCounter++;
                if (progCounter>(PROG_DELAY*2)) {
                    hwClearLed(LEDD | LEDM | LEDG);
                    progMode=pmNone;
                }
            }
            break;

        case pmPhase3:
        case pmPhase4:

            // Si on poirote trop longtemps, on sort
            progCounter++;
            if (progCounter>=PROG_DELAY) {
                progMode=pmNone;
            }

            break;

        case pmPhase5:
            // Attente du bouton orange soit relâché
            if ((!bG) && (!bM) && (!bD)) {
                progMode=pmProg;
                hwSetLed(LEDD | LEDM | LEDG);
                mSetTimer(tmTurnLEDoffAndButtonOn);
            } else {
                progCounter++;
                if (progCounter>=PROG_DELAY) {
                    progMode=pmNone;
                }
            }
            break;

            // Mode de programmation, on attend le timeout ou les pressions de touches
        case pmProg:
            if (progTimer>0) {
                progTimer--;
            } else {
                // Calcule le temps de lock
                int tSec=progLock1Second + (10*progLock10Secs) + (60*progLock1Minute);

                // Ne prend en compte que si on a programmé qqc
                if (tSec>0) {
                    if (tSec<LOCK_MIN_SEC) tSec=LOCK_MIN_SEC;
                    if (tSec>LOCK_MAX_SEC) tSec=LOCK_MAX_SEC;
                    mButtonLockTime = tSec ;
                    mSavedLockTime = mButtonLockTime;
                }

                progMode=pmNone;
                mButtonBlanking=PROG_BLANKING;
                hwDisableButtonsInterrupt();


                // Petit flash
                progShowCurrentProg();
            }
            break;

        case pmProgFreq:
            break;

        // V1.2
        default:

            mReset(2);
            break;
    }
}

// Vérifie qu'aucun bouton n'est pressé, annule la procédure d'indication du temps de prog
//   si un bouton est pressé
void __progCheckNoButton(bool bG, bool bM, bool bD)
{
    if (bG || bM || bD) {
        // Finit de montrer
        progMode = pmNone;
        TMR1IE=0;
        TMR1ON=0;
        TMR1IF=0;
        hwClearLed(LEDD | LEDM | LEDG);
    }
}

// Appelé lors d'une pression de bouton après avoir montré le temps de prog
//   Appelé depuis interrupt timer
void __progCheckButton(bool bG, bool bM, bool bD)
{
    // On attend bM, tous les autres font sortir
    if (bM) {
        progMode = pmProg;
        hwSetLed(LEDD | LEDM | LEDG);
        // prépare le bouton suivant
        mSetTimer(tmTurnLEDoffAndButtonOn);
    } else if (bG) {
        progStartFreqProg();

        progMode = pmProgFreq;
        mSetTimer(tmReadButtonState);
    } else {
        progMode = pmNone;
        // prépare le bouton suivant
        mSetTimer(tmTurnLEDoffAndButtonOn);
    }

}

// Traite les boutons pendant le mode de programmation
void __progHandleButton(bool bG, bool bM, bool bD)
{
    // Selon le bouton, incrémente les compteurs
    if (bG) {
        if (progLock1Second<10) {
            hwSetLed(LEDG);
            progLock1Second++;
            progTimer=PROG_DELAY;
        }
    }

    if (bM) {
        if (progLock10Secs<10) {
            hwSetLed(LEDM);
            progLock10Secs++;
            progTimer=PROG_DELAY;
        }
    }


    if (bD) {
        if (progLock1Minute<10) {
            hwSetLed(LEDD);
            progLock1Minute++;
            progTimer=PROG_DELAY;
        }
    }

    // prépare le bouton suivant
    mSetTimer(tmTurnLEDoffAndButtonOn);
}

#define PROG_FREQ_DELAY 60

typedef enum {stfWaitReleased, stfShowCurrent, stfSetNewFreq, stfShowProgrammed} tStateFreq;

tStateFreq stateFreq;

// Démarre la programmation de la fréquence
void __progStartFreqProg()
{
    progSetNewFreqFlag=false;
    progFreqTimer=PROG_FREQ_MAX;
    stateFreq=stfWaitReleased;
    tempRFsetup=mRFsetup;
}

void __progStartShowing()
{
    progFreqTimer=PROG_FREQ_INIT;
    switch(tempRFsetup) {
        case EU_VERSION:
        case MIN_VERSION:
            hwSetLed(LEDG); break;
        case US_VERSION:
        case MID_VERSION:
            hwSetLed(LEDM); break;
        case ASIA_VERSION:
        case MAX_VERSION:
            hwSetLed(LEDD); break;
    }
}

void __progContinueShowing()
{
    if (progFreqTimer==0) {
        // Time out, bouton trop long
        hwClearLed(LEDD | LEDM | LEDG);
    } else if (progFreqTimer==PROG_FREQ_OFF1) {
        // Eteint les LEDs
        hwClearLed(LEDD | LEDM | LEDG);

    } else if (progFreqTimer==PROG_FREQ_EXT) {
        // Rallume la LED pour les valeurs étendues
        switch(tempRFsetup) {
        case MIN_VERSION:
            hwSetLed(LEDG); break;
        case MID_VERSION:
            hwSetLed(LEDM); break;
        case MAX_VERSION:
            hwSetLed(LEDD); break;
        }
    }
}

// Traite les boutons pendant le mode de programmation
//   Appelé depuis le timer
void __progHandleButtonFreq(bool bG, bool bM, bool bD)
{
    bool bPressed = bG && (!bM)  && (!bD);

    CLRWDT();

    switch(stateFreq) {
        case stfWaitReleased:
            // Wait the button is release, check this is in the delay
            progFreqTimer--;
            if (progFreqTimer<=0) {
                progFreqDone();
                return;
            } else if (!bPressed) {
                if (progFreqTimer<PROG_FREQ_MID) {
                    stateFreq=stfShowCurrent;
                    progStartShowing();
                } else {
                    progFreqDone();
                    return;
                }
            }
            break;

        case stfShowCurrent:
            progFreqTimer--;
            progContinueShowing();
            if(progFreqTimer<=0) {
                stateFreq=stfSetNewFreq;
                progFreqTimer=PROG_FREQ_MAX;
                tempRFsetup=-1;
            }
            break;

        case stfSetNewFreq:
            progFreqTimer--;
            if(progFreqTimer==0) {

                stateFreq=stfShowProgrammed;
                progStartShowing();
            } else if (progFreqTimer>30) {
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
            } else if (progFreqTimer==30) {
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
            }

            break;

        case stfShowProgrammed:
            progFreqTimer--;
            progContinueShowing();
            if(progFreqTimer<=0) {
                progFreqDone();
                return;
            }
            break;
    }

    mSetTimer(tmReadButtonState);

}

void __progFreqDone()
{
    progMode=pmNone;
#ifdef DEBUG_TEST
    mButtonBlanking=0;
#else
    mButtonBlanking=PROG_BLANKING;
#endif
    mSetTimer(tmTurnLEDoff);
    if (tempRFsetup>=0 && tempRFsetup<6) {
        // hwSetLed(LEDD | LEDM | LEDG);
        progSetNewFreqFlag=true;
    }
}

// Montre le temps de blocage sur les LEDs
void __progShowCurrentProg()
{
    progShowUpdate=true;
    progTempLockTime = mButtonLockTime;
    TMR1 = 0; // Periode avec prescaler de 4 donne 1/4s environ
    TMR1IF=0;
    TMR1ON=1;
    TMR1IE=1;
}

// Utilisé pour indiquer l'état de programmation
//  Appelé de l'interruption du timer1
void __progHandleTimer1()
{
    // Ca prend du temps, il faut quittancer le WD qui est toujours en marche
    CLRWDT();

    if (progShowUpdate) {

        if (progTempLockTime>=60) {
            hwSetLed(LEDD);
            progTempLockTime-=60;
        } else if (progTempLockTime>=10) {
            hwSetLed(LEDM);
            progTempLockTime-=10;
        } else if (progTempLockTime>=1) {
            hwSetLed(LEDG);
            progTempLockTime-=1;
        } else {
            // Finit de montrer
            TMR1IE=0;
            TMR1ON=0;
            TMR1IF=0;
            /* TODO Problème avec ceci ?
            if (progMode==pmPhase3) {
                progMode=pmPhase4;
            } else {
                progMode=pmNone;
            }
            mButtonBlanking=PROG_BLANKING;
             * */
            progMode=pmPhase4;
        }
        progShowUpdate=false;
    } else {
        hwClearLed(LEDD | LEDM | LEDG);
        progShowUpdate=true;
    }


}
#endif

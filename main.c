#include <driverlib.h>
#include <stdbool.h>

#include "hwi2c.h"
#include "hw.h"
#include "hwspi.h"

#include "si115x.h"
#include "si115x_functions.h"

#include "def.h"
#include "nrf905.h"
#include "buttons.h"
#include "prog.h"
#include "store.h"


#pragma SET_DATA_SECTION(".fram_vars")

uint8_t devID[4] = DEV_ID;

#pragma SET_DATA_SECTION()

uint16_t mRFsetup = DEFAULT_RF_VERSION;
uint16_t mLockTime = DEFAULT_LOCK_SEC * TICK_PER_SECOND;
int mDetectCount=0;

bool mSending = false;
int16_t mBatVoltage=0;
volatile int timer=0;

/* Tick est utilisé pour éteindre les LEDs
 *   TODO utilise pour de plus amples tâches ?
 */
void mTick()
{
    timer++;
    hwLedTick();
}

// En même temps que le FLASH, on envoie un message
void mButtonPressed(uint8_t mask)
{
    if (bHandleButtonNormal((mask&LED_GREEN)!=0, (mask&LED_YELLOW)!=0, (mask&LED_RED)!=0)) {
        nrfNewButton();
    }
}

void mDelay_us(unsigned long us)
{
    // TODO ajuster selon fréquence du proc ?
    while(us>0) {
        __delay_cycles(1000); // 1ms
        if (us>=1000) us -= 1000;
        else us=0;
    }
}

struct I2C_HANDLE I2C_LEFT;
struct I2C_HANDLE I2C_MID;
struct I2C_HANDLE I2C_RIGHT;

int main(void) {

    //configuration des I2C_HANDLE



    I2C_LEFT.BASE = EUSCI_B1_BASE;
    I2C_LEFT.isA = false;
    I2C_LEFT.slave_addr = 0x53;

    I2C_MID.BASE = EUSCI_B0_BASE;
    I2C_MID.isA = false;
    I2C_MID.slave_addr = 0x53;

    I2C_RIGHT.BASE = EUSCI_B0_BASE;
    I2C_RIGHT.isA = false;
    I2C_RIGHT.slave_addr = 0x52;

    WDT_A_hold(WDT_A_BASE);
    // Démarre le WD
    WDT_A_initWatchdogTimer(WDT_A_BASE, WDT_A_CLOCKSOURCE_VLOCLK , WDT_A_CLOCKDIVIDER_32K);
    //WDT_A_initWatchdogTimer(WDT_A_BASE, WDT_A_CLOCKSOURCE_VLOCLK , WDT_A_CLOCKDIVIDER_512K);

    WDT_A_start(WDT_A_BASE);

    hwInit();

    hwDebLedOn(7);


    hwInitI2C(&I2C_MID);
    hwInitI2C(&I2C_LEFT);

#ifdef USE_SPI
    hwspiInit();
#endif

    hwSetLed(LED_RED+LED_YELLOW+LED_GREEN);

    switch(CONFIG_CHIP) {
    case SINGLE_CHIP:
        si115x_init_3CH(&I2C_MID);
        Si115xStart(&I2C_MID);
        break;

    case MULTI_CHIP:
        si115x_init_1CH(&I2C_MID);
        si115x_init_1CH(&I2C_RIGHT);
        si115x_init_1CH(&I2C_LEFT);

        Si115xStart(&I2C_LEFT);
        Si115xStart(&I2C_MID);
        Si115xStart(&I2C_RIGHT);
        break;

    }

    hwClearLed(LED_YELLOW);

    // Ils font ça dans la démo, mais ça ne fait pas marcher
    Si115xReadFromRegister(&I2C_MID, SI115x_REG_PART_ID);

    hwClearLed(LED_GREEN+LED_RED);

    storeInit();

    hwDebLedOff(7);

    // Test conso avant démarrage
    //   LA CONSO EST ELEVEE ICI... L'INTERRUPT DOIT ÊTRE ACTIVE
    //   Si on n'active pas le Si11x5, la conso est de 10uA
   // __bis_SR_register(LPM4_bits);

    while (1)
    {
        //Enter LPM0, enable interrupts
            //__bis_SR_register(LPM0_bits + GIE);
            if ((P2IN & 0x7) != 0x7) { // Une ligne est activée...
                hwFlagP2Interrupt = (P2IN & 0x7) ^ 0x7;
            } else {
                __bis_SR_register(LPM4_bits + GIE);
            }
            WDT_A_resetTimer(WDT_A_BASE);
            hwBackground();
    }
//    {
//        hwSendI2C(I2C_LEFT, transmitData, 2);
//
//        GPIO_setOutputHighOnPin(
//                GPIO_PORT_P1,
//                GPIO_PIN0
//            );
//    }
}

// Mesures faites par un des capteurs
SI115X_SAMPLES samples_left = {.min=0xFFFF};
SI115X_SAMPLES samples_mid = {.min=0xFFFF};
SI115X_SAMPLES samples_right = {.min=0xFFFF};

SI115X_SAMPLES samples_all;

int mBlankingCounter = 0;


// Traite les résultats obtenus pour allumer les LEDs
//   On détermine quel bouton est le plus probablement pressé
//   Sert de base temps, car la période est régulière.
void mHandleResult()
{
    const uint16_t nbMeas = 1; // Ajuster selon le setup du chip
    const uint16_t thrSET = 20*nbMeas; // Seuil dépend de la puissance de la LED et des distances désirées.
    const uint16_t thrCLEAR = 10*nbMeas;
    const uint16_t thrPROG = 20*nbMeas;

    // Les 3 canaux donnent les résultat dans l'ordre centre, droite, gauche
    static uint16_t thr = thrSET;
    uint16_t currentLED=0;
    int nbLedsOn=0;
    static int lastLED = 0;
    static int LedOnCount = 0; // Limite le temps ON
    static bool isProgramming = false;

    // Test du blocage
    if (mBlankingCounter) {
        mBlankingCounter--;
    } else {

        // Traite la gestion de la programmation. Chaque bouton est traité individuellement
        bool BG = samples_all.ch2>thrPROG;
        bool BM = samples_all.ch0>thrPROG;
        bool BD = samples_all.ch1>thrPROG;
        if (progHandleButtonState(BG, BM, BD)) {
            isProgramming=true;
            return;
        } else if (isProgramming) {
            mBlankingCounter=PROG_BLANKING;
            isProgramming=false;
        }

        // Variantes d'implémentation
        switch(2) {
        case 1: // Différence entre les boutons
            // Canal 1 (centre)
            if (samples_all.ch0>samples_all.ch1 && samples_all.ch0>samples_all.ch2) {
                if (samples_all.ch0>thr) {
                    hwSetLed(LED_YELLOW);
                    currentLED=LED_YELLOW;
                    hwClearLed(LED_RED+LED_GREEN);
                }

                // Canal 1 (droite)
            } else if (samples_all.ch1>samples_all.ch0 && samples_all.ch1>samples_all.ch2 && samples_all.ch1>samples_all.ch2*2 ) {
                if (samples_all.ch1>thr) {
                    hwSetLed(LED_RED);
                    currentLED=LED_RED;
                    hwClearLed(LED_YELLOW + LED_GREEN);
                }

                // Canal 2 (gauche)
            } else if (samples_all.ch2>samples_all.ch1 && samples_all.ch2>samples_all.ch0 && samples_all.ch2>samples_all.ch1*2 ) {
                if (samples_all.ch2>thr) {
                    hwSetLed(LED_GREEN);
                    currentLED=LED_GREEN;
                    hwClearLed(LED_RED + LED_YELLOW);
                }
            }
            break;

        case 2: // Valeurs absolues
            // Canal 1 (centre)
                if (samples_all.ch0>thr) {
                    currentLED|=LED_YELLOW;
                    nbLedsOn++;
                } else {
                    hwClearLed(LED_YELLOW);
                }

                if (samples_all.ch1>thr) {
                    currentLED|=LED_RED;
                    nbLedsOn++;
                } else {
                    hwClearLed(LED_RED);
                }

                if (samples_all.ch2>thr) {
                    currentLED|=LED_GREEN;
                    nbLedsOn++;
                } else {
                    hwClearLed(LED_GREEN);
                }
                break;
        }

        // Si on a un bouton pressé, on traite le moment où un vote est envoyé
        if (currentLED && nbLedsOn==1) { // Un seul bouton détecte
            // TODO pas tout de suite hwSetLed(currentLED);
            // Attend quelques périodes
            if (lastLED==currentLED) {
                mDetectCount++;
                int nb_succ=NB_SUCCESSIVE;
#ifdef TEST_TEMP_MORT
                if (currentLED==LED_RED) nb_succ=NB_SUCCESSIVE;
                if (currentLED==LED_YELLOW) nb_succ=NB_SUCCESSIVE+1;
                if (currentLED==LED_GREEN) nb_succ=NB_SUCCESSIVE+2;
#endif
                if (mDetectCount==nb_succ) {
                    // Plus de bouton, on accepte le dernier pressé et on fait un FLASH
                    mButtonPressed(lastLED);
                    hwFlashLed(lastLED);
                    lastLED=0;
                    mBlankingCounter=mLockTime;
                    mDetectCount=0;
                }
            } else {
                mDetectCount=0;
                lastLED=currentLED;
            }
            thr = thrCLEAR; // Seuil plus bas pour déclencher
        } else if (nbLedsOn>1) {
            LedOnCount++;
            mDetectCount=0;
            if (LedOnCount>MAX_TIME_LED_ON) {
                mBlankingCounter=mLockTime;
                hwClearLed(LED_ALL);
            } else {
                // TODO ne pas allumer hwSetLed(currentLED);
            }
        } else {
            thr = thrSET;
            lastLED=0;
            mDetectCount=0;
            LedOnCount=0;
        }
    } // on n'était pas en blanking

    // Timer pour la répétition des votes
    nrfTimerHandler();

    // Conversion de la tension
    if (0) {
    static int subD = 10;
    if (--subD==0) {
        mBatVoltage=hwReadVoltage();
        subD=10;
    }
    }
}

// Partir avec une grande valeur
void mTrakMin(SI115X_SAMPLES * sample)
{
    // Traque le minimum
    if (sample->ch0!=0) {
        if (sample->min >= sample->ch0) {
            sample->min=sample->ch0;
            sample->nb_min=0;
        } else {
            sample->min++;
            sample->nb_min++;
            if (sample->nb_min>MIN_OVERRIDE) sample->min=sample->ch0;
        }
    } else {
        // TODO MN ignorer le 0, valeur improbable plutot liée à une problème ?
        sample->min=0;
        sample->nb_min=0;
    }
}

/* Traite une interruption
 *    Lit les capteurs qui ont fait une interruption
 *    Selon les valeurs mesurées, allume les LEDs
 */
void mSi115xHandler(int src)
{
    hwDebLedOn(2);

    if (src&GPIO_PIN0) {
       si115x_handler(&I2C_LEFT, &samples_left);
        mTrakMin(&samples_left);
    }
    if (src&GPIO_PIN1) {
        si115x_handler(&I2C_MID, &samples_mid);
        mTrakMin(&samples_mid);


        if (CONFIG_CHIP==SINGLE_CHIP) {
            mHandleResult();
        }

        // Traite sur le bouton du milieu pour avoir la cadence 1/mesures
        if (CONFIG_CHIP==MULTI_CHIP) {

            samples_all.ch0 = samples_mid.ch0 - samples_mid.min;
            samples_all.ch1 = samples_right.ch0 - samples_right.min;
            samples_all.ch2 = samples_left.ch0 - samples_left.min;
            mHandleResult();
        }
    } // capteur central

    if (src&GPIO_PIN2) {
        si115x_handler(&I2C_RIGHT, &samples_right);
        mTrakMin(&samples_right);
    }

    hwDebLedOff(2);
}

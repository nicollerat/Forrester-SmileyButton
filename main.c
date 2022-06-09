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
#include "main.h"


#pragma SET_DATA_SECTION(".fram_vars")

uint8_t devID[4] = DEV_ID;

#pragma SET_DATA_SECTION()

uint16_t mRFsetup = DEFAULT_RF_VERSION;
uint16_t mLockTime = DEFAULT_LOCK_SEC * TICK_PER_SECOND;
int mDetectCount=0;

bool mSending = false;
int16_t mBatVoltage=0;
volatile int timer=0;
bool mSensorStopped=false;

#define TEST_DELAY (3000 * 3)

// Periode de mesure
int mMeasPeriod_10ms = 50;
int mTurnOffDelay_10ms = TEST_DELAY;

bool isProgramming = false;

// Keep the current mode of the Smiley
tSmileyMode smileyMode = mode_TEST;

struct I2C_HANDLE I2C_LEFT;
struct I2C_HANDLE I2C_MID;
struct I2C_HANDLE I2C_RIGHT;

/* Tick est utilisé pour éteindre les LEDs
 *   TODO utilise pour de plus amples tâches ?
 */
void mTick()
{
    hwDebLedOn(1);
    timer++;
    hwLedTick();
    hwDebLedOff(1);
}

/* Stoppe les capteurs et programme le watchdog pour qu'il soit un timer
 *      La smiley peut fonctionne en basse conso dans ce mode
 */
void mStopSensors()
{

    si115x_Stop(&I2C_MID);
    si115x_Stop(&I2C_LEFT);
    si115x_Stop(&I2C_RIGHT);

    // Utilise le watchdog comme timer pour tester le bouton
    WDT_A_hold(WDT_A_BASE);

    // Set Timer bit
    WDTCTL = WDTPW | WDT_A_CLOCKDIVIDER_8192 | WDTTMSEL | WDT_A_CLOCKSOURCE_VLOCLK;

    WDT_A_start(WDT_A_BASE);

    // Valide l'interruption du watchdog
    SFRIE1 |= WDTIE;

    smileyMode = mode_OFF;
    hwClearLed(LED_ALL);
}

/* Redémarre les capteur est supprime le mode timer du watchdog
 *      Retour au fonctionnement normal
 */
void mStartSensors()
{

    // Utilise le watchdog comme timer pour tester le bouton
    WDT_A_hold(WDT_A_BASE);

    // Clear Timer bit
    WDTCTL = WDTPW | WDT_A_CLOCKDIVIDER_32K | WDT_A_CLOCKSOURCE_VLOCLK;

    WDT_A_start(WDT_A_BASE);

    SFRIE1 &= ~WDTIE;

    si115x_Start(&I2C_MID);
    si115x_Start(&I2C_LEFT);
    si115x_Start(&I2C_RIGHT);

    GPIO_clearInterrupt( GPIO_PORT_P2, 0xFFFF);
}

// Démarre les capteurs et programme le temps de blocage normal
void mStartNormal()
{
    hwSetLed(LED_GREEN);
    mStartSensors();
    mTurnOffDelay_10ms = -1;
    hwClearLed(LED_GREEN);
    smileyMode = mode_NORMAL;
}

void mStartTest()
{
    hwSetLed(LED_RED);
    mStartSensors();
    mTurnOffDelay_10ms= TEST_DELAY; // As there are 3 sensors, time is decremented 3 times faster
    hwClearLed(LED_RED);
    smileyMode = mode_TEST;
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
        mMeasPeriod_10ms = si115x_init_3CH(&I2C_MID);
        Si115xStart(&I2C_MID);
        break;

    case MULTI_CHIP:
        mMeasPeriod_10ms = si115x_init_1CH(&I2C_MID, 0) ;
        si115x_init_1CH(&I2C_RIGHT, -1);
        si115x_init_1CH(&I2C_LEFT, 1);

        Si115xStart(&I2C_LEFT);
        Si115xStart(&I2C_MID);
        Si115xStart(&I2C_RIGHT);
        break;

    }

    hwClearLed(LED_YELLOW);

    // Ils font ça dans la démo, mais ça ne fait pas marcher
    Si115xReadFromRegister(&I2C_MID, SI115x_REG_PART_ID);

    hwClearLed(LED_GREEN+LED_RED);

    // Load dead time and RF setup
    storeInit();

    hwDebLedOff(7);

    mStartTest();

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

            // Test le délais d'extinction
            if (isProgramming) {
                // Reset the delay
                if (mTurnOffDelay_10ms>0) mTurnOffDelay_10ms=TEST_DELAY;
            } else {
                if (mTurnOffDelay_10ms>0) {
                    if (mTurnOffDelay_10ms<=3*mMeasPeriod_10ms) {
                        hwSetLed(LED_ALL);
                    }
                    mTurnOffDelay_10ms -= mMeasPeriod_10ms;
                    if (mTurnOffDelay_10ms<=0) {
                        mStopSensors();
                    }
                }
            }
    }

}

int mBlankingCounter = 0;

// Traite une pression de bouton
void mAcceptButton(int button)
{
    mButtonPressed(button);
    hwFlashLed(button);
    mBlankingCounter=mLockTime;
}

void mPushButtonHandler(int button)
{
    if (mBlankingCounter && (smileyMode == mode_NORMAL)) {
        mBlankingCounter--;
    } else {
        if (smileyMode != mode_OFF) {
            mAcceptButton(button);
        }
    }
}

// Mesures faites par un des capteurs
SI115X_SAMPLES samples_left = {.min=0xFFFF};
SI115X_SAMPLES samples_mid = {.min=0xFFFF};
SI115X_SAMPLES samples_right = {.min=0xFFFF};

SI115X_SAMPLES samples_all;



// Traite les résultats obtenus pour allumer les LEDs
//   On détermine quel bouton est le plus probablement pressé
//   Sert de base temps, car la période est régulière.
void mHandleSiResult()
{
    const uint16_t nbMeas = 1; // Ajuster selon le setup du chip
    const uint16_t thrSET = THR_SET*nbMeas; // Seuil dépend de la puissance de la LED et des distances désirées.
    const uint16_t thrCLEAR = 10*nbMeas;
    const uint16_t thrPROG = 50*nbMeas;
    const uint16_t thrOffsetChange = 5 * nbMeas;


    // Les 3 canaux donnent les résultat dans l'ordre centre, droite, gauche
    static uint16_t thr = thrSET;
    uint16_t currentLED=0;
    int nbLedsOn=0;
    static int lastLED = 0;
    static int LedOnCount = 0; // Limite le temps ON


    // Test du blocage
    if (mBlankingCounter && (smileyMode == mode_NORMAL)) {
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
            if (mTurnOffDelay_10ms>0) {
                mBlankingCounter=0;
            } else {
                mBlankingCounter=PROG_BLANKING;
            }
            isProgramming=false;
            hwClearLed(LED_ALL);
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

        case 2: // Valeurs absolues VARIANTE DE LA PREMIèRE SERIE
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

        case 3: // Valeurs absolues, compte les leds qui ont un offset changé
            // Canal 1 (centre)
                if (samples_all.ch0>thr) {
                    currentLED|=LED_YELLOW;
                    nbLedsOn++;
                } else if (samples_all.ch0>thrOffsetChange) {
                    nbLedsOn++;
                } else {
                    hwClearLed(LED_YELLOW);
                }

                if (samples_all.ch1>thr) {
                    currentLED|=LED_RED;
                    nbLedsOn++;
                } else if (samples_all.ch1>thrOffsetChange) {
                    nbLedsOn++;
                } else {
                    hwClearLed(LED_RED);
                }

                if (samples_all.ch2>thr) {
                    currentLED|=LED_GREEN;
                    nbLedsOn++;
                } else if (samples_all.ch2>thrOffsetChange) {
                    nbLedsOn++;
                } else {
                    hwClearLed(LED_GREEN);
                }
                break;
        }



        // Si on a un bouton pressé, on traite le moment où un vote est envoyé
        if (currentLED && nbLedsOn==1) { // Un seul bouton détecte
            // pour ne pas allumer tout de suite hwSetLed(currentLED);
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
                    mAcceptButton(lastLED);

                    mDetectCount=0;
                    lastLED=0;
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
                // pour ne pas allumer tout de suite hwSetLed(currentLED);
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
        sample->min=0;
        sample->nb_min=0;
    }
}

/* Traite une interruption
 *    Lit les capteurs qui ont fait une interruption
 *    Selon les valeurs mesurées, allume les LEDs (appel à mHandleResult)
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
            mHandleSiResult();
        }

        // Traite sur le bouton du milieu pour avoir la cadence 1/mesures
        if (CONFIG_CHIP==MULTI_CHIP) {

            samples_all.ch0 = samples_mid.ch0 - samples_mid.min;
            samples_all.ch1 = samples_right.ch0 - samples_right.min;
            samples_all.ch2 = samples_left.ch0 - samples_left.min;
            mHandleSiResult();
        }
    } // capteur central

    if (src&GPIO_PIN2) {
        si115x_handler(&I2C_RIGHT, &samples_right);
        mTrakMin(&samples_right);
    }

    hwDebLedOff(2);
}

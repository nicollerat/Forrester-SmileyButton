#include <driverlib.h>
#include <stdbool.h>

#include "hwi2c.h"
#include "hw.h"

#include "si115x.h"
#include "si115x_functions.h"

#include "def.h"
#include "nrf905.h"
#include "buttons.h"

#pragma SET_DATA_SECTION(".fram_vars")

uint8_t devID[4] = DEV_ID;
uint8_t RFsetup = DEFAULT_RF_VERSION;

#pragma SET_DATA_SECTION()

uint8_t mRFsetup = DEFAULT_RF_VERSION; // copy of the EEPROM

bool mSending = false;

volatile int timer=0;

/* Tick est utilisé pour éteindre les LEDs
 *   TODO utilise pour de plus amples tâches ?
 */
void mTick()
{
    timer++;
    hwClearLed( LED_ALL);
    hwPWMLedStop();
    hwTimerStop();
}

// En même temps que le FLASH, on envoie un message
void mButtonPressed(uint8_t mask)
{
    if (bHandleButtonNormal((mask&LED_GREEN)!=0, (mask&LED_YELLOW)!=0, (mask&LED_RED)!=0)) {
        nrfSendData();
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

    hwInit();


    hwInitI2C(&I2C_MID);
    hwInitI2C(&I2C_LEFT);

    hwspiInit();

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
SI115X_SAMPLES samples_left;
SI115X_SAMPLES samples_mid;
SI115X_SAMPLES samples_right;

// Traite les résultats obtenus pour allumer les LEDs
void mHandleResult()
{
    const uint16_t nbMeas = 1; // Ajuster selon le setup du chip
    const uint16_t thrSET = 40*nbMeas; // Seuil dépend de la puissance de la LED et des distances désirées.
    const uint16_t thrCLEAR = 20*nbMeas;

    // Les 3 canaux donnent les résultat dans l'ordre centre, droite, gauche
    static uint16_t thr = thrSET;
    uint16_t currentLED=0;
    static int lastLED = 0;

    // Canal 1 (centre)
    if (samples_mid.ch0>samples_mid.ch1 && samples_mid.ch0>samples_mid.ch2) {
        if (samples_mid.ch0>thr) {
            hwSetLed(LED_YELLOW);
            currentLED=LED_YELLOW;
            hwClearLed(LED_RED+LED_GREEN);
        }
        //else hwClearLed(LED_YELLOW);

        // Canal 2 (droite)
    } else if (samples_mid.ch1>samples_mid.ch0 && samples_mid.ch1>samples_mid.ch2 && samples_mid.ch1>samples_mid.ch2*2 ) {
        if (samples_mid.ch1>thr) {
            hwSetLed(LED_RED);
            currentLED=LED_RED;
            hwClearLed(LED_YELLOW + LED_GREEN);
        }
        //else hwClearLed(LED_RED);

        // Canal 3 (gauche)
    } else if (samples_mid.ch2>samples_mid.ch1 && samples_mid.ch2>samples_mid.ch0 && samples_mid.ch2>samples_mid.ch1*2 ) {
        if (samples_mid.ch2>thr) {
            hwSetLed(LED_GREEN);
            currentLED=LED_GREEN;
            hwClearLed(LED_RED + LED_YELLOW);
        }
        //else hwClearLed(LED_GREEN);
    } //else { }

    if (currentLED) {
        lastLED=currentLED;
        thr = thrCLEAR; // Seuil plus bas pour déclencher
    } else if (lastLED) {
        mButtonPressed(lastLED);
        hwSetLedFlash(lastLED);
        lastLED=0;
        thr = thrSET;
        hwTimerStart();
    }
}

/* Traite une interruption
 *    Lit les capteurs qui ont fait une interruption
 *    Selon les valeurs mesurées, allume les LEDs
 */
void mSi115xHandler(int src)
{

    if (src&GPIO_PIN0) {
        si115x_handler(&I2C_LEFT, &samples_left);
    }
    if (src&GPIO_PIN1) {
        si115x_handler(&I2C_MID, &samples_mid);


        if (CONFIG_CHIP==SINGLE_CHIP) {
            mHandleResult();
        }
    } // capteur central

    if (src&GPIO_PIN2) {
        si115x_handler(&I2C_RIGHT, &samples_right);
    }

    if (CONFIG_CHIP==MULTI_CHIP) {
        samples_mid.ch1 = samples_right.ch0;
        samples_mid.ch2 = samples_left.ch0;
        mHandleResult();
    }
}

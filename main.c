#include <driverlib.h>

#include "hwi2c.h"
#include "hw.h"

#include "si115x.h"
#include "si115x_functions.h"

#include "def.h"

volatile int timer=0;

void mTick()
{
    timer++;
    hwClearLed( LED_ALL);
    hwPWMLedStop();
    hwTimerStop();
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
            __bis_SR_register(LPM4_bits + GIE);

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
    // Les 3 canaux donnent les résultat dans l'ordre centre, droite, gauche
    const uint16_t thr = 0x30;
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
    } else if (lastLED) {
        hwSetLedFlash(lastLED);
        lastLED=0;
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

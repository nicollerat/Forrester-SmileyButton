/*
 * hw.c
 *
 *  Created on: 13 juil. 2020
 *      Author: Marc
 */

#include "driverlib.h"
#include "hw.h"
#include "main.h"

#define PIN_LED_GREEN   GPIO_PIN0
#define PIN_LED_YELLOW  GPIO_PIN1
#define PIN_LED_RED     GPIO_PIN2

#define CS_SMCLK_DESIRED_FREQUENCY_IN_KHZ   400
#define CS_SMCLK_FLLREF_RATIO   30

uint16_t hwFlagP2Interrupt = 0;

void hwBackground()
{
    if (hwFlagP2Interrupt) {
        mSi115xHandler(hwFlagP2Interrupt);
        hwFlagP2Interrupt=0;
    }
}

void hwInitClock()
{
    //Initialisations selon exemple :

    //Set DCO FLL reference = REFO
    CS_initClockSignal(
            CS_FLLREF,
            CS_REFOCLK_SELECT,
            CS_CLOCK_DIVIDER_1
        );

    //Set Ratio and Desired MCLK Frequency and initialize DCO
    CS_initFLLSettle(
            CS_SMCLK_DESIRED_FREQUENCY_IN_KHZ,
            CS_SMCLK_FLLREF_RATIO
            );

    //Set ACLK = VLO with frequency divider of 1
    CS_initClockSignal(
            CS_ACLK,
            CS_VLOCLK_SELECT,
            CS_CLOCK_DIVIDER_1
            );

    //Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(
            CS_SMCLK,
            CS_DCOCLKDIV_SELECT,
            CS_CLOCK_DIVIDER_1
            );

    //Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(
            CS_MCLK,
            CS_DCOCLKDIV_SELECT,
            CS_CLOCK_DIVIDER_1
            );
}

#define TIMER_PERIOD 5000 // v * 10us

void hwTimerInit()
{
    //Start timer in continuous mode sourced by ACLK

//    Timer_B_initContinuousModeParam param = {0};
//    param.clockSource = TIMER_B_CLOCKSOURCE_ACLK;
//    param.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_1;
//    param.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_ENABLE;
//    param.timerClear = TIMER_B_DO_CLEAR;
//    param.startTimer = true;
//    Timer_B_initContinuousMode(TIMER_B0_BASE, &param);
//
    Timer_B_initUpModeParam param = {0};
    param.clockSource = TIMER_B_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_10;
    param.timerPeriod = TIMER_PERIOD;
    param.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_DISABLE;
    param.captureCompareInterruptEnable_CCR0_CCIE =
            TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE;
    param.timerClear = TIMER_B_DO_CLEAR;
    param.startTimer = true;
    Timer_B_initUpMode(TIMER_B0_BASE, &param);
    //Timer_B_startCounter(TIMER_B0_BASE,  TIMER_B_CONTINUOUS_MODE);
}

void hwTimerStart()
{
    Timer_B_clear(TIMER_B0_BASE);
    Timer_B_clearTimerInterrupt(TIMER_B0_BASE);
    Timer_B_startCounter(TIMER_B0_BASE,  TIMER_B_CONTINUOUS_MODE);
}

void hwTimerStop()
{
    Timer_B_stop(TIMER_B0_BASE);
    Timer_B_clearTimerInterrupt(TIMER_B0_BASE);
}

void hwPWMLedsInit()
{
     P6SEL0=7; // Timer sur PIN P6.0 6.1 et 6.2

     TB3CCR0 = 1000-1;                         // PWM Period
     TB3CCTL1 = OUTMOD_7;                      // CCR1 reset/set
     TB3CCR1 = 250;                            // CCR1 PWM duty cycle
     TB3CCTL2 = OUTMOD_7;                      // CCR2 reset/set
     TB3CCR2 = 250;                            // CCR2 PWM duty cycle
     TB3CCTL3 = OUTMOD_7;                      // CCR3 reset/set
     TB3CCR3 = 250;                            // CCR3 PWM duty cycle

     TB3CTL = TBSSEL__SMCLK  | TBCLR;  // SMCLK, clear TBR

}

void hwPWMLedStop()
{
    TB3CTL  &= ~MC_3;
    P6SEL0=0; // Timer sur PIN P6.0 6.1 et 6.2
}

void hwPWMLedStart()
{
    TB3CTL |= MC__UP;  // Reset UP mode
    P6SEL0=7; // Timer sur PIN P6.0 6.1 et 6.2

}

/* Initialise les I/O utilisées
 *
 */
void hwInit()
{
    hwInitClock();

    PMM_unlockLPM5();

    // Definition de l'état des ports inutilisés
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN1);

    GPIO_setOutputLowOnPin(GPIO_PORT_P3, 0xFF);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN2); // nRF power disable
    GPIO_setAsOutputPin(GPIO_PORT_P3, 0xFF);

    GPIO_setOutputLowOnPin(GPIO_PORT_P4, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P4, 0x3F); // Let SDA2 with pull-up

    GPIO_setOutputLowOnPin(GPIO_PORT_P5, 0x1F);
    GPIO_setAsOutputPin(GPIO_PORT_P5, 0x1F);

   // LED + LED_POWER
    GPIO_setOutputLowOnPin(GPIO_PORT_P6,
        GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN4);

   GPIO_setAsOutputPin(
        GPIO_PORT_P6,
        GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN4
        );

   GPIO_setAsOutputPin(GPIO_PORT_P6, 0xe8);

   // Interruption des chip capteurs
   GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2);
   GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);

   GPIO_clearInterrupt( GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2);
   GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2);

   GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 + GPIO_PIN6 + GPIO_PIN7);
   GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4 + GPIO_PIN5 + GPIO_PIN6 + GPIO_PIN7); // Inutilisé et les LED de debug
   GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3); // Masse pour la mesure de la tension de pile

   // Initialisation d'un timer
   hwTimerInit();

   // PWM pour les LEDs
   hwPWMLedsInit();
}

#define LED_INTENSITY 150

void hwSetLed(int mask)
{
    uint16_t pins=0;

    hwPWMLedStart();

    switch(2) {
    case 1: // GPIO
        if (mask & LED_GREEN) pins |= PIN_LED_GREEN ;
        if (mask & LED_YELLOW) pins |= PIN_LED_YELLOW ;
        if (mask & LED_RED) pins |= PIN_LED_RED ;

        GPIO_setOutputHighOnPin(GPIO_PORT_P6, pins);
        break;

    case 2: // PWM
        if (mask & LED_GREEN)  {TB3CCR1 = LED_INTENSITY; TB3CCTL1 =OUTMOD_7;}
        if (mask & LED_YELLOW) {TB3CCR2 = LED_INTENSITY; TB3CCTL2 =OUTMOD_7;}
        if (mask & LED_RED)    {TB3CCR3 = LED_INTENSITY; TB3CCTL3 =OUTMOD_7;}
        break;
    }
}

void hwSetLedFlash(int mask)
{
    if (mask & LED_GREEN)  TB3CCR1 = 1000;
    if (mask & LED_YELLOW) TB3CCR2 = 1000;
    if (mask & LED_RED) TB3CCR3 = 1000 ;
}

void hwClearLed(int mask)
{
    uint16_t pins=0;
    switch(2) {
    case 1:
        if (mask & LED_GREEN) pins |= PIN_LED_GREEN ;
        if (mask & LED_YELLOW) pins |= PIN_LED_YELLOW ;
        if (mask & LED_RED) pins |= PIN_LED_RED ;

        GPIO_setOutputLowOnPin(GPIO_PORT_P6, pins);
        break;

    case 2:
        // Pour éteindre, on met le rapport cyclique à 0
        // Pour éviter un glitch quand le timer redémarre, on fixe la sortie.
        if (mask & LED_GREEN)  {TB3CCR1 = 0; TB3CCTL1&=~(OUTMOD_7);}
        if (mask & LED_YELLOW) {TB3CCR2 = 0; TB3CCTL2&=~(OUTMOD_7);}
        if (mask & LED_RED) {TB3CCR3 = 0 ; TB3CCTL3&=~(OUTMOD_7);}
        break;
    }
}

// Traitement des interruptions des capteurs
#pragma vector=PORT2_VECTOR
__interrupt void P2_ISR (void)
{
    hwFlagP2Interrupt = GPIO_getInterruptStatus(GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2);
    GPIO_clearInterrupt( GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2);
    __bic_SR_register_on_exit(CPUOFF);
}

//******************************************************************************
//
//This is the TIMER0_B0 interrupt vector service routine.
//
//******************************************************************************
#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR (void)
{
    //Any access, read or write, of the TAIV register automatically resets the
    //highest "pending" interrupt flag
    switch ( __even_in_range(TB0IV,14) ){
        case  0: break;                          //No interrupt
        case  2: break;                          //CCR1 not used
        case  4: break;                          //CCR2 not used
        case  6: break;                          //CCR3 not used
        case  8: break;                          //CCR4 not used
        case 10: break;                          //CCR5 not used
        case 12: break;                          //CCR6 not used
        case 14:
            mTick();
            break;
        default: break;
    }
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR (void)
{
    mTick();
}

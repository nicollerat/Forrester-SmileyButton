/*
 * hw.c
 *
 *  Created on: 13 juil. 2020
 *      Author: Marc
 */

#include "driverlib.h"
#include "hw.h"
#include "main.h"
#include "def.h"

#define PIN_LED_GREEN   GPIO_PIN0
#define PIN_LED_YELLOW  GPIO_PIN1
#define PIN_LED_RED     GPIO_PIN2

#define DEBUG_LED

#if PROC_SPEED==1
#define CS_SMCLK_DESIRED_FREQUENCY_IN_KHZ   400
#define CS_SMCLK_FLLREF_RATIO  30
#else
#define CS_SMCLK_DESIRED_FREQUENCY_IN_KHZ   8000 // Start with 400
#define CS_SMCLK_FLLREF_RATIO   200  // 30
#endif

uint16_t hwFlagP2Interrupt = 0;
uint16_t hwFlagP4Interrupt = 0;


void hwInitAD()
{
    // Initialize ADC with ADC’s built-in oscillator
    ADC_init (ADC_BASE,   ADC_SAMPLEHOLDSOURCE_SC,   ADC_CLOCKSOURCE_ADCOSC,  ADC_CLOCKDIVIDER_1);
    //Switch ON ADC
    ADC_enable(ADC_BASE);
    // Setup sampling timer to sample-and-hold for 16 clock cycles
    ADC_setupSamplingTimer (ADC_BASE,   ADC_CYCLEHOLD_16_CYCLES,   false);
    // Configure the Input to the Memory Buffer with the specified Reference Voltages
    ADC_configureMemory(ADC_BASE,   ADC_INPUT_A0,
                        ADC_VREFPOS_INT, // Vref+ = Int
                        ADC_VREFNEG_AVSS // Vref- = AVss
    );

}

uint16_t hwReadADC()
{
    // Clear the Interrupt Flag and start another conversion
    ADC_clearInterrupt(ADC_BASE,ADC_COMPLETED_INTERRUPT_FLAG);
    // Start a single conversion, no repeating or sequences.
    ADC_startConversion (ADC_BASE,  ADC_SINGLECHANNEL);
    // Wait for the Interrupt Flag to assert
    while( !(ADC_getInterruptStatus(ADC_BASE,ADC_COMPLETED_INTERRUPT_FLAG)) );


    return ADC_getResults(ADC_BASE);
}

uint16_t hwReadVoltage()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN3); // Masse pour la mesure de la tension de pile

    uint16_t v = hwReadADC();

    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3); // Masse pour la mesure de la tension de pile

    return v;
}

// Redémarre le logiciel
void hwReset()
{
    // Compte sur le watchdog pour redémarrer
   PMMCTL0 |= PMMSWPOR;
}

uint16_t buttonCount = 0;

void hwBackground()
{
    while (hwFlagP2Interrupt) {
        uint16_t src=hwFlagP2Interrupt;
        mSi115xHandler(src);
        __bic_SR_register(GIE);
        hwFlagP2Interrupt &= ~src;
        __bis_SR_register(GIE);
    }

    if (hwFlagP4Interrupt) {
        uint16_t src=hwFlagP4Interrupt;
        uint16_t button = 0;
        if (src & GPIO_PIN4) button |= LED_GREEN;
        if (src & GPIO_PIN5) button |= LED_YELLOW;
        if (src & GPIO_PIN0) button |= LED_RED;

        mPushButtonHandler(button);

        __bic_SR_register(GIE);
        hwFlagP4Interrupt &= ~src;
        __bis_SR_register(GIE);
    }

    if (smileyMode==mode_OFF) {
        // Test du bouton
        if ((P5IN & 0x10) ==0) {
            hwDebLedOn(4);
            buttonCount++;
        } else {
            if (buttonCount>5) {
                // Mise en marche normale
                mStartNormal();
            } else if (buttonCount>2) {
                // Mise en marche test
                mStartTest();
            }
            buttonCount=0;
            hwDebLedOff(4);
        }
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

    switch(PROC_SPEED) {
    case 1:
        // Version 400kHz
        param.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_10;
        param.timerPeriod = TIMER_PERIOD;
        break;
    case 2:
        // Version 8MHZ
        param.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_40;
        param.timerPeriod = TIMER_PERIOD*5;
        break;

    }

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
    Timer_B_startCounter(TIMER_B0_BASE,  TIMER_B_UP_MODE);
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
    GPIO_setAsOutputPin(GPIO_PORT_P3, 0xF7);
    P3REN=GPIO_PIN2; // Enable Resistor

    GPIO_setOutputLowOnPin(GPIO_PORT_P4, 0xF1);
    GPIO_setAsOutputPin(GPIO_PORT_P4, 0x31); // Let SDA2 with pull-up

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

   // Bouton power ON
   GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN4);

   // Bouton physiques
   GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN0+GPIO_PIN4+GPIO_PIN5);
   GPIO_selectInterruptEdge(GPIO_PORT_P4, GPIO_PIN0+GPIO_PIN4+GPIO_PIN5, GPIO_HIGH_TO_LOW_TRANSITION);
   GPIO_clearInterrupt( GPIO_PORT_P4, GPIO_PIN0+GPIO_PIN4+GPIO_PIN5);
   GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN0+GPIO_PIN4+GPIO_PIN5);

   // Initialisation d'un timer
   hwTimerInit();

   if ( LED_MODE==2) {
       // PWM pour les LEDs
       hwPWMLedsInit();
   }

   // ADC
   // hwInitAD();
}

// LED de debug
void hwDebLedOn(uint8_t mask)
{
#ifdef DEBUG_LED
    P2OUT |= mask << 5;
#endif
}

void hwDebLedOff(uint8_t mask)
{
    P2OUT &= ~(mask << 5);
}

int hwLedON=0;
// LED principales
void hwSetLed(int mask)
{
    uint16_t pins=0;

    hwLedON |= mask;

    switch(LED_MODE) {
    case 1: // GPIO
        if (mask & LED_GREEN) pins |= PIN_LED_GREEN ;
        if (mask & LED_YELLOW) pins |= PIN_LED_YELLOW ;
        if (mask & LED_RED) pins |= PIN_LED_RED ;

        GPIO_setOutputHighOnPin(GPIO_PORT_P6, pins);
        break;

    case 2: // PWM
        hwPWMLedStart();
        if (mask & LED_GREEN)  {TB3CCR1 = LED_INTENSITY; TB3CCTL1 =OUTMOD_7;}
        if (mask & LED_YELLOW) {TB3CCR2 = LED_INTENSITY; TB3CCTL2 =OUTMOD_7;}
        if (mask & LED_RED)    {TB3CCR3 = LED_INTENSITY; TB3CCTL3 =OUTMOD_7;}
        break;
    }
}

int hwLedFlashState=0;
int hwFlashedLeds=0;

void hwFlashLed(int mask)
{
    hwFlashedLeds=mask;
    hwLedFlashState=2; // set à 0 pour 1 flash, set à 2 pour avoir 2 flash
    hwSetLedFlash(mask);
    hwTimerStart(); // Le timer gère les LEDs
}

void hwBlinkLed(int mask)
{
    hwSetLed(mask);
    hwLedFlashState=0;
    hwTimerStart(); // Le timer gère les LEDs
}

void hwStartTimerForever()
{
    hwLedFlashState=99;
    hwTimerStart(); // Le timer gère les LEDs
}

void hwSetLedFlash(int mask)
{
    hwLedON |= mask;

    switch(LED_MODE) {
    case 2: // PWM
        hwPWMLedStart();
        if (mask & LED_GREEN)  {TB3CCR1 = 1000;TB3CCTL1 =OUTMOD_7;}
        if (mask & LED_YELLOW) {TB3CCR2 = 1000;TB3CCTL2 =OUTMOD_7;}
        if (mask & LED_RED) {TB3CCR3 = 1000 ;TB3CCTL3 =OUTMOD_7;}
        break;
    }
}

// Tick de la LED, vrai quand on peut arrêter le timer
bool hwLedTick()
{
    bool ret=false;
    switch(hwLedFlashState) {
    case 2:
        hwClearLed( LED_ALL);
        hwLedFlashState--;
        break;

    case 1:
        hwSetLedFlash( hwFlashedLeds);
        hwLedFlashState--;
        break;
    case 0:
        hwClearLed( LED_ALL);
        hwTimerStop();
        ret=true;
        break;

    case 99: // rien, laisse tourner le timer
        WDT_A_resetTimer(WDT_A_BASE);
        break;
    }
    return ret;
}

void hwClearLed(int mask)
{
    hwLedON &= ~mask;

    uint16_t pins=0;
    switch(LED_MODE) {
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
        if (hwLedON==0) hwPWMLedStop();
        break;
    }
}

// Traitement des interruptions des capteurs
#pragma vector=PORT2_VECTOR
__interrupt void P2_ISR (void)
{
    uint16_t src = GPIO_getInterruptStatus(GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2);
    hwFlagP2Interrupt |= src;
    GPIO_clearInterrupt( GPIO_PORT_P2, src);
    __bic_SR_register_on_exit(CPUOFF);
}

// Traitement des interruptions des capteurs
#pragma vector=PORT4_VECTOR
__interrupt void P4_ISR (void)
{
    uint16_t src = GPIO_getInterruptStatus(GPIO_PORT_P4, GPIO_PIN0+GPIO_PIN4+GPIO_PIN5);
    hwFlagP4Interrupt |= src;
    GPIO_clearInterrupt( GPIO_PORT_P4, src);
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
    Timer_B_clearTimerInterrupt(TIMER_B0_BASE);

}

int hwWatchdogCnt=0;
// Interrupt du watchdog utilisé pour tester le bouton en mode veille
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void)
{
    hwWatchdogCnt++;
    __bic_SR_register_on_exit(CPUOFF);
}


int hwTrapCnt=0;
int hwUNMICnt=0;
int hwSYSNMICnt=0;


#pragma vector = UNMI_VECTOR
__interrupt void TrapUNMI(void)
{
// this is a trap ISR - check for the interrupt cause here by
// checking the interrupt flags, if necessary also clear the interrupt
// flag
    hwUNMICnt++;
}

#pragma vector = SYSNMI_VECTOR
__interrupt void TrapSYSNMI(void)
{
// this is a trap ISR - check for the interrupt cause here by
// checking the interrupt flags, if necessary also clear the interrupt
// flag
    hwSYSNMICnt++;
}

#pragma vector = unused_interrupts
__interrupt void TrapIsr(void)
{
// this is a trap ISR - check for the interrupt cause here by
// checking the interrupt flags, if necessary also clear the interrupt
// flag
    hwTrapCnt++;
}


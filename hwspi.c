/*
 * hwspi.c
 *
 *  Created on: 29 juil. 2020
 *      Author: Marc
 */

#include "hwspi.h"
#include <stdint.h>

void hwspiInit()
{


        //Initialisation selon exemple

        EUSCI_A_SPI_initMasterParam param = {0};
        param.selectClockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK;
        param.clockSourceFrequency = CS_getSMCLK();
        param.desiredSpiClock = 1000000;
        param.clockPhase = EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
        param.clockPolarity = EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
        param.msbFirst = EUSCI_A_SPI_MSB_FIRST;
        param.spiMode = EUSCI_A_SPI_3PIN;
        EUSCI_A_SPI_initMaster(EUSCI_A1_BASE, &param);

        // Mets les pins en sortie ou en entrée selon la fonction
        //  Laisse le port sur GPIO tant que l'alim du chip n'est pas enclenchée
        P4SEL0 &= ~( GPIO_SPI_PIN_ALL);
        P4DIR |= GPIO_SPI_PIN_OUT;
        P4DIR &= ~GPIO_PIN_SPISOMI;
        P4OUT &= ~GPIO_SPI_PIN_OUT;

}

void hwspiEnable()
{
    // Configurer les Pins pour le SPI (Variables def. dans hwi2c.h)
    P4SEL0 |= GPIO_SPI_PIN_ALL;


    EUSCI_A_SPI_enable(EUSCI_A1_BASE);

}

void hwspiDisable()
{
    P4SEL0 &= ~( GPIO_SPI_PIN_ALL);

    EUSCI_A_SPI_disable(EUSCI_A1_BASE);

}

static volatile int nbSendReceive = 0;
static uint8_t * txBuf = 0;
static uint8_t * rxBuf = 0;

void hwspiTransmission(uint8_t * trm, uint8_t * rcv, int len)
{
    nbSendReceive = len;
    txBuf=trm;
    rxBuf=rcv;
    if (rxBuf) UCA1IE |= UCRXIE;                         // Enable USCI_A1 RX interrupt

    /*
   // while(nbSendReceive>0)
        {
            UCA1IE |= UCTXIE;                     // Enable TX interrupt
            __bis_SR_register(LPM0_bits | GIE);   // enable global interrupts, enter LPM0
            //__no_operation();                     // For debug,Remain in LPM0
        }
    UCA1IE &= ~UCTXIE;
    */
    while(nbSendReceive>0)
    {
        UCA1TXBUF = *txBuf++;
        nbSendReceive--;
        while((UCA1IFG & UCTXIFG)==0) ;
    }

}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCA1IV,USCI_SPI_UCTXIFG))
    {
        case USCI_NONE: break;                // Vector 0 - no interrupt
        case USCI_SPI_UCRXIFG:
              *rxBuf++ = UCA1RXBUF;
              UCA1IFG &= ~UCRXIFG;
              __bic_SR_register_on_exit(LPM0_bits);// Wake up to setup next TX
              break;
        case USCI_SPI_UCTXIFG:
            if (nbSendReceive>0) {
                UCA1TXBUF = *txBuf++;             // Transmit characters
                nbSendReceive--;
            } else {
                UCA1IE &= ~UCTXIE;
                __bic_SR_register_on_exit(CPUOFF);// Wake up to end TX
            }
              break;
        default: break;
    }
}

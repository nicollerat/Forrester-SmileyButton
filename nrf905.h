/*
 * nrf905.h
 *
 *  Created on: 29 juil. 2020
 *      Author: Marc
 *
 *
 *          // SPI interface
    SSPCON=0; // standard, SPI master mode avec Fosc/4
    SSPEN=1; // Enable SPI
    CKE=1; // Data transmitted at falling edge
    TRISC5=1; // SDO
    TRISC3=1; // CLK

    // Contrôle nRF905
    PORTCbits.RC2=0; // Chip Select
    TRISC2=0;

    PORTCbits.RC1=0; // TRX_EN
    TRISC1=0;

    PORTAbits.RA6=0; // TX_EN
    TRISA6=0;

    PORTBbits.RB0=0;// DR
    TRISB0=1;

    TRISB2=1; // CR

    PORTBbits.RB3=0;// PWR_UP
    TRISB3=0;

    PORTAbits.RA7=1; // Transistor (OFF)
    TRISA7=0;
 */

#ifndef NRF905_H_
#define NRF905_H_

void nrfSendData();
void nrfNewButton();
void nrfTimerHandler();


#endif /* NRF905_H_ */

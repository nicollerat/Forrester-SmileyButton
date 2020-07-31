/*
 * def.h
 *
 *  Created on: 24 juil. 2020
 *      Author: Marc
 */

#ifndef DEF_H_
#define DEF_H_

//
#define SINGLE_CHIP 1
#define MULTI_CHIP 2

#define CONFIG_CHIP MULTI_CHIP

#define LED_INTENSITY 150 // 0-1000

// Define this for the US
#define EU_VERSION 0
#define US_VERSION 1
#define ASIA_VERSION 2
#define MIN_VERSION 3
#define MAX_VERSION 4
#define MID_VERSION 5

#define DEFAULT_RF_VERSION EU_VERSION
//#define DEFAULT_RF_VERSION US_VERSION

// Canaux de base pour les régions
    // Formule CH=(freq/2 -422.4)*10
#define EURO_CHANNEL    108  // 866.4 MHz
#define US_CHANNEL      326  // 910 MHz
#define ASIA_CHANNEL    391  // 923 MHz

    // Fréquences supplémentaires donnée par JGC
#define MAX_CHANNEL     495  // 943.8 MHz
#define MIN_CHANNEL     0    // 844.8 MHz
#define MID_CHANNEL     216  // 888 MHz

// Niveau de puissance
#define POWER_BYTE_VERY_LOW 0x02 // -10dBm
#define POWER_BYTE_LOW    0x06 // -2dBm
#define POWER_BYTE_MID    0x0A // 6 dBm
#define POWER_BYTE_HIGH   0x0E // 10 dBm

#define DEV_ID         {186,20,0,52}

#define LOCK_MIN_SEC    2

#define TIMER_CLOCK_US  256 // uS (1Mhz / 256)
#define TIMER_PERIOD    160 // Periode du timer

#define LOCK_MAX_SEC    (10*60)
#define PROG_DELAY      3 // s

#define DEFAULT_LOCK    10 // seconds, default button lock time
#define PROG_BLANKING   5

// Correction factor, in percent, used for button blanking correction
//#define DEBUG
#define RESEND_TIME     10 // Temps entre les envois

#endif /* DEF_H_ */

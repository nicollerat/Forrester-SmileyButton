/*
 * def.h
 *
 *
 *  Created on: 24 juil. 2020
 *      Author: Marc
 *
 *
 *      Version du logiciel
 *
 *      0.1   MN    28.08.20   Première série de 20 pièces
 *
 *      0.2   MN    13.10.20   Première série 50 pièces, 2 flash et pas d'allumage avant.
 *
 *      0.3   MN    02.11.20   Ajoute de la fonction d'arrêt
 *
 *      0.4   MN    12.02.20   Delais de 10s au lieu de 2s
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

//#define DEFAULT_RF_VERSION EU_VERSION
#define DEFAULT_RF_VERSION US_VERSION

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

#define DEV_ID         {0,0,200,85}

#define LOCK_MIN_SEC    2 // secondes

#define TICK_PER_SECOND 2
#define LOCK_MAX_SEC    (10*60)
#define PROG_MAX_DELAY  (5*TICK_PER_SECOND) // s après on prend le setup du temps

#define DEFAULT_LOCK_SEC   10 // seconds, default button lock time
#define PROG_BLANKING   10 // Temps de blocage après programmation
#define PROG_MIN_TIME   (2*TICK_PER_SECOND)
#define PROG_MAX_TIME   (10*60*TICK_PER_SECOND)

#define PROG_ENTER_LOW  (5*TICK_PER_SECOND)
#define PROG_ENTER_HIGH (8*TICK_PER_SECOND)

#define NB_SUCCESSIVE   (1) // nombre de mesure identique successives pour prendre un vote
#define RESEND_TIME     (10 * TICK_PER_SECOND) // Temps entre les envois (1/2 s)

#define MIN_OVERRIDE    (30) // Prend le min=valeur après ce nombre de fois sans corriger le min
#define MAX_TIME_LED_ON (10*TICK_PER_SECOND)

#define USE_SPI         // Ne pénalise pas la conso si pas d'envoi
#define LED_MODE     2  // 1 for direct, 2 for PWM
#define PROC_SPEED   1  // 1 400kHz, 2 8Mhz
//#define DEBUG_LED

#endif /* DEF_H_ */

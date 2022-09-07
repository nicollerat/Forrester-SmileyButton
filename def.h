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
 *      0.1   MN    28.08.20   Premi�re s�rie de 20 pi�ces
 *
 *      0.2   MN    13.10.20   Premi�re s�rie 50 pi�ces, 2 flash et pas d'allumage avant.
 *
 *      0.3   MN    02.11.20   Ajoute de la fonction d'arr�t
 *
 *      0.4   MN    12.02.21   Delais de 10s au lieu de 2s
 *
 *      0.5   MN    09.04.21   Ajout proc�dure de test
 *                      Le test du bouton dans hwBackground passe par un compteur de temps
 *                            On attend qu'il soit rel�ch�, si entre 2 et 5s, on met en test
 *                            si on a plus que 5s, on d�marre normalement
 *
 *                      En test, le temps de bloquage est r�duit au minimum, on s'arr�te apr�s 30s
 *
 *      0.6   MN    26.05.21 Flash des LEDs � la fin de la p�riode
                        Correction programmation fr�quence

        0.7   MN    03.06.21 Permet la programmation en mode test
                            Le temps est suspendu pendant que le mode prog est actif
                            Le d�lais de mise en arr�t est red�marr� � la fin

        0.8   MN    16.09.21 Ajout d'un crit�re pour la d�tection bas� sur l'�volution commune de l'offset
                             Permet d'�viter les d�clenchements impromptus de votes lors de l'allumage de la lumi�re

************************************************************ NOUVEAU HARDWARE
************************************************************
        1.0   MN    1.04.22 Premi�re version avec bouttons

        1.1   MN    26.05.22  Arrangements pour les configurations seuil et fr�quence � partir de ce fichier def.h
                              l�g�re diff�rence entre les p�riodes des 3 capteurs pour �viter les votes cons�cutifs

        1.2   MN    9.6.22  R�surrection de l'algorithme avec le changement d'offset

        1.3   MN    15.6.22  Algo minimum revu pour tenir compte pic n�gatif.
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

// Canaux de base pour les r�gions
    // Formule CH=(freq/2 -422.4)*10
#define EURO_CHANNEL    108  // 866.4 MHz
#define US_CHANNEL      326  // 910 MHz
#define ASIA_CHANNEL    391  // 923 MHz

    // Fr�quences suppl�mentaires donn�e par JGC
#define MAX_CHANNEL     495  // 943.8 MHz
#define MIN_CHANNEL     0    // 844.8 MHz
#define MID_CHANNEL     216  // 888 MHz

// Niveau de puissance
#define POWER_BYTE_VERY_LOW 0x02 // -10dBm
#define POWER_BYTE_LOW    0x06 // -2dBm
#define POWER_BYTE_MID    0x0A // 6 dBm
#define POWER_BYTE_HIGH   0x0E // 10 dBm

#define DEV_ID         {120,30,100,85}

#define LOCK_MIN_SEC    2 // secondes

// Define the period
// can handle 2 and 4. See function si115x_init_1CH()
#define TICK_PER_SECOND 4

#define LED_CURRENT 2  // 1:390mA, 2:100mA, 3:50mA

#if LED_CURRENT==1
#define THR_SET         60
#define THR_OFFSET      20 // Originally 5
#else
#define THR_SET         40
#define THR_OFFSET      20 // Originally 5
#endif

// Debug stuff
#define USE_UART 1
#define DEBUG_LED 0

#define LOCK_MAX_SEC    (10*60)
#define PROG_MAX_DELAY  (5*TICK_PER_SECOND) // s apr�s on prend le setup du temps

#define DEFAULT_LOCK_SEC   2 // seconds, default button lock time (10)
#define PROG_BLANKING   10 // Temps de blocage apr�s programmation
#define PROG_MIN_TIME   (2*TICK_PER_SECOND)
#define PROG_MAX_TIME   (10*60*TICK_PER_SECOND)

#define PROG_ENTER_LOW  (5*TICK_PER_SECOND)
#define PROG_ENTER_HIGH (8*TICK_PER_SECOND)

#define NB_SUCCESSIVE   (1) // nombre de mesure identique successives pour prendre un vote
#define RESEND_TIME     (10 * TICK_PER_SECOND) // Temps entre les envois (1/2 s)

#define MIN_OVERRIDE    (30 * TICK_PER_SECOND) // Prend le min=valeur apr�s ce nombre de fois sans corriger le min
#define MAX_TIME_LED_ON (10*TICK_PER_SECOND)

#define USE_SPI         // Ne p�nalise pas la conso si pas d'envoi
#define LED_MODE     2  // 1 for direct, 2 for PWM
#define PROC_SPEED   1  // 1 400kHz, 2 8Mhz
//#define DEBUG_LED

#endif /* DEF_H_ */

/*
 * nrf905.c
 *
 *  Created on: 29 juil. 2020
 *      Author: Marc
 */

#include "main.h"
#include "hwspi.h"
#include "nrf905.h"
#include "buttons.h"
#include <stdint.h>
#include "def.h"
#include "hw.h"
#include "hwspi.h"

#define NRF_OUT_PORT  P3OUT
#define NRF_IN_PORT   P3IN
#define NRF_PULLUP    P3REN

#define NRF_PWR_UP    GPIO_PIN0
#define NRF_POWER     GPIO_PIN2
#define NRF_DR        GPIO_PIN3
#define NRF_EN        GPIO_PIN4
#define NRF_TRX_CE    GPIO_PIN5
#define NRF_TX_EN     GPIO_PIN6

void nrfEnableRFTransmit(bool en)
{
    if (en)
        NRF_OUT_PORT |= NRF_TX_EN; // TX_EN
    else
        NRF_OUT_PORT &= ~NRF_TX_EN; // TX_EN
}

void nrfEnableRFRadio(bool en)
{
    if (en)
        NRF_OUT_PORT |= NRF_TRX_CE; // NRF_TRX_CE
    else
        NRF_OUT_PORT &= ~NRF_TRX_CE; // NRF_TRX_CE
}

// Retourne vrai si l'envoi est fini
bool nrfReadDR()
{
    return (NRF_IN_PORT & NRF_DR)!=0;
}

// Retourne vrai si l'envoi est fini
bool nrfReadCD()
{
    //return (NRF_IN_PORT & NRF_CD)!=0;
    return false ; //la ligne n'est pas connectée
}

void nrfCSLow()
{
    NRF_OUT_PORT &= ~NRF_EN; // CS
}

void nrfCSHigh()
{
    NRF_OUT_PORT |= NRF_EN; // CS
}

void nrfPowerNRF(bool on)
{
    if (on) {
        NRF_OUT_PORT &= ~NRF_POWER; // Transistor ON
        NRF_OUT_PORT |= NRF_PWR_UP; // PWR_UP
        NRF_OUT_PORT |= NRF_EN; // CS
        NRF_OUT_PORT &= ~NRF_TX_EN; // TX_EN
        NRF_OUT_PORT &= ~NRF_TRX_CE; // Enable radio
        NRF_PULLUP |= NRF_DR; // Pull-up DR

        hwspiEnable();
    } else {
        hwspiDisable();

        NRF_PULLUP &= ~NRF_DR; // désactive le Pull-up DR
        NRF_OUT_PORT &= ~NRF_EN; // CS
        NRF_OUT_PORT &= ~NRF_TX_EN; // TX_EN
        NRF_OUT_PORT &= ~NRF_TRX_CE; // Enable radio
        NRF_OUT_PORT &= ~NRF_PWR_UP; // PWR_UP
        NRF_OUT_PORT |= NRF_POWER; // Transistor OFF
    }
}

// structure des votes envoyé par les smiley box
typedef struct {
    union {
        struct {
            uint8_t f;       // == 0x0C
            uint8_t ident[4];  // ID du smiley
            uint8_t l;       // ??
            uint8_t uniq;    // Compteur et indication bouton
            uint8_t vote2:4; // valeur du vote
            uint8_t vote1:4; // valeur du vote
            uint8_t vote4:4; // n'existe pas
            uint8_t vote3:4; // valeur du vote
            };
        uint8_t bytes[16]; // taille espérée
    };
} tSFDMessageRF;



typedef struct {
    uint16_t freq;
    uint8_t  power;
} tRFSetup;

const tRFSetup tabRFSetup[] = {
    {EURO_CHANNEL, POWER_BYTE_MID},
    {US_CHANNEL, POWER_BYTE_LOW},
    {ASIA_CHANNEL, POWER_BYTE_MID},
    {MIN_CHANNEL, POWER_BYTE_MID},
    {MAX_CHANNEL, POWER_BYTE_MID},
    {MID_CHANNEL, POWER_BYTE_MID}
};


// Prépare et envoie les données par radio
//   Met à jour mSending
void nrfSendData()
{
    hwDebLedOn(1);

    int t;
    char data[32];
    tSFDMessageRF * m = (tSFDMessageRF *)&data[1];

    // Prépare les données
    data[0] = 0x20; // Addresse TX payload
    m->f = 0x0C;
    m->ident[0] = devID[3];
    m->ident[1] = devID[2];
    m->ident[2] = devID[1];
    m->ident[3] = devID[0];
    m->l = 0;
    uint8_t bt=0;
    if (repeatG>0) {
        m->vote1 = ((4-repeatG)<<1) | 1;
        bt|=0x4;
        repeatG--;
    } else {
        m->vote1 = 0;
    }
    if (repeatM>0) {
        m->vote2 = ((4-repeatM)<<1) | 1;
        bt|=0x2;
        repeatM--;
    } else {
        m->vote2 = 0;
    }
    if (repeatD>0) {
        m->vote3 = ((4-repeatD)<<1) | 1;
        bt|=0x1;
        repeatD--;
    } else {
        m->vote3 = 0;
    }
    m->vote4=0;

    m->uniq = totalKey<<3 | bt;

    // Power up du chip, attente 3 ms
    nrfPowerNRF(true);
    mDelay_us(3000);

    // Ecrit les données dans le chip
    //  -- CONFIGURATION
    uint8_t init0[] = {
        0x00, // adresse du registre de  configuration
        0x6C, // channel
        // this one is changed later...
        0x00, // [7:6] unused, [5]AUTO=0, [4]RX RED POWER=0, [3:2]PA_PWR=3 (+10dBm), [1]HF_PLL=1, [0]CH[8]=0
        0x44, // bit[7] not used, TX_AFW[2:0] , bit[3] not used, RX_AFW[2:0]=4 (4 bytes addr)
        0x0B, // bit[7:6] not used, RX_PW[5:0]
        0x0B, // bit[7:6] not used, TX_PW[5:0] number of bytes sent (11))
        0xE7,0xE7,0xE7,0xE7, // RX_ADDRESS
        0xD8  }; // 11011000  CRC_MODE=1,CRC_EN=1, XOF[2:0]=011(16MHz), UP_CLK_EN=0, UP_CLK_FREQ[1:0]=0

    init0[1]=tabRFSetup[RFsetup].freq;
    init0[2]=(tabRFSetup[RFsetup].power&0xFE) | (tabRFSetup[RFsetup].freq>>8);

    nrfCSLow();
    hwspiTransmission(init0, 0, sizeof(init0));
    nrfCSHigh();
    mDelay_us(10);

    // -- DONNEES
    nrfCSLow();
    hwspiTransmission(data, 0, sizeof(data));
    nrfCSHigh();

    nrfEnableRFRadio(true);

    mDelay_us(1000);
    // LBT, première attente si déjà actif
    for(t=0;t<10;t++) {
        if (!nrfReadCD()) break;
        //hwSetLED(LEDM);
        mDelay_us(1000);
        //hwClearLED(LEDM);
    }

    // Envoie les données, TRX_CE pulse de 10uS min
    nrfEnableRFTransmit(true);

    // Attente de la fin de l'envoi
    while(!nrfReadDR()) ;

    // Arrête le chip
    nrfPowerNRF(false);

    // Met à jour les flags
    if ((repeatD==0) && (repeatM==0) && (repeatG==0)) {
        mSending=false;
    } else {
        mSending=true;
    }
    hwDebLedOff(1);
}

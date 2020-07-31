/*
 * si115x.c
 *
 *  Created on: 5 juil. 2020
 *      Author: loic
 */


#include "si115x_functions.h"
#include "si115x.h"

 uint8_t devPartID=0;
 uint8_t devHWID=0;
 uint8_t devREVID=0;
 uint8_t devResp0=0;

int16_t si115x_init_3CH( HANDLE*si115x_handle )
{
    int16_t    retval;

    // Vu sur l'analyse de la démo
    Si115xWriteToRegister(si115x_handle, SI115x_REG_COMMAND, CMD_PAUSE_CH);

    retval = Si115xReset(si115x_handle );
    retval = Si115xWriteToRegister(si115x_handle, SI115x_REG_COMMAND, 0); // Reset du compteur CNT_R

    devResp0 = Si115xReadFromRegister(si115x_handle,  SI115x_REG_RESPONSE0);
    //__delay_cycles(2000);

    devPartID = Si115xReadFromRegister(si115x_handle, SI115x_REG_PART_ID);
    devHWID = Si115xReadFromRegister(si115x_handle, SI115x_REG_REV_ID);
    devREVID = Si115xReadFromRegister(si115x_handle, SI115x_REG_MFR_ID);

    // Proposé par le soft de test
    switch(2){
    case 1: // high current
        retval += Si115xParamSet(si115x_handle, PARAM_LED1_A, 0x3f);
        retval += Si115xParamSet(si115x_handle, PARAM_LED2_A, 0x3f);
        retval += Si115xParamSet(si115x_handle, PARAM_LED3_A, 0x3f);
        break;
    case 2: // mid current 100mA
        retval += Si115xParamSet(si115x_handle, PARAM_LED1_A, 0x2A);
        retval += Si115xParamSet(si115x_handle, PARAM_LED2_A, 0x2A);
        retval += Si115xParamSet(si115x_handle, PARAM_LED3_A, 0x2A);
        break;
    case 3: // low current 50mA
        retval += Si115xParamSet(si115x_handle, PARAM_LED1_A, 0x12);
        retval += Si115xParamSet(si115x_handle, PARAM_LED2_A, 0x12);
        retval += Si115xParamSet(si115x_handle, PARAM_LED3_A, 0x12);
        break;
    }
    retval += Si115xParamSet(si115x_handle, PARAM_CH_LIST, 0x07);
    retval += Si115xParamSet(si115x_handle, PARAM_ADCCONFIG0, 0x62);
    retval += Si115xParamSet(si115x_handle, PARAM_MEASCONFIG0, 0x61);
    retval += Si115xParamSet(si115x_handle, PARAM_ADCCONFIG1, 0x62);
    retval += Si115xParamSet(si115x_handle, PARAM_MEASCONFIG1, 0x62);
    retval += Si115xParamSet(si115x_handle, PARAM_ADCCONFIG2, 0x62);
    retval += Si115xParamSet(si115x_handle, PARAM_MEASCONFIG2, 0x64);

    // Ajouté pour faire marcher...
    retval += Si115xParamSet(si115x_handle, PARAM_MEASCOUNT0, 0x1);

    switch(4) {
    case 1: // Rapide 20ms
        retval += Si115xParamSet(si115x_handle, PARAM_MEASRATE_L, 0x19);
        break;


    case 2: // moyen 100ms
        retval += Si115xParamSet(si115x_handle, PARAM_MEASRATE_L, 0xFA);
        break;

    case 3: // moyen 250ms
        retval += Si115xParamSet( si115x_handle, PARAM_MEASRATE_H, 0x01);
        retval += Si115xParamSet( si115x_handle, PARAM_MEASRATE_L, 0x38);
        break;

    case 4: // Lent 500ms
        retval += Si115xParamSet( si115x_handle, PARAM_MEASRATE_H, 0x02);
        retval += Si115xParamSet( si115x_handle, PARAM_MEASRATE_L, 0x71);
        break;

    case 5: // Très lent 1s
        retval += Si115xParamSet( si115x_handle, PARAM_MEASRATE_H, 0x04);
        retval += Si115xParamSet( si115x_handle, PARAM_MEASRATE_L, 0xe2);
    }

    retval += Si115xWriteToRegister(si115x_handle, SI115x_REG_IRQ_ENABLE, 0x07);


    return retval;
}

// Initialise le capteur pour un seul canal
int16_t si115x_init_1CH( HANDLE*si115x_handle )
{
    int16_t    retval;

    // Vu sur l'analyse de la démo
    Si115xWriteToRegister(si115x_handle, SI115x_REG_COMMAND, CMD_PAUSE_CH);

    retval  = Si115xReset( si115x_handle );

    devPartID = Si115xReadFromRegister(si115x_handle, SI115x_REG_PART_ID);
    devHWID = Si115xReadFromRegister(si115x_handle, SI115x_REG_REV_ID);
    devREVID = Si115xReadFromRegister(si115x_handle, SI115x_REG_MFR_ID);

    // Proposé par le soft de test
    switch(1){
    case 1: // high current
        retval += Si115xParamSet(si115x_handle, PARAM_LED1_A, 0x3f);
        break;
    case 2: // mid current 100mA
        retval += Si115xParamSet(si115x_handle, PARAM_LED1_A, 0x2A);
        break;
    case 3: // low current 50mA
        retval += Si115xParamSet(si115x_handle, PARAM_LED1_A, 0x12);
        break;
    }

    retval += Si115xParamSet( si115x_handle, PARAM_CH_LIST, 0x01);
    retval += Si115xParamSet( si115x_handle, PARAM_ADCCONFIG0, 0x62);

    switch(1) {
    case 1: // Une mesure de 24us
        retval += Si115xParamSet( si115x_handle, PARAM_ADCSENS0, 0x80);
        break;

    case 2:// 4 mesures (96us)
        retval += Si115xParamSet( si115x_handle, PARAM_ADCSENS0, 0x82);
        break;
    }

    retval += Si115xParamSet( si115x_handle, PARAM_MEASCONFIG0, 0x61);

    switch(4) {
     case 1: // Rapide 20ms
         retval += Si115xParamSet(si115x_handle, PARAM_MEASRATE_L, 0x19);
         break;


     case 2: // moyen 100ms
         retval += Si115xParamSet(si115x_handle, PARAM_MEASRATE_L, 0xFA);
         break;

     case 3: // moyen 250ms
         retval += Si115xParamSet( si115x_handle, PARAM_MEASRATE_H, 0x01);
         retval += Si115xParamSet( si115x_handle, PARAM_MEASRATE_L, 0x38);
         break;

     case 4: // Lent 500ms
         retval += Si115xParamSet( si115x_handle, PARAM_MEASRATE_H, 0x02);
         retval += Si115xParamSet( si115x_handle, PARAM_MEASRATE_L, 0x71);
         break;

     case 5: // Très lent 1s
         retval += Si115xParamSet( si115x_handle, PARAM_MEASRATE_H, 0x04);
         retval += Si115xParamSet( si115x_handle, PARAM_MEASRATE_L, 0xe2);
     }

    retval += Si115xWriteToRegister( si115x_handle, SI115x_REG_IRQ_ENABLE, 0x01);

    return retval;
}



uint8_t st;
uint8_t irq;
uint8_t irq_en;

void si115x_GetMeasure(HANDLE *handle, SI115X_SAMPLES *samples)
{
    irq_en=i2c_read(handle->slave_addr, SI115x_REG_IRQ_ENABLE);
    Si115xWriteToRegister(handle, SI115x_REG_COMMAND, CMD_FORCE_CH);

    st=i2c_read(handle->slave_addr, SI115x_REG_RESPONSE0);
    irq=i2c_read(handle->slave_addr, SI115x_REG_IRQ_STATUS);
    st=i2c_read(handle->slave_addr, SI115x_REG_RESPONSE0);
    irq=i2c_read(handle->slave_addr, SI115x_REG_IRQ_STATUS);
    st=i2c_read(handle->slave_addr, SI115x_REG_RESPONSE0);
    irq=i2c_read(handle->slave_addr, SI115x_REG_IRQ_STATUS);
}


void si115x_handler(HANDLE *si115x_handle, SI115X_SAMPLES *samples)
{
    uint8_t buffer[10];
    Si115xBlockRead( si115x_handle,
                     SI115x_REG_IRQ_STATUS,
                      7,
                      buffer);
    samples->irq_status = buffer[0];
    samples->ch0  = buffer[1] <<  8;
    samples->ch0 |= buffer[2];
    samples->ch1  = buffer[3] <<  8;
    samples->ch1 |= buffer[4];
    samples->ch2  = buffer[5] <<  8;
    samples->ch2 |= buffer[6];
}


/*
 * hwi2c.c
 *
 *  Created on: 10 juil. 2020
 *      Author: loic
 */

#include "hwi2c.h"

#define HANDLE I2C_HANDLE

void hwReadI2CB0(HANDLE*handle, uint8_t address, uint8_t * data, int len);
void hwInitI2CB1(HANDLE*handle);

void hwInitI2C(HANDLE*handle)
{
    if(handle->isA)
    {
        hwInitI2CA0(handle);
    }

    else
    {
        if (handle->BASE==EUSCI_B1_BASE) hwInitI2CB1(handle);
        else hwInitI2CB0(handle);
    }
}

void hwInitI2CB0(HANDLE*handle)
{
    GPIO_setAsOutputPin(GPIO_PORT_UCB0SCL, GPIO_PIN_UCB0SCL);
    __delay_cycles(1000);
    GPIO_setOutputLowOnPin(GPIO_PORT_UCB0SCL, GPIO_PIN_UCB0SCL);
    __delay_cycles(1000);
    GPIO_setOutputHighOnPin(GPIO_PORT_UCB0SCL, GPIO_PIN_UCB0SCL);
    __delay_cycles(1000);

    // Configurer les Pins pour le I2C (Variables def. dans hwi2c.h)
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_UCB0SCL,
        GPIO_PIN_UCB0SCL,
        GPIO_FUNCTION_UCB0SCL
    );
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_UCB0SDA,
        GPIO_PIN_UCB0SDA,
        GPIO_FUNCTION_UCB0SDA
    );


    //Initialisation selon exemple

    EUSCI_B_I2C_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = CS_getSMCLK();
    param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_400KBPS;
    param.byteCounterThreshold = 0;
    param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;
    EUSCI_B_I2C_initMaster(handle->BASE, &param);

    EUSCI_B_I2C_enable(handle->BASE);

}


void hwInitI2CB1(HANDLE*handle)
{

    GPIO_setAsOutputPin(GPIO_PORT_UCB1SCL, GPIO_PIN_UCB1SCL);
    __delay_cycles(1000);
    GPIO_setOutputLowOnPin(GPIO_PORT_UCB1SCL, GPIO_PIN_UCB1SCL);
    __delay_cycles(1000);
    GPIO_setOutputHighOnPin(GPIO_PORT_UCB1SCL, GPIO_PIN_UCB1SCL);
    __delay_cycles(1000);


    // Configurer les Pins pour le I2C (Variables def. dans hwi2c.h)
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_UCB1SCL,
        GPIO_PIN_UCB1SCL,
        GPIO_FUNCTION_UCB1SCL
    );
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_UCB1SDA,
        GPIO_PIN_UCB1SDA,
        GPIO_FUNCTION_UCB1SDA
    );

    //Initialisation selon exemple

    EUSCI_B_I2C_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = CS_getSMCLK();
    param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_400KBPS;
    param.byteCounterThreshold = 0;
    param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;
    EUSCI_B_I2C_initMaster(EUSCI_B1_BASE, &param);

    EUSCI_B_I2C_enable(handle->BASE);
}

/*
 * Fonction de redirection vers B0 / A0
 *
 */

int16_t hwSendI2C(HANDLE *handle, uint8_t * data, int len)
{
    int16_t ret;
    if(handle->isA)
    {
        hwSendI2CA(handle, data, len);
    }

    else
    {
        ret=hwSendI2CB0(handle, data, len);
    }
    return ret;
}

int16_t hwSendI2CB0(HANDLE *handle, uint8_t * data, int len)
{
    int16_t ret;
    //définir mode (transmit) et slave addr.

    EUSCI_B_I2C_setSlaveAddress(handle->BASE,
           handle->slave_addr
            );

    EUSCI_B_I2C_setMode(handle->BASE,
            EUSCI_B_I2C_TRANSMIT_MODE
            );

    EUSCI_B_I2C_enable(handle->BASE);



    while (EUSCI_B_I2C_SENDING_STOP == EUSCI_B_I2C_masterIsStopSent(EUSCI_B0_BASE));


    if(len > 1) //cas pour plusieurs Bytes
    {
        // TODO cause une interrupt ? HWREG16(handle->BASE + OFS_UCBxIE) |= UCNACKIE;
        // EUSCI_B_I2C_masterSendMultiByteStart(handle->BASE, data[0]);

        //Send start condition.
        HWREG16(handle->BASE + OFS_UCBxCTLW0) |= UCTR +  UCTXSTT;

        //Poll for transmit interrupt flag.
        while (!(HWREG16(handle->BASE + OFS_UCBxIFG) & (UCTXIFG|UCNACKIE))) ;

        //Send single byte data.
//        HWREG16(handle->BASE + OFS_UCBxTXBUF) = txData;

        //Poll for transmit interrupt flag.
//        while (!(HWREG16(handle->BASE + OFS_UCBxIFG) & (UCTXIFG| UCNACKIE))) ;

        if (HWREG16(handle->BASE + OFS_UCBxIFG) & ( UCNACKIE )) {
            HWREG16(handle->BASE + OFS_UCB0CTLW0) |= UCTXSTP;
            return -1;
        }

        unsigned int i = 0;

        for(i = 0; i<len; i++)
        {
            // EUSCI_B_I2C_masterSendMultiByteNext(handle->BASE, data[i]);
            while (!(HWREG16(handle->BASE + OFS_UCBxIFG) & UCTXIFG)) ;

            //Send single byte data.
            HWREG16(handle->BASE + OFS_UCBxTXBUF) = data[i];
        }

        while (!(HWREG16(handle->BASE + OFS_UCBxIFG) & (UCTXIFG| UCNACKIE))) ;
        EUSCI_B_I2C_masterSendMultiByteStop(handle->BASE);

    }

    else if(len == 1) //cas pour un seul byte
    {

        // TODO cause une interrupt ? HWREG16(handle->BASE + OFS_UCBxIE) |= UCNACKIE;

        EUSCI_B_I2C_masterSendSingleByte(handle->BASE, data[0]);

        if (HWREG16(handle->BASE + OFS_UCBxIFG) & UCNACKIE) {
            HWREG16(handle->BASE + OFS_UCB0CTLW0) |= UCTXSTP;
            return -1;
        }

    }

    return 0;
}

void hwSendI2CA(HANDLE *handle, uint8_t * data, int len)
{
    //TODO: Write pour le port A
}

/*
 * Fonction de redirection vers B0 / A0
 *
 */

void hwReadI2C(HANDLE*handle, uint8_t address, uint8_t * data, int len)
{
    if(handle->isA)
    {
        hwReadI2CB0(handle, address, data, len);
    }

    else
    {
        hwReadI2CB0(handle,address, data, len);
    }
}

void hwInitI2CA0(HANDLE*handle)
{
    //TODO: Init A0
}

// Exemple sans accès à driverlib
unsigned char i2c_read_B0(unsigned char slv_addr, unsigned char reg_addr)
{

    unsigned char data = 0;

    while(UCB0STAT & UCBBUSY);

    // Définit l'adresse du slave
    UCB0I2CSA = slv_addr;
    // TODO cause une interrupt ? UCB0IE |= UCNACKIE;

    // Transmet le start
    UCB0CTLW0 |= UCTR | UCTXSTT;

    while(UCB0CTLW0 & UCTXSTT);

    // Transmet l'adresse du registre
    UCB0TXBUF = reg_addr;
    while(!(UCB0IFG & (UCTXIFG0 | UCNACKIE)));// Attend une interrupt

    if (UCB0IFG & UCNACKIE) { // Le device ne répond pas
        UCB0CTLW0 |= UCTXSTP; // Transmet un stop
        return 0;
    }
    // Met en lecture
    UCB0CTLW0 &= ~UCTR;

    // transmet l'adress et start
    UCB0CTLW0 |= UCTXSTT;

    while(UCB0CTLW0 & UCTXSTT);

    // Transmet un STOP
    UCB0CTLW0 |= UCTXSTP;

    while(!(UCB0IFG & UCRXIFG0)); // Attend une interrupt

    // Récupère la donnée
    data = UCB0RXBUF;

    while(UCB0CTLW0 & UCTXSTP);

    return data;
}

// Exemple sans accès à driverlib
unsigned char i2c_read_B1(unsigned char slv_addr, unsigned char reg_addr)
{

    unsigned char data = 0;

    while(UCB1STAT & UCBBUSY);

    // Définit l'adresse du slave
    UCB1I2CSA = slv_addr;
    // TODO cause une interrupt ? UCB1IE |= UCNACKIE;

    // Transmet le start
    UCB1CTLW0 |= UCTR | UCTXSTT;

    while(UCB1CTLW0 & UCTXSTT);

    // Transmet l'adresse du registre
    UCB1TXBUF = reg_addr;
    while(!(UCB1IFG & (UCTXIFG0 | UCNACKIE)));// Attend une interrupt

    if (UCB1IFG & UCNACKIE) { // Le device ne répond pas
           UCB1CTLW0 |= UCTXSTP; // Transmet un stop
           return 0;
    }

    // Met en lecture
    UCB1CTLW0 &= ~UCTR;

    // transmet l'adress et start
    UCB1CTLW0 |= UCTXSTT;

    while(UCB1CTLW0 & UCTXSTT);

    // Transmet un STOP
    UCB1CTLW0 |= UCTXSTP;

    while(!(UCB1IFG & UCRXIFG0)); // Attend une interrupt

    // Récupère la donnée
    data = UCB1RXBUF;

    while(UCB1CTLW0 & UCTXSTP);

    return data;
}

uint8_t EUSCI_B_I2C_masterReceiveMultiByteFinishNack (uint16_t baseAddress)
{
    //Send stop condition.
    HWREG16(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP | UCTXNACK;

    //Wait for Stop to finish
    while (HWREG16(baseAddress + OFS_UCBxCTLW0) & UCTXSTP)

        /*
    // Wait for RX buffer
    while (!(HWREG16(baseAddress + OFS_UCBxIFG) & UCRXIFG)) ;
*/
    //Capture data from receive buffer after setting stop bit due to
    //MSP430 I2C critical timing.
    return (HWREG16(baseAddress + OFS_UCBxRXBUF));
}

uint16_t ifg_val[16]={0};
int c1=0,c2=0,c3=0,c4=0,c5=0;

void hwReadI2CB0(HANDLE*handle, uint8_t address, uint8_t * data, int len)
{
    if (len==1) {
        if (handle->BASE==EUSCI_B1_BASE)
            data[0]=i2c_read_B1(handle->slave_addr, address);
        else
            data[0]=i2c_read_B0(handle->slave_addr, address);
    } else {
        switch(2) {
        case 1:
            while(UCB0STAT & UCBBUSY);

            // Définit l'adresse du slave
            UCB0I2CSA = handle->slave_addr;

            // Transmet le start
            UCB0CTLW0 |= UCTR | UCTXSTT;

            while(UCB0CTLW0 & UCTXSTT);

            // Transmet l'adresse du registre
            UCB0TXBUF = address;
            while(!(UCB0IFG & UCTXIFG0));// Attend une interrupt

            // Met en lecture
            UCB0CTLW0 &= ~UCTR;

            // transmet l'adress et start
            UCB0CTLW0 |= UCTXSTT;

            while(UCB0CTLW0 & UCTXSTT);

            // lit un mot dans le vide
            len--;
            uint8_t bidon=UCB0RXBUF;
            while(UCB0CTLW0 & UCTXSTT);

            while(--len) {
                // Récupère des données
                *data++ = UCB0RXBUF;
                while(UCB0CTLW0 & UCTXSTT);
            }

            // Transmet un STOP
            UCB0CTLW0 |= UCTXSTP;

            while(!(UCB0IFG & UCRXIFG0)); // Attend une interrupt

            // Récupère la donnée
            *data++ = UCB0RXBUF;

            while(UCB0CTLW0 & UCTXSTP);
            break;

        case 2:
        {
            uint16_t * pIfg = ifg_val;
            c1=0;c2=0;c3=0;c4=0;c5=0;

            // TODO faire marcher ça
            *pIfg++=HWREG16(handle->BASE + OFS_UCBxIFG);
            //def. mode (receive) + slave addr.

            //Set the address of the slave with which the master will communicate.
            HWREG16(handle->BASE + OFS_UCBxI2CSA) = handle->slave_addr;

            //Send start condition.
            HWREG16(handle->BASE + OFS_UCBxCTLW0) |= UCTR +  UCTXSTT;

            //Poll for transmit interrupt flag.
            while ((HWREG16(handle->BASE + OFS_UCBxCTLW0) & UCTXSTT)) c1++;

            //Send single byte data.
            HWREG16(handle->BASE + OFS_UCBxTXBUF) = address;

            //Poll for transmit interrupt flag.
            while (!(HWREG16(handle->BASE + OFS_UCBxIFG) & (UCTXIFG0 | UCNACKIE))) c2++;

            if (HWREG16(handle->BASE + OFS_UCBxIFG) & (UCNACKIE)) {
                return;
            }

            *pIfg++=HWREG16(handle->BASE + OFS_UCBxIFG);


//            EUSCI_B_I2C_masterReceiveStart(handle->BASE);
            //Set USCI in Receive mode
            HWREG16(handle->BASE + OFS_UCBxCTLW0) &= ~UCTR;
            //Send start
            HWREG16(handle->BASE + OFS_UCBxCTLW0) |= UCTXSTT;

            *pIfg++=HWREG16(handle->BASE + OFS_UCBxIFG);

            //Poll for receive interrupt flag.
//            while (!(HWREG16(handle->BASE + OFS_UCBxIFG) & UCRXIFG)) ;
            while ((HWREG16(handle->BASE + OFS_UCBxCTLW0) & UCTXSTT)) c3++;

            *pIfg++=HWREG16(handle->BASE + OFS_UCBxIFG);

            unsigned int i = 0;
            // Lit un dans le vide
            //EUSCI_B_I2C_masterReceiveMultiByteNext(handle->BASE);
            //while (!(HWREG16(handle->BASE + OFS_UCBxIFG)& UCRXIFG)) ;

            // Lit dans le vide
            HWREG16(handle->BASE + OFS_UCBxRXBUF);

            for(i = 0; i<len-1; i++)
            {
                //data[i] = EUSCI_B_I2C_masterReceiveMultiByteNext(handle->BASE);
                //Poll for receive interrupt flag.
                *pIfg++=HWREG16(handle->BASE + OFS_UCBxIFG);
                while (!(HWREG16(handle->BASE + OFS_UCBxIFG)& UCRXIFG)) c4++;
                data[i] = HWREG16(handle->BASE + OFS_UCBxRXBUF);
            }

            *pIfg++=HWREG16(handle->BASE + OFS_UCBxIFG);

            while (!(HWREG16(handle->BASE + OFS_UCBxIFG)& UCRXIFG)) c4++;

//            data[i++]=HWREG16(handle->BASE + OFS_UCBxRXBUF);

            //data[i++]=EUSCI_B_I2C_masterReceiveMultiByteFinishNack(handle->BASE);
            //Send stop condition.
            HWREG16(handle->BASE + OFS_UCBxCTLW0) |= UCTXSTP ;
            data[i]=HWREG16(handle->BASE + OFS_UCBxRXBUF);

            //Wait for Stop to finish
            while (( HWREG16(handle->BASE + OFS_UCBxCTLW0) & UCTXSTP)!=0)  c5++;

            /*
            *pIfg++=HWREG16(handle->BASE + OFS_UCBxIFG);

            data[i]=HWREG16(handle->BASE + OFS_UCBxRXBUF);

            *pIfg++=HWREG16(handle->BASE + OFS_UCBxIFG);
            */
        }
            break;
        }
    }


}

void hwReadI2CA0(HANDLE*handle, uint8_t address, uint8_t * data, int len)
{
    //TODO: Read pour le port A
}

int hwUCB0Cnt=0;

#pragma vector = EUSCI_B0_VECTOR
__interrupt void intUCB0(void)
{
// this is a trap ISR - check for the interrupt cause here by
// checking the interrupt flags, if necessary also clear the interrupt
// flag
    hwUCB0Cnt++;
}


int hwUCB1Cnt=0;

#pragma vector = EUSCI_B1_VECTOR
__interrupt void intUCB1(void)
{
// this is a trap ISR - check for the interrupt cause here by
// checking the interrupt flags, if necessary also clear the interrupt
// flag
    hwUCB1Cnt++;
}

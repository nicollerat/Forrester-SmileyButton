/*
 * hauart.c
 *
 *  Created on: 9 juin 2022
 *      Author: Marc
 */



#include "hwuart.h"
#include <driverlib.h>

#define UART_TXD_PORT        GPIO_PORT_P1
#define UART_TXD_PIN         GPIO_PIN7
#define UART_RXD_PORT        GPIO_PORT_P1
#define UART_RXD_PIN         GPIO_PIN5
#define UART_SELECT_FUNCTION GPIO_PRIMARY_MODULE_FUNCTION

void hwuart_Init()
{
    // Configure UCA0TXD and UCA0RXD
    GPIO_setAsPeripheralModuleFunctionOutputPin(UART_TXD_PORT, UART_TXD_PIN, UART_SELECT_FUNCTION);
    //GPIO_setAsPeripheralModuleFunctionInputPin(UART_RXD_PORT, UART_RXD_PIN, UART_SELECT_FUNCTION);

    // Configure UART @9600 baud
    // http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
    EUSCI_A_UART_initParam param = {0};
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = CS_getSMCLK()/ 9600 / 32; // 19200
    param.firstModReg = 2;
    param.secondModReg = 182;
    param.parity = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;
    param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);
}

void hwuart_Send(char * str)
{

    int i = 0;
    for(i = 0; i < MAX_STRBUF_SIZE; i++)
    {
        if (str[i] != 0)
        {
            while (EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY));
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, str[i]);
        }
        else
        {
            while (EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY));
            //EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            break;
        }
    }
}

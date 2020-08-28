/*
 * hwspi.h
 *
 *  Created on: 29 juil. 2020
 *      Author: Marc
 */

#include <driverlib.h>

#ifndef HWSPI_H_
#define HWSPI_H_

#define GPIO_PORT_SPICLK       GPIO_PORT_P4
#define GPIO_PIN_SPICLK        GPIO_PIN1
#define GPIO_FUNCTION_SPICLK   GPIO_PRIMARY_MODULE_FUNCTION
#define GPIO_PORT_SPISIMO      GPIO_PORT_P4
#define GPIO_PIN_SPISIMO      GPIO_PIN3
#define GPIO_FUNCTION_SPISIMO  GPIO_PRIMARY_MODULE_FUNCTION
#define GPIO_PORT_SPISOMI      GPIO_PORT_P4
#define GPIO_PIN_SPISOMI       GPIO_PIN2
#define GPIO_FUNCTION_SPISOMI  GPIO_PRIMARY_MODULE_FUNCTION

#define GPIO_SPI_PIN_ALL           (GPIO_PIN_SPICLK |GPIO_PIN_SPISOMI | GPIO_PIN_SPISIMO)
#define GPIO_SPI_PIN_OUT           (GPIO_PIN_SPICLK | GPIO_PIN_SPISIMO)

void hwspiTransmission(uint8_t * trm, uint8_t * rcv, int len);
void hwspiEnable();
void hwspiDisable();
void hwspiInit();

#endif /* HWSPI_H_ */

/*
 * hwi2c.h
 *
 *  Created on: 10 juil. 2020
 *      Author: loic
 */
#include <driverlib.h>

#ifndef HWI2C_H_
#define HWI2C_H_
#define HANDLE I2C_HANDLE


#define GPIO_PORT_UCB0SCL       GPIO_PORT_P1
#define GPIO_PIN_UCB0SCL        GPIO_PIN3
#define GPIO_FUNCTION_UCB0SCL   GPIO_PRIMARY_MODULE_FUNCTION
#define GPIO_PORT_UCB0SDA       GPIO_PORT_P1
#define GPIO_PIN_UCB0SDA        GPIO_PIN2
#define GPIO_FUNCTION_UCB0SDA   GPIO_PRIMARY_MODULE_FUNCTION

#define GPIO_PORT_UCB1SCL       GPIO_PORT_P4
#define GPIO_PIN_UCB1SCL        GPIO_PIN7
#define GPIO_FUNCTION_UCB1SCL   GPIO_PRIMARY_MODULE_FUNCTION
#define GPIO_PORT_UCB1SDA       GPIO_PORT_P4
#define GPIO_PIN_UCB1SDA        GPIO_PIN6
#define GPIO_FUNCTION_UCB1SDA   GPIO_PRIMARY_MODULE_FUNCTION

typedef struct I2C_HANDLE{
    uint16_t BASE;
    bool isA;
    uint8_t slave_addr;
} I2C_HANDLE;


void hwInitI2C(HANDLE *handle);
void hwInitI2CB0(HANDLE *handle);
void hwInitI2CA0(HANDLE *handle);

int16_t hwSendI2C(HANDLE *handle, uint8_t * data, int len);
void hwReadI2C(HANDLE *handle, uint8_t address, uint8_t * data, int len);

int16_t hwSendI2CB0(HANDLE *handle, uint8_t * data, int len);
void hwReadI2CB0(HANDLE *handle, uint8_t address, uint8_t * data, int len);

void hwSendI2CA(HANDLE *handle, uint8_t * data, int len);
void hwReadI2CA(HANDLE *handle, uint8_t address, uint8_t * data, int len);

unsigned char i2c_read(unsigned char slv_addr, unsigned char reg_addr);

#endif /* HWI2C_H_ */

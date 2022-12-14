/*
 * si115x.h
 *
 *  Created on: 9 juil. 2020
 *      Author: loic
 */

#ifndef SI115X_H_
#define SI115X_H_
#define HANDLE I2C_HANDLE

typedef struct
{
    uint8_t     irq_status;
    int16_t    ch0;
    int16_t    ch1;
    int16_t    ch2;
    int16_t    min;
    int16_t     nb_min;
} SI115X_SAMPLES ;

int16_t si115x_init_1CH( HANDLE*si115x_handle, int timeShift );
int16_t si115x_init_3CH( HANDLE*si115x_handle );

void si115x_handler(HANDLE *si115x_handle, SI115X_SAMPLES *samples);
void si115x_GetMeasure(HANDLE *si115x_handle, SI115X_SAMPLES *samples);
void si115x_Stop(HANDLE *si115x_handle);
void si115x_Start(HANDLE *si115x_handle);

#endif /* SI115X_H_ */

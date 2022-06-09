/*
 * hwuart.h
 *
 *  Created on: 9 juin 2022
 *      Author: Marc
 */

#ifndef HWUART_H_
#define HWUART_H_


#define MAX_STRBUF_SIZE 32

void hwuart_Send(char * str);
void hwuart_Init();

#endif /* HWUART_H_ */

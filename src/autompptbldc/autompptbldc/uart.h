/*
* uart.h
*
* Created: 14.11.2018 16:49:13
*  Author: JWingert
*/


#ifndef UART_H_
#define UART_H_


void uart_setup(void);

int uart_getchar(FILE *stream);
int uart_putchar(char c, FILE *stream);


#endif /* UART_H_ */
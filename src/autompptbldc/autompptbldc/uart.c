/*
* uart.c
*
* Created: 14.11.2018 16:49:24
*  Author: JWingert
*/

#include "includes.h"
#include "uart.h"

// uart ->printf binding
FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

int uart_getchar(FILE *stream)
{
	char data;
	unsigned char status;

	while (1)
	{
		while (((status=USARTD0.STATUS) & USART_RXCIF_bm) == 0);
		data=USARTD0.DATA;
		if ((status & (USART_FERR_bm | USART_PERR_bm | USART_BUFOVF_bm)) == 0) return data;
	}
}


int uart_putchar(char c, FILE *stream)
{
	while ((USARTD0.STATUS & USART_DREIF_bm) == 0);
	USARTD0.DATA=c;
	return 1;
}

void uart_setup(void){

	//****************************************************************************************************
	// uart setup
	// Baud rate 115200

	// Set TxD=1
	PORTD.OUT=0x08;

	// Communication mode: Asynchronous USART
	// Data bits: 8
	// Stop bits: 1
	// Parity: Disabled
	USARTD0.CTRLC=USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
	USARTD0.CTRLA=(USARTD0.CTRLA & (~(USART_RXCINTLVL_gm | USART_TXCINTLVL_gm | USART_DREINTLVL_gm))) |
	USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	// Baud rate: 500000
	USARTD0.BAUDCTRLA=0x80;
	USARTD0.BAUDCTRLB=((0x09 << USART_BSCALE_gp) & USART_BSCALE_gm) | 0x01;

	// Receiver
	USARTD0.CTRLB=(USARTD0.CTRLB & (~(USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm | USART_MPCM_bm | USART_TXB8_bm))) |
	USART_RXEN_bm | USART_TXEN_bm;

	// now bind it to stdout and in
	stdout = &uart_output;
	stdin  = &uart_input;
}
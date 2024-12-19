/* $Id: serial.c,v 1.2 2017/06/05 11:00:19 bouyer Exp $ */
/*
 * Copyright (c) 2024 Manuel Bouyer
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <xc.h>
#include <stdio.h>
#include <serial.h>

/* uart1 (debug) support */

char uart_txbuf[UART_TXBUFSIZE];
unsigned char uart_txbuf_prod;
volatile unsigned char uart_txbuf_cons;

void
usart_putchar (char c)
{
#if 1
	unsigned char new_uart_txbuf_prod = (uart_txbuf_prod + 1) & UART_TXBUFSIZE_MASK;

again:
        while (new_uart_txbuf_prod == uart_txbuf_cons) {
		PIE4bits.U1TXIE = 1; /* ensure we'll make progress */
	}
	uart_txbuf[uart_txbuf_prod] = c;
	uart_txbuf_prod = new_uart_txbuf_prod;
	PIE4bits.U1TXIE = 1;
	if (c == '\n') {
		c = '\r';
		new_uart_txbuf_prod = (uart_txbuf_prod + 1) & UART_TXBUFSIZE_MASK;
		goto again;
	}
#else
	char d = c;
again:
	if  (!PIR4bits.U1TXIF) {
		goto again;
	}
	U1TXB = d;
	if (d == '\n') {
		d = '\r';
		goto again;
	}
#endif
}

void __interrupt(__irq(U1TX), __low_priority, base(IVECT_BASE))
irql_uart1tx(void)
{
	if (PIE4bits.U1TXIE && PIR4bits.U1TXIF) {
		if (uart_txbuf_prod == uart_txbuf_cons) {
			PIE4bits.U1TXIE = 0; /* buffer empty */
		} else {
			/* Place char in TXREG - this starts transmition */
			U1TXB = uart_txbuf[uart_txbuf_cons];
			uart_txbuf_cons = (uart_txbuf_cons + 1) & UART_TXBUFSIZE_MASK;
		}
	}
}

char uart_rxbuf1[UART_RXBUFSIZE];
char uart_rxbuf2[UART_RXBUFSIZE];
unsigned char uart_rxbuf_idx;
unsigned char uart_rxbuf_a;
volatile union uart_softintrs uart_softintrs;

void __interrupt(__irq(U1RX), __low_priority, base(IVECT_BASE))
irql_uart1rx(void)
{
	if (PIR4bits.U1RXIF) {
		char c = U1RXB;

		if (U1ERRIRbits.RXFOIF || U1ERRIRbits.FERIF) {
			(void)c;
			U1ERRIRbits.RXFOIF = 0;
			/* error; ignore line */
			uart_rxbuf_idx = UART_RXBUFSIZE;
			return;
		}
		if (c == 0x0a)
			return;

		if (uart_rxbuf_idx == UART_RXBUFSIZE) {
			/* overflow, reset on \n */
			if (c == 0x0d)
				uart_rxbuf_idx = 0;
			return;
		}
		switch(uart_rxbuf_a) {
		case 1:
			if (c == 0x0d) {
				uart_rxbuf1[uart_rxbuf_idx] = 0;
				uart_rxbuf_idx = 0;
				uart_softintrs.bits.uart1_line1 = 1;
				if (uart_softintrs.bits.uart1_line2) {
					/* overflow */
					uart_rxbuf_a = 0;
				} else {
					uart_rxbuf_a = 2;
				}
			} else {
				uart_rxbuf1[uart_rxbuf_idx] = c;
				uart_rxbuf_idx++;
			}
			return;
		case 2:
			if (c == 0x0d) {
				uart_rxbuf2[uart_rxbuf_idx] = 0;
				uart_rxbuf_idx = 0;
				uart_softintrs.bits.uart1_line2 = 1;
				if (uart_softintrs.bits.uart1_line1) {
					/* overflow */
					uart_rxbuf_a = 0;
				} else {
					uart_rxbuf_a = 1;
				}
			} else {
				uart_rxbuf2[uart_rxbuf_idx] = c;
				uart_rxbuf_idx++;
			}
			return;
		default:
			/* overflow condition, wait for next \n */
			uart_rxbuf_idx = UART_RXBUFSIZE;
			(void)c;
		}
	}
}

/* UART2 (rs232) support */
char uart232_txbuf[UART232_TXBUFSIZE];
unsigned char uart232_txbuf_prod;
volatile unsigned char uart232_txbuf_cons;

void
uart232_putchar (char c)
{
	unsigned char new_uart232_txbuf_prod = (uart232_txbuf_prod + 1) & UART232_TXBUFSIZE_MASK;

again:
        while (new_uart232_txbuf_prod == uart232_txbuf_cons) {
		PIE8bits.U2TXIE = 1; /* ensure we'll make progress */
	}
	uart232_txbuf[uart232_txbuf_prod] = c;
	uart232_txbuf_prod = new_uart232_txbuf_prod;
	PIE8bits.U2TXIE = 1;
	if (c == '\n') {
		c = '\r';
		new_uart232_txbuf_prod = (uart232_txbuf_prod + 1) & UART232_TXBUFSIZE_MASK;
		goto again;
	}
}

void __interrupt(__irq(U2TX), __low_priority, base(IVECT_BASE))
irql_uart2321tx(void)
{
	if (PIE8bits.U2TXIE && PIR8bits.U2TXIF) {
		if (uart232_txbuf_prod == uart232_txbuf_cons) {
			PIE8bits.U2TXIE = 0; /* buffer empty */
		} else {
			/* Place char in TXREG - this starts transmition */
			U2TXB = uart232_txbuf[uart232_txbuf_cons];
			uart232_txbuf_cons = (uart232_txbuf_cons + 1) & UART232_TXBUFSIZE_MASK;
		}
	}
}

char uart232_rxbuf1[UART232_RXBUFSIZE];
char uart232_rxbuf2[UART232_RXBUFSIZE];
unsigned char uart232_rxbuf_idx;
unsigned char uart232_rxbuf_a;
volatile union uart_softintrs uart_softintrs;

void __interrupt(__irq(U2RX), __low_priority, base(IVECT_BASE))
irql_uart2321rx(void)
{
	if (PIR8bits.U2RXIF) {
		char c = U2RXB;

		if (U2ERRIRbits.RXFOIF || U2ERRIRbits.FERIF) {
			(void)c;
			U2ERRIRbits.RXFOIF = 0;
			/* error; ignore line */
			uart232_rxbuf_idx = UART232_RXBUFSIZE;
			return;
		}
		if (c == 0x0a)
			return;

		if (uart232_rxbuf_idx == UART232_RXBUFSIZE) {
			/* overflow, reset on \n */
			if (c == 0x0d)
				uart232_rxbuf_idx = 0;
			return;
		}
		switch(uart232_rxbuf_a) {
		case 1:
			if (c == 0x0d) {
				uart232_rxbuf1[uart232_rxbuf_idx] = 0;
				uart232_rxbuf_idx = 0;
				uart_softintrs.bits.uart232_line1 = 1;
				if (uart_softintrs.bits.uart232_line2) {
					/* overflow */
					uart232_rxbuf_a = 0;
				} else {
					uart232_rxbuf_a = 2;
				}
			} else {
				uart232_rxbuf1[uart232_rxbuf_idx] = c;
				uart232_rxbuf_idx++;
			}
			return;
		case 2:
			if (c == 0x0d) {
				uart232_rxbuf2[uart232_rxbuf_idx] = 0;
				uart232_rxbuf_idx = 0;
				uart_softintrs.bits.uart232_line2 = 1;
				if (uart_softintrs.bits.uart232_line1) {
					/* overflow */
					uart232_rxbuf_a = 0;
				} else {
					uart232_rxbuf_a = 1;
				}
			} else {
				uart232_rxbuf2[uart232_rxbuf_idx] = c;
				uart232_rxbuf_idx++;
			}
			return;
		default:
			/* overflow condition, wait for next \n */
			uart232_rxbuf_idx = UART232_RXBUFSIZE;
			(void)c;
		}
	}
}

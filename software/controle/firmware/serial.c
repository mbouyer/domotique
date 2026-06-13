/*
 * Copyright (c) 2025 Manuel Bouyer
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

/* uart2 (debug) support */

char uart_txbuf[UART_TXBUFSIZE];
unsigned char uart_txbuf_prod;
volatile unsigned char uart_txbuf_cons;

void
usart_putchar (char c)
{
#if 1
	unsigned char new_uart_txbuf_prod = (uart_txbuf_prod + 1) & UART_TXBUFSIZE_MASK;

again:
	if (new_uart_txbuf_prod == uart_txbuf_cons) {
		while (new_uart_txbuf_prod == uart_txbuf_cons) {
			PIE8bits.U2TXIE = 1; /* ensure we'll make progress */
		}
	}
	uart_txbuf[uart_txbuf_prod] = c;
	uart_txbuf_prod = new_uart_txbuf_prod;
	PIE8bits.U2TXIE = 1; /* ensure we'll make progress */
	if (c == '\n') {
		c = '\r';
		new_uart_txbuf_prod = (uart_txbuf_prod + 1) & UART_TXBUFSIZE_MASK;
		goto again;
	}
#else
	char d = c;
again:
	if  (!PIR8bits.U2TXIF) {
		goto again;
	}
	U2TXB = d;
	if (d == '\n') {
		d = '\r';
		goto again;
	}
#endif
}

void __interrupt(__irq(U2TX), __low_priority, base(IVECT_BASE))
irql_uart2tx(void)
{
	if (PIE8bits.U2TXIE && PIR8bits.U2TXIF) {
		if (uart_txbuf_prod == uart_txbuf_cons) {
			PIE8bits.U2TXIE = 0; /* buffer empty */
		} else {
			/* Place char in TXREG - this starts transmition */
			U2TXB = uart_txbuf[uart_txbuf_cons];
			uart_txbuf_cons = (uart_txbuf_cons + 1) & UART_TXBUFSIZE_MASK;
		}
	}
}

char uart_rxbuf1[UART_RXBUFSIZE];
char uart_rxbuf2[UART_RXBUFSIZE];
unsigned char uart_rxbuf_idx;
unsigned char uart_rxbuf_a;
volatile union uart_softintrs uart_softintrs;

void __interrupt(__irq(U2RX), __low_priority, base(IVECT_BASE))
irql_uart2rx(void)
{
	if (PIR8bits.U2RXIF) {
		char c;

		if (U2ERRIRbits.RXFOIF || U2ERRIRbits.FERIF) {
			c = U2RXB;
			(void)c;
			U2ERRIRbits.RXFOIF = 0;
			/* error; ignore line */
			uart_rxbuf_idx = UART_RXBUFSIZE;
			return;
		}

		c = U2RXB;
		if (c == 0x0a)
			return;

		if (uart_rxbuf_idx == UART_RXBUFSIZE) {
			/* overflow, reset on \n */
			if (c == 0x0d) {
				uart_rxbuf_idx = 0;
			}
			return;
		}
		switch(uart_rxbuf_a) {
		case 1:
			if (c == 0x0d) {
				uart_rxbuf1[uart_rxbuf_idx] = 0;
				uart_rxbuf_idx = 0;
				uart_softintrs.bits.uart2_line1 = 1;
				if (uart_softintrs.bits.uart2_line2) {
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
				uart_softintrs.bits.uart2_line2 = 1;
				if (uart_softintrs.bits.uart2_line1) {
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

enum lin_state lin_state;

uint8_t lin_userid;
uint8_t lin_pid;
uint8_t lin_errir;
uint8_t lin_rxidx;
uint8_t lin_txidx;
uint8_t lin_txcount;
uint8_t lin_rxbuf[LIN_BUF_SIZE];
uint8_t lin_txbuf[LIN_BUF_SIZE];

#define PERIFv ((uint8_t)1 << _U1ERRIR_PERIF_POSN)
#define RXFOIFv ((uint8_t)1 << _U1ERRIR_RXFOIF_POSN)
#define FERIFv ((uint8_t)1 << _U1ERRIR_FERIF_POSN)
#define CERIFv ((uint8_t)1 << _U1ERRIR_CERIF_POSN)

void __interrupt(__irq(U1RX), __low_priority, base(IVECT_BASE))
irql_uartlinrx(void)
{
	if (PIR4bits.U1RXIF) {
		char c;
		if (U1ERRIR & (PERIFv | RXFOIFv | FERIFv)) {
			c = U1RXB;
			(void)c;
			U1ERRIRbits.RXFOIF = 0;
			U1P2 = U1P3 = 0;
			lin_state = IDLE;
			return;

		}
		c = U1RXB;
		if (U1ERRIRbits.U1RXBKIF) {
			U1ERRIRbits.U1RXBKIF = 0;
			U1ERRIRbits.CERIF = 0;
			lin_errir = U1ERRIR;
			lin_pid = (c & 0x3f);
			uart_softintrs.bits.int_linpid = 1;
			if ((lin_pid & 0x0f) == lin_userid) {
				switch(lin_pid & 0x30) {
				case 0x00:
					lin_state = RXDATA;
					lin_rxidx = 0;
					U1P3 = 1;
					break;
				case 0x10:
					lin_state = TXDATA;
					lin_txidx = 0;
					lin_txcount = U1P2 = 5;
					PIE4bits.U1TXIE=1;
					break;
				default:
					lin_state = IDLE;
				}
			} else {
				lin_state = IDLE;
			}
		} else if (lin_state == RXDATA) {
			lin_rxbuf[lin_rxidx] = c;
			lin_rxidx++;
			if (lin_rxidx == U1P3) {
				lin_state = RXCSUM;
			}
		} else if (lin_state == RXCSUM) {
			if (U1ERRIRbits.CERIF == 0) {
				uart_softintrs.bits.int_linrx = 1;
			} else {
				U1ERRIRbits.CERIF = 0;
			}
			lin_state = IDLE;
			(void)c;
		} else {
			(void)c;
		}
	}
}

void __interrupt(__irq(U1TX), __low_priority, base(IVECT_BASE))
irql_uartlintx(void)
{
	U1TXB = lin_txbuf[lin_txidx++];
	lin_txcount--;
	if (lin_txcount == 0)
		lin_state = IDLE;
}

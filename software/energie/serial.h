/* $Id: autopilot_serial.h,v 1.2 2017/06/05 11:00:18 bouyer Exp $ */
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

/* #define UART_TXBUFSIZE 16 */
/* #define UART_TXBUFSIZE_MASK 0x0f */
#define UART_TXBUFSIZE 128
#define UART_TXBUFSIZE_MASK 0x7f

extern char uart_txbuf[UART_TXBUFSIZE];
extern unsigned char uart_txbuf_prod;
extern volatile unsigned char uart_txbuf_cons;
void usart_putchar (char c);

#define UART_RXBUFSIZE 16
extern char uart_rxbuf1[UART_RXBUFSIZE];
extern char uart_rxbuf2[UART_RXBUFSIZE];
extern unsigned char uart_rxbuf_idx;
extern unsigned char uart_rxbuf_a;

#define USART_INIT(p) { \
		IPR4bits.U1TXIP=p; \
		IPR4bits.U1RXIP=p; \
		uart_txbuf_prod = uart_txbuf_cons = 0; \
		uart_softintrs.byte = 0; uart_rxbuf_idx = 0; uart_rxbuf_a= 1;\
		PIE4bits.U1RXIE = 1; \
	}

union uart_softintrs {
        struct uart_softintrs_bits {
		char uart1_line1 : 1;    /* a line is ready in uart_rxbuf1 */
		char uart1_line2 : 1;    /* a line is ready in uart_rxbuf2 */
		char uart232_line1 : 1;  /* a line is ready in uart232_rxbuf1 */
		char uart232_line2 : 1;  /* a line is ready in uart232_rxbuf2 */
	} bits;
	char byte;
};
extern volatile union uart_softintrs uart_softintrs;

#define UART232_TXBUFSIZE 32
#define UART232_TXBUFSIZE_MASK 0x1f

extern char uart232_txbuf[UART232_TXBUFSIZE];
extern unsigned char uart232_txbuf_prod;
extern volatile unsigned char uart232_txbuf_cons;
void uart232_putchar (char c);

#define UART232_RXBUFSIZE 16
extern char uart232_rxbuf1[UART232_RXBUFSIZE];
extern char uart232_rxbuf2[UART232_RXBUFSIZE];
extern unsigned char uart232_rxbuf_idx;
extern unsigned char uart232_rxbuf_a;

#define UART232_INIT(p) { \
		IPR8bits.U2TXIP=p; \
		IPR8bits.U2RXIP=p; \
		uart232_txbuf_prod = uart232_txbuf_cons = 0; \
		uart232_rxbuf_idx = 0; uart232_rxbuf_a= 1;\
		PIE8bits.U2RXIE = 1; \
	}

extern volatile union uart232_softintrs uart232_softintrs;

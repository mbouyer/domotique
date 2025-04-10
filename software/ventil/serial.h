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
		IPR8bits.U2TXIP=p; \
		IPR8bits.U2RXIP=p; \
		uart_txbuf_prod = uart_txbuf_cons = 0; \
		uart_softintrs.byte = 0; uart_rxbuf_idx = 0; uart_rxbuf_a= 1;\
	}

union uart_softintrs {
        struct uart_softintrs_bits {
		char uart2_line1 : 1;	/* a line is ready in uart_rxbuf1 */
		char uart2_line2 : 1;	/* a line is ready in uart_rxbuf2 */
		char int_linrx : 1;	/* lin RX ready */
		char int_linpid : 1;	/* lin RX ready */
	} bits;
	char byte;
};
extern volatile union uart_softintrs uart_softintrs;

#define LIN_BUF_SIZE 9

enum lin_state {
	IDLE = 0,
	RXPID,
	RXDATA,
	RXCSUM,
	TXDATA,
};
extern enum lin_state lin_state;

extern uint8_t lin_userid;
extern uint8_t lin_pid;
extern uint8_t lin_errir;
extern uint8_t lin_rxidx;
extern uint8_t lin_txidx;
extern uint8_t lin_rxbuf[LIN_BUF_SIZE];
extern uint8_t lin_txbuf[LIN_BUF_SIZE];

#define LIN_INIT(p) { \
		IPR4bits.U1TXIP=p; \
		IPR4bits.U1RXIP=p; \
		U1BRGL = 207; /* 19200 at 64Mhz */ \
		U1CON0 = 0x3b; /* 00111011, line client mode  */ \
		U1CON2bits.U1RUNOVF = 1; \
		lin_state = IDLE; \
		U1P2 = U1P3 = 0; \
		U1CON1bits.U1ON = 1; \
		PIE4bits.U1RXIE=1;\
	}

static inline char
lin_get(void)
{
	PIE4bits.U1RXIE = 0;
	if (lin_state == IDLE)
		return 1;
	PIE4bits.U1RXIE = 1;
	return 0;
}

static inline void
lin_put(void)
{
	PIE4bits.U1RXIE = 1;
}

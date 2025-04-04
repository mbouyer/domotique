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
		rx_debug_ov = tx_debug_w = 0; \
		rx_232_ov = rx_232_cs = tx_232_w = 0; \
		rx_linky_ov = rx_linky_cs = 0; \
	}

union uart_softintrs {
        struct uart_softintrs_bits {
		char uart1_line1 : 1;    /* a line is ready in uart_rxbuf1 */
		char uart1_line2 : 1;    /* a line is ready in uart_rxbuf2 */
		char uart232_line1 : 1;  /* a line is ready in uart232_rxbuf1 */
		char uart232_line2 : 1;  /* a line is ready in uart232_rxbuf2 */
		char linky_line1 : 1;    /* a line is ready in linky_rxbuf1 */
		char linky_line2 : 1;    /* a line is ready in linky_rxbuf2 */
		char linky_badcs_l1 : 1; /* bad checksum in linky_rxbuf1 */
		char linky_badcs_l2 : 1; /* bad checksum in linky_rxbuf2 */
	} bits;
	char byte;
};
extern volatile union uart_softintrs uart_softintrs;

#define UART232_TXBUFSIZE 256
#define UART232_TXBUFSIZE_MASK 0xff

void uart232_putchar (char c);
void uart232_init(void);

#define UART232_RXBUFSIZE 16
extern char uart232_rxbuf1[UART232_RXBUFSIZE];
extern char uart232_rxbuf2[UART232_RXBUFSIZE];
extern unsigned char uart232_rxbuf_idx;
extern unsigned char uart232_rxbuf_a;

union linky_softintrs {
        struct linky_softintrs_bits {
		char linky_line1 : 1;    /* a line is ready in linky_rxbuf1 */
		char linky_line2 : 1;    /* a line is ready in linky_rxbuf2 */
		char linky_badcs_l1 : 1; /* bad checksum in linky_rxbuf1 */
		char linky_badcs_l2 : 1; /* bad checksum in linky_rxbuf2 */
		char linky_sof : 1; /* start of frame detected */
		char linky_eof : 1; /* end of frame detected */
		char linky_interframe_active; /* doing interframe outputs */
	} bits;
	char byte;
};
extern volatile union linky_softintrs linky_softintrs;
#define LINKY_RXBUFSIZE 32
extern char linky_rxbuf1[LINKY_RXBUFSIZE];
extern char linky_rxbuf2[LINKY_RXBUFSIZE];
extern unsigned char linky_rxbuf_idx;
extern unsigned char linky_rxbuf_a;
void linky_init(void);

extern unsigned char rx_debug_ov;
extern unsigned char tx_debug_w;
extern unsigned char rx_232_ov;
extern unsigned char rx_232_cs;
extern unsigned char tx_232_w;
extern unsigned char rx_linky_ov;
extern unsigned char rx_linky_cs;

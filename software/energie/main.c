/* $Id: main.c,v 1.36 2019/03/12 19:24:19 bouyer Exp $ */
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
#include <string.h> 
#include <stdlib.h> 
#include "serial.h"
#include "vers.h"

typedef unsigned char u_char;
typedef unsigned int  u_int;
typedef unsigned long u_long;

static u_char default_src;

#define LEDN LATCbits.LATC7


u_int timer0_read(void);

#define TIMER0_1MS    10
#define TIMER0_5MS    49
#define TIMER0_20MS   195
#define TIMER0_100MS  977

static char counter_100hz;
static char counter_100hz;
static char counter_10hz;
static uint16_t seconds;
static volatile union softintrs {
	struct softintrs_bits {
		char int_100hz : 1;	/* 0.01s timer */
		char int_adcc : 1;	/* A/D convertion complete */
	} bits;
	char byte;
} softintrs;

static union time_events {
	struct tmev_bits {
		char ev_100hz : 1;	/* 0.01s timer */
		char ev_10hz : 1;	/* 0.1s timer */
		char ev_1hz : 1;	/* 1s timer */
	} bits;
	char byte;
} time_events;

static union uout {
	struct _uout {
		char debug : 1;
		char rs232 : 1;
	} bits;
	char byte;
} uout;

void
putch(char c)
{
        if (PORTBbits.RB7) {
		usart_putchar(c);
	}
}


int
main(void)
{
	char c;

	default_src = 0;
	uout.byte = 0;
        if (PORTBbits.RB7)
		uout.bits.debug = 1;
	U1CON1bits.U1RXBIMD = 1; /* detect RX going low */

	ANSELC = 0;
	ANSELB = 0;
	ANSELA = 0x7F; /* RA0->RA6 analog */

	LATA = 0;

	LEDN = 0;
	TRISCbits.TRISC7 = 0; /* RC7/LED output */

	/* configure watchdog timer for 2s */
	WDTCON0 = 0x16;
	WDTCON1 = 0x07;

	/* configure sleep mode: PRI_IDLE */
	CPUDOZE = 0x80;

	/*
	 * configure priotities memory access priorities
	 * ISR > DMA1 > main
	 */
	PRLOCK = 0x55;
	PRLOCK = 0xAA;
	PRLOCKbits.PRLOCKED = 0;
	ISRPR = 0;
	MAINPR = 4;
	DMA1PR = 3;
	PRLOCK = 0x55;
	PRLOCK = 0xAA;
	PRLOCKbits.PRLOCKED = 1;


	softintrs.byte = 0;
	counter_100hz = 10;
	counter_100hz = 10;
	counter_10hz = 10;
	seconds = 0;
	time_events.byte = 0;

	IPR1 = 0;
	IPR2 = 0;
	IPR3 = 0;
	IPR4 = 0;
	IPR5 = 0;
	IPR6 = 0;
	IPR7 = 0;
	IPR8 = 0;
	IPR9 = 0;
	IPR10 = 0;
	IPR11 = 0;
	IPR12 = 0;
	IPR13 = 0;
	IPR14 = 0;
	IPR15 = 0;
	INTCON0 = 0;
	INTCON1 = 0;
	INTCON0bits.IPEN=1; /* enable interrupt priority */

	USART_INIT(0);

	/* configure UART2 */
	TRISBbits.TRISB2 = 0 ;
	LATBbits.LATB2 = 1 ;
	U2RXPPS = 0x0b; /* RB3, 001 011 */
	RB2PPS = 0x23; /* RB2 */
	U2BRGL = 68; /* 57600 at 64Mhz */
	U2CON0 = 0x30; /* 00110000 */
	U2CON2bits.U2RUNOVF = 1;
	U2CON1bits.U2ON = 1;
	UART232_INIT(0);


	/* configure timer0 as free-running counter at 9.765625Khz */
	T0CON0 = 0x0;
	T0CON0bits.MD16 = 1; /* 16 bits */
	T0CON1 = 0x4b; /* 01001011 Fosc/4, sync, 1/2048 prescale */
	PIR3bits.TMR0IF = 0;
	PIE3bits.TMR0IE = 0; /* no interrupt */
	T0CON0bits.T0EN = 1;

	/* configure timer2 for 100Hz interrupt */
	T2CON = 0x69; /* b01101001: postscaller 1/10, prescaler 1/64 */
	T2PR = 250; /* 100hz output */
	T2CLKCON = 0x01; /* Fosc / 4 */
	T2CONbits.TMR2ON = 1;
	PIR3bits.TMR2IF = 0;
	IPR3bits.TMR2IP = 1; /* high priority interrupt */
	PIE3bits.TMR2IE = 1;

	INTCON0bits.GIEH=1;  /* enable high-priority interrupts */   
	INTCON0bits.GIEL=1; /* enable low-priority interrrupts */   

	printf("energie %d.%d %s\n", MAJOR, MINOR, buildstr);

	/* enable watchdog */
	WDTCON0bits.SEN = 1;

	/* set up ADC */
	ADCON0 = 0x4; /* right-justified */
	ADCLK = 0x7F; /* Fosc/64 */

	ADCON0bits.ADON = 1;

	printf("enter loop\n");
	LEDN = 1;

again:
	while (1) {
		if (PORTBbits.RB7) {
			uout.bits.debug = 1;
			PIE4bits.U1RXIE = 1;
			U1CON1bits.U1ON = 1;
		} else if (U1ERRIRbits.RXBKIF) {
			/* TX disconnected */
			uout.bits.debug = 0;
			PIE4bits.U1RXIE = 0;
			U1ERRIRbits.RXBKIF = 0;
			U1CON1bits.U1ON = 0;
		}
		time_events.byte = 0;
		if (softintrs.bits.int_100hz) {
			softintrs.bits.int_100hz = 0;
			time_events.bits.ev_100hz = 1;
			counter_100hz--;
			if (counter_100hz == 0) {
				counter_100hz = 10;
				time_events.bits.ev_10hz = 1;
				counter_10hz--;
				if (counter_10hz == 0) {
					counter_10hz = 10;
					time_events.bits.ev_1hz = 1;
				}
			}
		}

		CLRWDT();

		if (time_events.bits.ev_1hz) {
			LEDN ^= 1;
			printf("st %d %d\n", uart_rxbuf_idx, uart_rxbuf_a);
			printf("0x%x 0x%x 0x%x 0x%x\n", U1CON0, U1CON1, U1CON2, U1ERRIR);
			printf("st232 %d %d\n", uart232_rxbuf_idx, uart232_rxbuf_a);
			uart232_putchar('a');
			uart232_putchar('b');
			uart232_putchar('\r');
			uart232_putchar('\n');
		}

		if (uart_softintrs.bits.uart1_line1) {
			printf("line1: %s\n", uart_rxbuf1);
			if (strcmp(uart_rxbuf1, "reb") == 0)
				break;
			uart_softintrs.bits.uart1_line1 = 0;
		} else if (uart_softintrs.bits.uart1_line2) {
			printf("line2: %s\n", uart_rxbuf2);
			uart_softintrs.bits.uart1_line2 = 0;
			if (strcmp(uart_rxbuf2, "reb") == 0)
				break;
		} else if (uart_rxbuf_a == 0) {
			/* clear overflow */
			uart_rxbuf_a = 1;
		}
		if (uart_softintrs.bits.uart232_line1) {
			printf("line232_1: %s\n", uart232_rxbuf1);
			uart_softintrs.bits.uart232_line1 = 0;
		} else if (uart_softintrs.bits.uart232_line2) {
			printf("line232_2: %s\n", uart232_rxbuf2);
			uart_softintrs.bits.uart232_line2 = 0;
		} else if (uart232_rxbuf_a == 0) {
			/* clear overflow */
			uart232_rxbuf_a = 1;
		}
				
		if (default_src != 0) {
			printf("default handler called for 0x%x\n",
			    default_src);
			default_src = 0;
		}

	}
	WDTCON0bits.SEN = 0;
	printf("returning\n");
	while (!PIR4bits.U1TXIF || PIE4bits.U1TXIE) {
		; /* wait */ 
	}
	RESET();
	return 0;
}

u_int
timer0_read(void)
{
	u_int value;

	/* timer0_value = TMR0L | (TMR0H << 8), reading TMR0L first */
	di();
	asm("movff TMR0L, timer0_read@value");
	asm("movff TMR0H, timer0_read@value+1");
	ei();
	return value;
}

void __interrupt(__irq(TMR2), __high_priority, base(IVECT_BASE))
irqh_timer2(void)
{
	PIR3bits.TMR2IF = 0;
	softintrs.bits.int_100hz = 1;
}

void __interrupt(__irq(default), __low_priority, base(IVECT_BASE))
irqh_default(void)
{
	default_src = 0xff;
}

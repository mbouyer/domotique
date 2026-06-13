/*
 * Copyright (c) 2026 Manuel Bouyer
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
#include "i2c.h"
#include "vers.h"

typedef unsigned char u_char;
typedef unsigned int  u_int;
typedef unsigned long u_long;

u_int revid, devid;

static u_char default_src; 

u_int timer0_read(void);

#define TIMER0_1MS    10
#define TIMER0_5MS    49
#define TIMER0_20MS   195
#define TIMER0_100MS  977

static void delay(uint16_t d)
{
	uint16_t tmr0read = timer0_read();
	while ((timer0_read() - tmr0read) < d) {
		CLRWDT();
	}
}

static char counter_100hz;
static char counter_10hz;
static uint16_t seconds;
static __uint24 time; /* in 10ms units */

static volatile union softintrs {
	struct softintrs_bits {
		char int_100hz : 1;	/* 0.01s timer */
		char int_10hz : 1;	/* 0.01s timer */
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

static int16_t lux;

#define O_VCPU_EN 0x04 /* RA2 */
#define VCPU_EN	LATAbits.LATA2
#define O_I2C   (u_char)0x03 /* RC0, RC1 */

static union uout {
	struct _uout {
		char debug_present : 1;
		char debug : 1;
	} bits;
	char byte;
} uout;

void
putch(char c)
{
	if (uout.bits.debug && uout.bits.debug_present) {
		usart_putchar(c);
	}
}


static char
command(__ram char *buf)
{
	char r = 1;

	if (strcmp(buf, "1") == 0) {
		VCPU_EN = 1;
	} else if (strcmp(buf, "0") == 0) {
		VCPU_EN = 0;
	} else
		r = 0;
	return r;
}

static void
debug(void)
{
#if 0
	printf("PIR8 0x%x U2ERR 0x%x prod %d cons %d idx %d a %d\n",
	    PIR8, U2ERRIR, uart232_txbuf_prod, uart232_txbuf_cons,
	    uart232_rxbuf_idx, uart232_rxbuf_a);
#endif
}

static void
print_stats()
{
}

static void
do_debug(__ram char *buf)
{
	if (strcmp(buf, "stats") == 0)
		print_stats();
	else if (strncmp(buf, "debug ", 6) == 0) {
	} else {
		command(buf);
	}
}

static uint16_t
read_nvm_word(__uint24 p)
{
	NVMADR = (__uint24)p;
	NVMCON1bits.CMD = 0;
	NVMCON0bits.GO = 1;

	while (NVMCON0bits.GO)
		; /* wait */
	return NVMDAT;
}

int
main(void)
{
	char c, d;

	revid = read_nvm_word(0x3ffffc);
	devid = read_nvm_word(0x3ffffe);

	uout.byte = 0;
        if (PORTAbits.RA0) {
		uout.bits.debug_present = 1;
		U2CON1bits.U2RXBIMD = 1; /* detect RX going low */
	} else {
		U2CON1bits.U2ON = 0;
	}
	uout.bits.debug = 1;

	ANSELC = 0;
	ANSELA = 0x10; /* RA4 analog, others digital */

	TRISA |= (O_VCPU_EN);
	LATA = O_VCPU_EN;

	LATC = 0;
	/* I2C and GPIOs as outout */
	TRISC = 0xff & ~(O_I2C | 0x04 | 0x08 | 0x10 | 0x20);

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
	DMA2PR = 3;
	DMA3PR = 3;
	PRLOCK = 0x55;
	PRLOCK = 0xAA;
	PRLOCKbits.PRLOCKED = 1;


	softintrs.byte = 0;
	counter_100hz = 10;
	counter_10hz = 10;
	seconds = 0;
	time = 0;
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
	INTCON0 = 0;
	INTCON1 = 0;
	INTCON0bits.IPEN=1; /* enable interrupt priority */

	USART_INIT(0);
	if (uout.bits.debug_present)
		PIE8bits.U2RXIE = 1;

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

	/* I2C_INIT; */

	INTCON0bits.GIEH=1;  /* enable high-priority interrupts */   
	INTCON0bits.GIEL=1; /* enable low-priority interrrupts */   

	printf("controle %d.%d %s\n", MAJOR, MINOR, buildstr);
	printf("device 0x%x rev 0x%x\n", devid, revid);

	/* enable watchdog */
	WDTCON0bits.SEN = 1;

	/*
	 * set up ADC
	 */
	ADCON0 = 0x14; /* right-justified, ADCRC */
	ADCON1 = 0;
	ADCON2 = 0x3a; /* divide by 8, clear, average mode */
	ADCON3 = 0x07; /* always interrupt */
	ADCLK = 0x7F; /* Fosc/64 => 1MHz*/
	ADREF = 0x00; /* Vref- to GND, Vref+ to VCC */
	ADPRE = 0;
	ADACQ = 128; /* 2us acquisition time */
	ADRPT = 8;
	PIR1bits.ADIF = 0;
	PIE1bits.ADIE = 1;

	ADPCH = 4; /* input RA4 */

	ADCON0bits.ON = 1;

	printf("enter loop\n");

again:
	while (1) {
		if (PORTAbits.RA0) {
			uout.bits.debug_present = 1;
			PIE8bits.U2RXIE = 1;
			U2CON1bits.U2ON = 1;
		} else if (U2ERRIRbits.RXBKIF) {
			/* debug RX disconnected */
			uout.bits.debug_present = 0;
			PIE8bits.U2RXIE = 0;
			U2ERRIRbits.RXBKIF = 0;
			U2CON1bits.U2ON = 0;
		}
		time_events.byte = 0;
		if (softintrs.bits.int_adcc) {
			__uint24 adr = 0;
			if (/*debug_out.bits.adc*/ 1) 
				printf("adcc 0x%x 0x%lx, 0x%x 0x%x 0x%x\n",
				 ADRES, (uint32_t)ADACC, ADCNT, ADSTAT, ADCON0);
			ADCON2bits.ACLR = 1;
			if (/* debug_out.bits.adc */ 1) 
				printf(" %lu\n", (uint32_t)adr);
			softintrs.bits.int_adcc = 0;
		}
		if (softintrs.bits.int_100hz) {
			softintrs.bits.int_100hz = 0;
			time_events.bits.ev_100hz = 1;
		}

		CLRWDT();

		if (softintrs.bits.int_10hz) {
			softintrs.bits.int_10hz = 0;
			time_events.bits.ev_10hz = 1;
			counter_10hz--;
			if (counter_10hz == 0) {
				counter_10hz = 10;
				time_events.bits.ev_1hz = 1;
			}
		}
		if (time_events.bits.ev_1hz) {
			ADCON0bits.GO = 1;
			seconds++;
		}

		if (uart_softintrs.bits.uart2_line1) {
			printf("line1: %s\n", uart_rxbuf1);
			if (strcmp(uart_rxbuf1, "reb") == 0)
				break;
			do_debug(uart_rxbuf1);
			uart_softintrs.bits.uart2_line1 = 0;
			debug();
		} else if (uart_softintrs.bits.uart2_line2) {
			printf("line2: %s\n", uart_rxbuf2);
			if (strcmp(uart_rxbuf2, "reb") == 0)
				break;
			do_debug(uart_rxbuf2);
			uart_softintrs.bits.uart2_line2 = 0;
			debug();
		} else if (uart_rxbuf_a == 0) {
			/* clear overflow */
			uart_rxbuf_a = 1;
		}

		if (default_src != 0) {
			printf("default handler called for 0x%x\n",
			    default_src);
			default_src = 0;
		}
		if (softintrs.byte == 0 &&
		    uart_softintrs.byte == 0)
			SLEEP();
	}
	WDTCON0bits.SEN = 0;
do_reset:
	printf("returning\n");
	while (!PIR8bits.U2TXIF || PIE8bits.U2TXIE) {
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
	time++;
	counter_100hz--;
	if (counter_100hz == 0) {
		counter_100hz = 10;
		softintrs.bits.int_10hz = 1;
	}
}

void __interrupt(__irq(IRQ_AD), __low_priority, base(IVECT_BASE))
irqh_adc(void)
{
	PIR1bits.ADIF = 0;
	ADACT = 0;
	softintrs.bits.int_adcc = 1;
}

void __interrupt(__irq(default), __low_priority, base(IVECT_BASE))
irqh_default(void)
{
	default_src = 0xff;
}

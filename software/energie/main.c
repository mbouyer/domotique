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

u_int timer0_read(void);

#define TIMER0_1MS    10
#define TIMER0_5MS    49
#define TIMER0_20MS   195
#define TIMER0_100MS  977

static char counter_100hz;
static char counter_10hz;
static uint16_t seconds;
static __uint24 time; /* in 10ms units */

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
		char debug_present : 1;
		char debug : 1;
		char rs232 : 1;
	} bits;
	char byte;
} uout;

static union linky_state {
	struct _lstate {
		char hc : 1;
		char hpjr : 1;
	} bits;
	char byte;
} linky_state;

static union debug_out {
	struct _dout {
		char linky : 1;
		char adc : 1;
	} bits;
	char byte;
} debug_out;

void
putch(char c)
{
	if (uout.bits.debug && uout.bits.debug_present) {
		usart_putchar(c);
	}
	if (uout.bits.rs232) {
		uart232_putchar(c);
	}
}


/*
 * portC outputs and definitions
 * portC/latC are updated from the arrays below by DMA engines
 */

#define	O_LED	(u_char)0x80 /* RC7 */
#define O1	(u_char)0x20 /* RC5 */
#define O2	(u_char)0x40 /* RC6 */
#define OA	(u_char)0x04 /* RC2 */
#define OB	(u_char)0x02 /* RC1 */
#define OC	(u_char)0x01 /* RC0 */
#define O_I2C	(u_char)0x18 /* RC3, RC4 */

#define O1_OA	(u_char)(0)
#define O2_OA	(u_char)(O1_OA + 3)
#define O1_OB	(u_char)(O2_OA + 3)
#define O2_OB	(u_char)(O1_OB + 3)
#define O1_OC	(u_char)(O2_OB + 3)
#define O2_OC	(u_char)(O1_OC + 2)

#define LATC_DATA_SIZE (O2_OC + 2)
static char latc_data[LATC_DATA_SIZE];

#define NOUTS 6
static char outputs_status[NOUTS];

#define PIL_OFF 0
#define PIL_ON 1
#define PIL_POS 2
#define PIL_NEG 3

/*
 * intensity measures
 *  ADC value for 32A (RMS) is 2133
 */
/*
 * Accumulates 200 (25 * 8) measures over 20ms (one 50Hz cycle)
 * The sum will be intensity RMS * 100 (in ADC units), max about 213300
 */
static uint16_t adc_results[25];
/*
 * accumulate over a short period of time. In a __uint24 we can sum about 70
 * samples, just a bit more than 1s. So use a uint32 so we can average over
 * several seconds
 */

static __uint24 I_average[6];
static u_char I_count[6]; /* count of samples */
static __uint24 I_timestamp; /* time of first sample */
static u_char channel; /* active channel */

static void
do_outputs_status(void)
{
	char c;
	uout.bits.rs232 = 1;
	for (c = 0; c < 2; c++)
		printf("O%d %d\n", c, outputs_status[c]);
	for (c = 0; c < 4; c++)
		printf("P%d %d\n", c, outputs_status[c + 2]);
	uout.bits.rs232 = 0;
}

static void
update_outputs(void)
{
	switch(outputs_status[0]) {
	case 0:
		latc_data[O2_OC] &= ~(O2|OC);
		break;
	case 1:
		latc_data[O2_OC] |= OC;
		latc_data[O2_OC] &= ~O2;
		break;
	}
	switch(outputs_status[1]) {
	case 0:
		latc_data[O1_OC] &= ~(O1|OC);
		break;
	case 1:
		latc_data[O1_OC] |= OC;
		latc_data[O1_OC] &= ~O1;
		break;
	}
	switch(outputs_status[2]) {
	case PIL_OFF:
		latc_data[O2_OB] &= ~(O2|OB);
		latc_data[O2_OB+1] &= ~(O2|OB);
		break;
	case PIL_ON:
		latc_data[O2_OB] |= O2;
		latc_data[O2_OB] &= ~OB;
		latc_data[O2_OB+1] |= OB;
		latc_data[O2_OB+1] &= ~O2;
		break;
	case PIL_POS:
		latc_data[O2_OB] |= O2;
		latc_data[O2_OB] &= ~OB;
		latc_data[O2_OB+1] &= ~(O2|OB);
		break;
	case PIL_NEG:
		latc_data[O2_OB+1] |= OB;
		latc_data[O2_OB+1] &= ~O2;
		latc_data[O2_OB] &= ~(O2|OB);
		break;
	}
	switch(outputs_status[3]) {
	case PIL_OFF:
		latc_data[O1_OB] &= ~(O1|OB);
		latc_data[O1_OB+1] &= ~(O1|OB);
		break;
	case PIL_ON:
		latc_data[O1_OB] |= O1;
		latc_data[O1_OB] &= ~OB;
		latc_data[O1_OB+1] |= OB;
		latc_data[O1_OB+1] &= ~O1;
		break;
	case PIL_POS:
		latc_data[O1_OB] |= O1;
		latc_data[O1_OB] &= ~OB;
		latc_data[O1_OB+1] &= ~(O1|OB);
		break;
	case PIL_NEG:
		latc_data[O1_OB+1] |= OB;
		latc_data[O1_OB+1] &= ~O1;
		latc_data[O1_OB] &= ~(O1|OB);
		break;
	}
	switch(outputs_status[4]) {
	case PIL_OFF:
		latc_data[O2_OA] &= ~(O2|OA);
		latc_data[O2_OA+1] &= ~(O2|OA);
		break;
	case PIL_ON:
		latc_data[O2_OA] |= O2;
		latc_data[O2_OA] &= ~OA;
		latc_data[O2_OA+1] |= OA;
		latc_data[O2_OA+1] &= ~O2;
		break;
	case PIL_POS:
		latc_data[O2_OA] |= O2;
		latc_data[O2_OA] &= ~OA;
		latc_data[O2_OA+1] &= ~(O2|OA);
		break;
	case PIL_NEG:
		latc_data[O2_OA+1] |= OA;
		latc_data[O2_OA+1] &= ~O2;
		latc_data[O2_OA] &= ~(O2|OA);
		break;
	}
	switch(outputs_status[5]) {
	case PIL_OFF:
		latc_data[O1_OA] &= ~(O1|OA);
		latc_data[O1_OA+1] &= ~(O1|OA);
		break;
	case PIL_ON:
		latc_data[O1_OA] |= O1;
		latc_data[O1_OA] &= ~OA;
		latc_data[O1_OA+1] |= OA;
		latc_data[O1_OA+1] &= ~O1;
		break;
	case PIL_POS:
		latc_data[O1_OA] |= O1;
		latc_data[O1_OA] &= ~OA;
		latc_data[O1_OA+1] &= ~(O1|OA);
		break;
	case PIL_NEG:
		latc_data[O1_OA+1] |= OA;
		latc_data[O1_OA+1] &= ~O1;
		latc_data[O1_OA] &= ~(O1|OA);
		break;
	}

}

static char
do_output(__ram char *buf)
{
	if (buf[1] != ' ')
		return 0;
	switch (buf[0]) {
	case '0':
		switch(buf[2]) {
		case '0':
			outputs_status[0] = 0;
			return 1;
		case '1':
			outputs_status[0] = 1;
			return 1;
		}
		return 0;
	case '1':
		switch(buf[2]) {
		case '0':
			outputs_status[1] = 0;
			return 1;
		case '1':
			outputs_status[1] = 1;
			return 1;
		}
		return 0;
	}
	return 0;
}

static char
do_pilote(__ram char *buf)
{	
	char c = buf[2] - '0';
	if (buf[1] != ' ')
		return 0;
	switch (buf[0]) {
	case '0':
		switch(c) {
		case PIL_OFF:
			outputs_status[2] = PIL_OFF;
			return 1;
		case PIL_ON:
			outputs_status[2] = PIL_ON;
			return 1;
		case PIL_POS:
			outputs_status[2] = PIL_POS;
			return 1;
		case PIL_NEG:
			outputs_status[2] = PIL_NEG;
			return 1;
		}
		return 0;
	case '1':
		switch(c) {
		case PIL_OFF:
			outputs_status[3] = PIL_OFF;
			return 1;
		case PIL_ON:
			outputs_status[3] = PIL_ON;
			return 1;
		case PIL_POS:
			outputs_status[3] = PIL_POS;
			return 1;
		case PIL_NEG:
			outputs_status[3] = PIL_NEG;
			return 1;
		}
		return 0;
	case '2':
		switch(c) {
		case PIL_OFF:
			outputs_status[4] = PIL_OFF;
			return 1;
		case PIL_ON:
			outputs_status[4] = PIL_ON;
			return 1;
		case PIL_POS:
			outputs_status[4] = PIL_POS;
			return 1;
		case PIL_NEG:
			outputs_status[4] = PIL_NEG;
			return 1;
		}
		return 0;
	case '3':
		switch(c) {
		case PIL_OFF:
			outputs_status[5] = PIL_OFF;
			return 1;
		case PIL_ON:
			outputs_status[5] = PIL_ON;
			return 1;
		case PIL_POS:
			outputs_status[5] = PIL_POS;
			return 1;
		case PIL_NEG:
			outputs_status[5] = PIL_NEG;
			return 1;
		}
		return 0;
	}
	return 0;
}

static char
command(__ram char *buf)
{
	char r = 0;
	switch (buf[0]) {
	case 'O':
		r = do_output(&buf[1]);
		break;
	case 'P':
		r = do_pilote(&buf[1]);
		break;
	}
	if (r != 0) {
		char c;
		update_outputs();
		do_outputs_status();
	}
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
	printf("deb: rx_ov %d tx_w %d 232: rx_ov %d rx_cs %d tx_w %d linky: rx_ov %d rx_cs %d\n",
	    rx_debug_ov, tx_debug_w, rx_232_ov, rx_232_cs, tx_232_w,
	    rx_linky_ov, rx_linky_cs);
}

static void
do_debug(__ram char *buf)
{
	if (strcmp(buf, "stats") == 0)
		print_stats();
	else if (strncmp(buf, "debug ", 6) == 0) {
		switch(buf[6]) {
		case 'l':
			debug_out.bits.linky = 0;
			break;
		case 'L':
			debug_out.bits.linky = 1;
			break;
		case 'a':
			debug_out.bits.adc = 0;
			break;
		case 'A':
			debug_out.bits.adc = 1;
			break;
		}
	} else {
		command(buf);
	}
}

static void
do_linky(__ram char *buf)
{
	char c;
	if (debug_out.bits.linky == 0)
		uout.bits.debug = 0;
	printf("linky ");
	uout.bits.rs232 = 1;
	printf("%s\n", buf);
	uout.bits.rs232 = 0;
	uout.bits.debug = 1;
	if (strncmp(buf, "PTEC HC", 7) == 0 && linky_state.bits.hc == 0) {
		linky_state.bits.hc = 1;
		outputs_status[0] = 1;
		update_outputs();
	} else if (strncmp(buf, "PTEC HP", 7) == 0 && linky_state.bits.hc == 1) {
		linky_state.bits.hc = 0;
		outputs_status[0] = 0;
		update_outputs();
	}
	if (strncmp(buf, "PTEC HPJR", 9) == 0) {
		if (linky_state.bits.hpjr == 0) {
			linky_state.bits.hpjr = 1;
			for (c = 2; c < 6; c++) {
				outputs_status[c] = PIL_NEG;
			}
			update_outputs();
		}
	} else if (linky_state.bits.hpjr == 1 && strncmp(buf, "PTEC H", 6) == 0) {
		linky_state.bits.hpjr = 0;
		for (c = 2; c < 6; c++) {
			outputs_status[c] = 0;
		}
		update_outputs();
	}
}

int
main(void)
{
	char c;

	default_src = 0;
	uout.byte = 0;
        if (PORTBbits.RB7) {
		uout.bits.debug_present = 1;
		U1CON1bits.U1RXBIMD = 1; /* detect RX going low */
	} else {
		U1CON1bits.U1ON = 0;
	}
	uout.bits.debug = 1;
	linky_state.byte = 0;
	debug_out.byte = 0xff;

	ANSELC = 0;
	ANSELB = 0;
	ANSELA = 0x7F; /* RA0->RA6 analog */

	LATA = 0;

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
	IPR11 = 0;
	IPR12 = 0;
	IPR13 = 0;
	IPR14 = 0;
	IPR15 = 0;
	INTCON0 = 0;
	INTCON1 = 0;
	INTCON0bits.IPEN=1; /* enable interrupt priority */

	USART_INIT(0);
	if (uout.bits.debug_present)
		PIE4bits.U1RXIE = 1;

	/* configure UART2 */
	TRISBbits.TRISB2 = 0 ;
	LATBbits.LATB2 = 1 ;
	U2RXPPS = 0x0b; /* RB3, 001 011 */
	RB2PPS = 0x23; /* RB2 */
	U2BRGL = 68; /* 57600 at 64Mhz */
	U2CON0 = 0x30; /* 00110000 */
	U2CON2bits.U2RUNOVF = 1;
	U2CON1bits.U2ON = 1;
	uart232_init();

	/* configure UART3 */
	U3RXPPS = 0x0c; /* RB4, 001 100 */
	U3BRGL = 04; /* 1200 at 64Mhz */
	U3BRGH = 13; /* 1200 at 64Mhz */
	U3CON0 = 0x10; /* 00010000 */
	U3CON2bits.U3RUNOVF = 1;
	U3CON1bits.U3ON = 1;
	linky_init();

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

	/*
	 * set up DMA engines for portC outputs.
	 * we want DMA1 to update LATC on each timer4 tick; and every 
	 * 3 writes we want to update TRISC. Unfortunably there's
	 * no way to do this in HW so we'll have to use an interrupts for this.
	 */
	/* set up timer4 for 10kHz output */
	T4CON = 0x60; /* b01100000: postscaller 1/1, prescaler 1/64 */
	T4PR = 25; /* 10kHz output */
	T4CLKCON = 0x01; /* Fosc / 4 */

	/* at startup every data is low */

	for (c = 0; c < LATC_DATA_SIZE; c++)
		latc_data[c] = 0;
	for (c = 0; c < NOUTS; c++)
		outputs_status[c] = 0;
	LATC = 0;
	/*
	 * set up DMA1 to update LATC on timer4 interrupt.
	 */
	DMASELECT = 0;
	DMAnCON0 = 0x40; /* SIRQEN */
	DMAnCON1 = 0x02; /* 00 0 00 01 0 */
	DMAnSSA = (__uint24)&latc_data[0];
	DMAnSSZ = LATC_DATA_SIZE;
	DMAnDSA = (uint16_t)&LATC;
	DMAnDSZ = 1;
	DMAnSIRQ = 0x5b; /* timer4 */
	IPR2bits.DMA1DCNTIP = 1; /* high prio */
	PIR2bits.DMA1DCNTIF = 0;
	DMAnCON0bits.EN = 1;
	PIE2bits.DMA1DCNTIE = 1;

	printf("energie %d.%d %s\n", MAJOR, MINOR, buildstr);

	/* enable watchdog */
	WDTCON0bits.SEN = 1;

	/*
	 * set up ADC
	 * triggered at 10kHz (timer4 postscaled by 2), values transfered
	 * by DMA3. We want 200 measures to cover a full 50Hz cycle
	 * this means we have 200us per measures, which should be plenty
	 * with a 1Mhz (1us) ADC clock. We'll use the computation module
	 * to sum up 8 values, so we'll have only 25 values to transfers.
	 */
	ADCON0 = 0x4; /* right-justified */
	ADCON1 = 0;
	ADCON2 = 0x3a; /* divide by 8, clear, average mode */
	ADCON3 = 0x07; /* always interrupt */
	ADCLK = 0x7F; /* Fosc/64 => 1MHz*/
	ADREF = 0x02; /* Vref- to GND, Vref+ to RA3 */
	ADPRE = 0;
	ADACQ = 128; /* 2us acquisition time */
	ADRPT = 8;
	PIR2bits.ADTIF = 0;

	for (c = 0; c < 25; c++) {
		adc_results[c] = 0;
	}

	ADPCH = 0; /* input RA0 */
	channel = 0;

	for (c = 0; c < 6; c++) {
		I_average[c] = 0;
		I_count[c] = 0;
	}
	I_timestamp = time;

	DMASELECT = 2;
	DMAnCON0 = 0x00; /* !SIRQEN */
	DMAnCON1 = 0x62; /* 01 1 00 01 0 */
	DMAnSSA = ADACCL_M2;
	DMAnSSZ = 2;
	DMAnDSA = (uint16_t)&adc_results[0];
	DMAnDSZ = 25 * 2;
	DMAnSIRQ = 0x10; /* ADC threshold interrupt */
	IPR10bits.DMA3DCNTIP = 0; /* low prio */
	PIR10bits.DMA3DCNTIF = 0;
	DMAnCON0bits.EN = 1;
	PIE10bits.DMA3DCNTIE = 1;
	ADCON0bits.ON = 1;

	/* enable the whole C outputs updates */
	T4CONbits.TMR4ON = 1;

	printf("enter loop\n");
	for (c = 0; c < LATC_DATA_SIZE; c++)
		latc_data[c] |= O_LED;

again:
	while (1) {
		if (PORTBbits.RB7) {
			uout.bits.debug_present = 1;
			PIE4bits.U1RXIE = 1;
			U1CON1bits.U1ON = 1;
		} else if (U1ERRIRbits.RXBKIF) {
			/* debug RX disconnected */
			uout.bits.debug_present = 0;
			PIE4bits.U1RXIE = 0;
			U1ERRIRbits.RXBKIF = 0;
			U1CON1bits.U1ON = 0;
		}
		time_events.byte = 0;
		if (softintrs.bits.int_adcc) {
			__uint24 adr = 0;
			if (debug_out.bits.adc) 
				printf("adcc 0x%x 0x%lx, 0x%x 0x%x 0x%x\n",
				 ADRES, (uint32_t)ADACC, ADCNT, ADSTAT, ADCON0);
			ADCON2bits.ACLR = 1;
			for (c = 0; c < 25; c++) {
				adr += adc_results[c];
			}
			if (debug_out.bits.adc) 
				printf("adcc: %lu", (uint32_t)adr);
			adr = ((uint24_t)4095 * 200) - adr;
			if (debug_out.bits.adc) 
				printf(" %lu\n", (uint32_t)adr);
			I_average[channel] += adr;
			I_count[channel]++;
			channel++;
			if (channel == 6)
				channel = 0;
			if (channel < 3)
				ADPCH = channel;
			else
				ADPCH = channel + 1;
			softintrs.bits.int_adcc = 0;
			di();
			DMASELECT = 2;
			DMAnCON0bits.SIRQEN = 1;
			ei();
			ADACT = 0x06; /* timer4_postscaled , start conversion */
		}
		if (softintrs.bits.int_100hz) {
			softintrs.bits.int_100hz = 0;
			time_events.bits.ev_100hz = 1;
			time++;
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
			seconds++;
			for (c = 0; c < LATC_DATA_SIZE; c++)
				latc_data[c] ^= O_LED;
			if ((time - I_timestamp) > 300) { /* 3s */
				printf("II %ld", (u_long)(time - I_timestamp));
				for (c = 0; c < 6; c++) {
					printf(" %ld/%d",
					    (u_long)I_average[c], I_count[c]);
					I_average[c] = 0;
					I_count[c] = 0;
				/* intensity:
				 * Iadc = I_average / I_count / 100 
				 * Iadc = I(A) 3000 * 100 / 2.048 * 4096
				 * I(A) = I_average / I_count / 6666.6667
				 */
				}
				printf("\n");
				I_timestamp = time;
			}
			/*
			printf("st %d %d\n", uart_rxbuf_idx, uart_rxbuf_a);
			printf("0x%x 0x%x 0x%x 0x%x\n", U1CON0, U1CON1, U1CON2, U1ERRIR);
			printf("st232 %d %d\n", uart232_rxbuf_idx, uart232_rxbuf_a);
			uart232_putchar('a');
			uart232_putchar('b');
			uart232_putchar('\r');
			uart232_putchar('\n');
			*/
#if 0
			printf("portC 0x%x 0x%x\n", LATC, TRISC);
			DMASELECT = 0;
			printf(" DMA0 0x%x 0x%lx 0x%x 0x%x 0x%x\n", DMAnCON0, (uint32_t)DMAnSSA, DMAnSSZ, DMAnSCNT, DMAnDCNT);
			DMASELECT = 1;
			printf(" DMA1 0x%x 0x%lx 0x%x 0x%x 0x%x\n", DMAnCON0, (uint32_t)DMAnSSA, DMAnSSZ, DMAnSCNT, DMAnDCNT);
			printf("0x%x 0x%x\n", PIR2, PIR6);
#endif
		}

		if (uart_softintrs.bits.uart1_line1) {
			printf("line1: %s\n", uart_rxbuf1);
			if (strcmp(uart_rxbuf1, "reb") == 0)
				break;
			do_debug(uart_rxbuf1);
			uart_softintrs.bits.uart1_line1 = 0;
			debug();
		} else if (uart_softintrs.bits.uart1_line2) {
			printf("line2: %s\n", uart_rxbuf2);
			if (strcmp(uart_rxbuf2, "reb") == 0)
				break;
			do_debug(uart_rxbuf2);
			uart_softintrs.bits.uart1_line2 = 0;
			debug();
		} else if (uart_rxbuf_a == 0) {
			/* clear overflow */
			uart_rxbuf_a = 1;
		}
		if (uart_softintrs.bits.uart232_line1) {
			printf("line232_1: %s\n", uart232_rxbuf1);
			command(uart232_rxbuf1);
			uart_softintrs.bits.uart232_line1 = 0;
		} else if (uart_softintrs.bits.uart232_line2) {
			printf("line232_2: %s\n", uart232_rxbuf2);
			command(uart232_rxbuf2);
			uart_softintrs.bits.uart232_line2 = 0;
		} 
		if (uart_softintrs.bits.linky_line1) {
			if (uart_softintrs.bits.linky_badcs_l1) {
				printf("linky1badcs: %s\n", linky_rxbuf1);
			} else {
				do_linky(linky_rxbuf1);
			}
			uart_softintrs.bits.linky_badcs_l1 = 0;
			uart_softintrs.bits.linky_line1 = 0;
		} else if (uart_softintrs.bits.linky_line2) {
			if (uart_softintrs.bits.linky_badcs_l2) {
				printf("linky2badcs: %s\n", linky_rxbuf2);
			} else {
				do_linky(linky_rxbuf2);
			}
			uart_softintrs.bits.linky_badcs_l2 = 0;
			uart_softintrs.bits.linky_line2 = 0;
		} 

		if (default_src != 0) {
			printf("default handler called for 0x%x\n",
			    default_src);
			default_src = 0;
		}
		if (softintrs.byte == 0 &&  uart_softintrs.byte == 0)
			SLEEP();
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

void __interrupt(__irq(DMA1DCNT), __high_priority, base(IVECT_BASE))
irqh_dma1(void)
{
	PIR2bits.DMA1DCNTIF = 0;
	DMASELECT = 0;
	switch((u_char)DMAnSCNTL) {
	case (u_char)(LATC_DATA_SIZE - O1_OA): /* O1_OA */
		TRISC = (u_char)(~(O_LED | O_I2C | O1 | OA));
		break;
	case (u_char)(LATC_DATA_SIZE - O2_OA):
		TRISC = (u_char)(~(O_LED | O_I2C | O2 | OA));
		break;
	case (u_char)(LATC_DATA_SIZE - O1_OB):
		TRISC = (u_char)(~(O_LED | O_I2C | O1 | OB));
		break;
	case (u_char)(LATC_DATA_SIZE - O2_OB):
		TRISC = (u_char)(~(O_LED | O_I2C | O2 | OB));
		break;
	case (u_char)(LATC_DATA_SIZE - O1_OC):
		TRISC = (u_char)(~(O_LED | O_I2C | O1 | OC));
		break;
	case (u_char)(LATC_DATA_SIZE - O2_OC):
		TRISC = (u_char)(~(O_LED | O_I2C | O2 | OC));
		break;
	}
	DMAnCON0bits.DGO = 1;
}

void __interrupt(__irq(DMA3DCNT), __low_priority, base(IVECT_BASE))
irqh_dma3(void)
{
	PIR10bits.DMA3DCNTIF = 0;
	ADACT = 0;
	softintrs.bits.int_adcc = 1;
}

void __interrupt(__irq(default), __low_priority, base(IVECT_BASE))
irqh_default(void)
{
	default_src = 0xff;
}

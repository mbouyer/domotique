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

static unsigned char led_pattern;
static unsigned char led_pattern_count;

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

static int16_t temp;
static uint16_t hum;
static uint8_t status;

#define STATUS_VALID 0x1
#define STATUS_VMC 0x02 /* 0 = close, 1 = open */
#define STATUS_BUTTON 0x04

#define ACTION_OPEN	0x01
#define ACTION_CLOSE	0x02
#define ACTION_RESET	0x80

static char status_update;

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


/*
 * portC outputs and definitions
 */

#define	O_LED	(u_char)0x04 /* RC2 */
#define O1	(u_char)0x10 /* RC4 */
#define O2	(u_char)0x20 /* RC5 */
#define O_I2C	(u_char)0x03 /* RC0, RC1 */

#define SHTADDR (0x44 << 1)
#define SHT_READ_STATUS		0xF32D
#define SHT_CLEAR_STATUS	0x3041
#define SHT_RESET		0x30A2
#define SHT_BREAK		0x3093
#define SHT_START_PERIODIC	0x2126
#define SHT_READ_PERIODIC	0xE000
#define SHT_MEASURE_READ	0x2C0D

enum _motor_state {
	M_SLEEP = 0,
	M_PRE_OPEN,
	M_OPEN,
	M_PRE_CLOSE,
	M_CLOSE,
	M_BREAK
} motor_state;

uint16_t motor_timer;
#define T_MOTOR_PRE	TIMER0_20MS
#define T_MOTOR_ACT	(TIMER0_100MS * 5)
#define T_MOTOR_POST	TIMER0_100MS

static void
to_motor_state(enum _motor_state s)
{
	motor_state = s;
	switch(s) {
	case M_SLEEP:
		printf("M_SLEEP\n");
		RC4PPS = 0x0; /* LATC */
		RC5PPS = 0x0; /* LATC */
		PWM1CONbits.EN = 0;
		PWM2CONbits.EN = 0;
		break;
	case M_OPEN:
		printf("M_OPEN\n");
		RC4PPS = 0x0A; /* PWM1 slice1 P1 */
		RC5PPS = 0x0C; /* PWM2 slice1 P1 */
		PWM1CONbits.EN = 1;
		PWM2CONbits.EN = 0;
		motor_timer = timer0_read();
		status |= STATUS_VMC;
		status_update = 1;
		break;
	case M_CLOSE:
		printf("M_CLOSE\n");
		RC4PPS = 0x0A; /* PWM1 slice1 P1 */
		RC5PPS = 0x0C; /* PWM2 slice1 P1 */
		PWM1CONbits.EN = 0;
		PWM2CONbits.EN = 1;
		motor_timer = timer0_read();
		status &= ~STATUS_VMC;
		status_update = 1;
		break;
	default: /* PRE_*, BREAK */
		printf("M_x %d\n", s);
		/* both output high: keep PWM in reset */
		RC4PPS = 0x0A; /* PWM1 slice1 P1 */
		RC5PPS = 0x0C; /* PWM2 slice1 P1 */
		PWM1CONbits.EN = 0;
		PWM2CONbits.EN = 0;
		motor_timer = timer0_read();
		break;
	}
}

static char
command(__ram char *buf)
{
	char r = 1;

	if (strcmp(buf, "o") == 0) {
		to_motor_state(M_PRE_OPEN);
	} else if (strcmp(buf, "c") == 0) {
		to_motor_state(M_PRE_CLOSE);
	} else {
		r = 0;
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
	uint8_t i2c_data[8]; 
	uint16_t tmr0read;

	lin_userid = (uint8_t)read_nvm_word(0x200000);
	revid = read_nvm_word(0x3ffffc);
	devid = read_nvm_word(0x3ffffe);

	default_src = 0;
	uout.byte = 0;
        if (PORTAbits.RA0) {
		uout.bits.debug_present = 1;
		U2CON1bits.U2RXBIMD = 1; /* detect RX going low */
	} else {
		U2CON1bits.U2ON = 0;
	}
	uout.bits.debug = 1;

	ANSELC = 0x08; /* RC3 analog */
	ANSELA = 0;

	LATA = 0;

	LATC = O_LED | O_I2C;
	TRISC = 0xff & ~(O_LED | O1 | O2 | O_I2C);

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

	lin_txbuf[0] = status = 0;

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

	/* configure UART1 for LIN : TX on RA4, RX on RA5 */
	LATAbits.LATA4 = 1;
	TRISAbits.TRISA4 = 0;
	TRISAbits.TRISA5 = 1;
	U1RXPPS = 0x05; /* RA5, 000 101 */
	RA4PPS = 0x10; /* U1TX */
	LIN_INIT(0);

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

	I2C_INIT;

	/*
	 * configure PWM1/2 on RC4/RC5 (O1/O2)
	 * LATC/TRISC already configured
	 */
	PWM1CON = PWM2CON = 0;
	PWM1ERS = PWM2ERS = 0;
	PWM1CLK = PWM2CLK = 3; /* HFINTOSC (64Mhz) */
	PWM1LDS = PWM2LDS = 0;
	PWM1PR = PWM2PR = 1280; /* 50Khz PWM */
	PWM1CPRE = PWM2CPRE = 0;
	PWM1PIPOS = PWM2PIPOS = 0;
	PWM1PIPOS = PWM2PIPOS = 0;
	PWM1S1CFG = PWM2S1CFG = 0x40; /* left aligned, output 1 active low */
	PWM1S1P1 = PWM2S1P1 = 320; /* duty = 25% */
	PWM1CONbits.LD = 1;
	PWM2CONbits.LD = 1;

	motor_state = M_SLEEP;

	INTCON0bits.GIEH=1;  /* enable high-priority interrupts */   
	INTCON0bits.GIEL=1; /* enable low-priority interrrupts */   

	printf("vmc %d.%d %s\n", MAJOR, MINOR, buildstr);
	printf("lin_id 0x%x device 0x%x rev 0x%x\n", lin_userid, devid, revid);

	/* enable watchdog */
	WDTCON0bits.SEN = 1;

	/*
	 * set up ADC
	 */
	FVRCON  = 0x82; /* vref = 2.048 */
	ADCON0 = 0x4; /* right-justified */
	ADCON1 = 0;
	ADCON2 = 0x3b; /* divide by 8, clear, burst average mode */
	ADCON3 = 0x07; /* always interrupt */
	ADCLK = 0x7F; /* Fosc/64 => 1MHz*/
	ADREF = 0x03; /* Vref- to GND, Vref+ to FVR */
	ADPRE = 0x08; /* SOI */
	ADACQ = 128; /* 2us acquisition time */
	ADRPT = 8;
	ADCON0bits.ON = 1;
	PIR2bits.ADTIF = 0;
	PIE2bits.ADTIE = 1;

	ADPCH = 0x13; /* input RC3 */

	printf("reset SHT31\n");

	if (i2c_writereg(SHTADDR, SHT_BREAK, i2c_data, 0) != 1) {
		printf("i2c SHT_BREAK failed\n");
	}
	delay(TIMER0_1MS);

	if (i2c_writereg(SHTADDR, SHT_RESET, i2c_data, 0) != 1) {
		printf("i2c SHT_RESET failed\n");
	}
	delay(TIMER0_1MS);


	if (i2c_readreg(SHTADDR, SHT_READ_STATUS, i2c_data, 3) != 3) {
		printf("i2c SHT_READ_STATUS failed\n");
	} else {
		printf("i2c status 0x%02x%02x 0x%x\n", i2c_data[0], i2c_data[1], i2c_data[2]);
	}

	if (i2c_writereg(SHTADDR, SHT_CLEAR_STATUS, i2c_data, 0) != 1) {
		printf("i2c SHT_CLEAR_STATUS failed\n");
	}

#if 0
	if (i2c_writereg(SHTADDR, SHT_START_PERIODIC, i2c_data, 0) != 1) {
		printf("i2c SHT_START_PERIODIC failed\n");
	}
#endif

	printf("enter loop\n");
	LATC |= O_LED;

	led_pattern = 0xff;
	led_pattern_count = 8;

	to_motor_state(M_PRE_CLOSE);

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
			printf("adcc 0x%x 0x%lx, 0x%x 0x%x 0x%x\n",
			    ADRES, (uint32_t)ADACC, ADCNT, ADSTAT, ADCON0);
			ADCON2bits.ACLR = 1;
			softintrs.bits.int_adcc = 0;
			if (motor_state != M_SLEEP)
				ADCON0bits.GO = 1;
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
			if (led_pattern_count != 0) {
				if (led_pattern & 0x01) {
					LATC |= O_LED;
				} else {
					LATC &= ~O_LED;
				}
				led_pattern_count--;
				led_pattern >>= 1;
			} else {
				LATC &= ~O_LED;
			}
		}
		if (time_events.bits.ev_1hz) {
			seconds++;
			if ((seconds % 10) == 0) {
				uint16_t r_t, r_h;
				float t, h;
#if 0
				if (i2c_readreg(SHTADDR, SHT_READ_PERIODIC,
				    i2c_data, 6) != 6)
#else
				if (i2c_readreg(SHTADDR, SHT_MEASURE_READ,
				    i2c_data, 6) != 6) {
#endif
					printf("i2c SHT_FETCH_DATA failed\n");
					if (i2c_readreg(SHTADDR,
					    SHT_READ_STATUS, i2c_data, 3) != 3) {
						printf("i2c SHT_READ_STATUS "
						    "failed\n");
					} else {
						printf("i2c status 0x%02x%02x 0x%x\n",
						    i2c_data[0], i2c_data[1], i2c_data[2]);
					}
					led_pattern_count = 8;
					led_pattern = 0xaa;
					status &= ~STATUS_VALID;
				} else {
					led_pattern_count = 2;
					led_pattern = 1;
					printf("i2c data 0x%02x%02x 0x%x "
					    "0x%02x%02x 0x%x\n",
					    i2c_data[0], i2c_data[1],
					    i2c_data[2],
					    i2c_data[3], i2c_data[4],
					    i2c_data[5]);
					r_t = (uint16_t)i2c_data[0] << 8 |
					    i2c_data[1];
					r_h = (uint16_t)i2c_data[3] << 8 |
					    i2c_data[4];

					t = (float)r_t / (float)65535.0;
					t = (float)-45.0 + (float)175.0 * t;
					h = (float)r_h / (float)65535.0;
					h = h * (float)100.0;
					printf("t %lf h %lf\n", t, h);
					temp = (int16_t)(t * 100.0);
					hum = (uint16_t)(h * 100.0);
					status |= STATUS_VALID;
					status_update = 1;
				}
			}
		}
		if (status_update && lin_get()) {
			lin_txbuf[0] = status;
			*((int16_t *)&lin_txbuf[1]) = temp;
			*((uint16_t *)&lin_txbuf[3]) = hum;
			lin_put();
			status_update = 0;
		}

		/* motor FSM */
		switch(motor_state) {
		case M_SLEEP:
			break;
		case M_PRE_OPEN:
			if ((timer0_read() - motor_timer) >= T_MOTOR_PRE)
				to_motor_state(M_OPEN);
			break;
		case M_PRE_CLOSE:
			if ((timer0_read() - motor_timer) >= T_MOTOR_PRE)
				to_motor_state(M_CLOSE);
			break;
		case M_OPEN:
		case M_CLOSE:
			if ((timer0_read() - motor_timer) >= T_MOTOR_ACT)
				to_motor_state(M_BREAK);
			led_pattern_count = 2;
			led_pattern = 1;
			break;
		case M_BREAK:
			if ((timer0_read() - motor_timer) >= T_MOTOR_POST)
				to_motor_state(M_SLEEP);
			break;
		}

		if (uart_softintrs.bits.int_linpid) {
			printf("lin pid 0x%x\n", lin_pid);
			uart_softintrs.bits.int_linpid = 0;
		}
		if (uart_softintrs.bits.int_linrx) {
			printf("lin rx from 0x%x: 0x%x 0x%x\n", lin_pid, lin_rxbuf[0], lin_rxbuf[1]);       

			if (lin_rxbuf[0] & ACTION_OPEN) {
				to_motor_state(M_PRE_OPEN);
			} else if (lin_rxbuf[0] & ACTION_CLOSE) {
				to_motor_state(M_PRE_CLOSE);
			}
			if (lin_rxbuf[0] & ACTION_RESET) {
				goto do_reset;
			}
			uart_softintrs.bits.int_linrx = 0;
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

void __interrupt(__irq(IRQ_ADT), __low_priority, base(IVECT_BASE))
irqh_adc(void)
{
	PIR2bits.ADTIF = 0;
	softintrs.bits.int_adcc = 1;
}

void __interrupt(__irq(default), __low_priority, base(IVECT_BASE))
irqh_default(void)
{
	default_src = 0xff;
}

/*
 * Copyright (c) 2022 Manuel Bouyer
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

#define I2C_INIT(adr) { \
	i2csoftintrs.byte = 0; \
	RC0I2C = 0x41; /* I2C fast slew-rate, no pull-up, i2c threshold */ \
	RC1I2C = 0x41; /* I2C fast slew-rate, no pull-up, i2c threshold */ \
	RC0PPS = 0x21; /* SCL */ \
	RC1PPS = 0x22; /* SDA */ \
	I2C1SCLPPS = 0x10; /* RC0 */ \
	I2C1SDAPPS = 0x11; /* RC1 */ \
	ODCONCbits.ODCC0 = 1; \
	ODCONCbits.ODCC1 = 1; \
	LATCbits.LATC0 = 1; \
	LATCbits.LATC1 = 1; \
	TRISCbits.TRISC0 = 0; \
	TRISCbits.TRISC1 = 0; \
	I2C1CON0 = 0x00; /* 7 bits I2C client mode */ \
	I2C1CON1 = 0x80; /* ACKCNT */ \
	I2C1CON2 = 0; /* address buffer enabled */ \
	I2C1ADR0 = ((adr) << 1); /* set address */ \
	I2C1ADR1 = ((adr) << 1); /* set address */ \
	I2C1ADR2 = ((adr) << 1); /* set address */ \
	I2C1ADR3 = ((adr) << 1); /* set address */ \
	I2C1PIR = 0; \
	I2C1PIE = 0x01; /* interrupt on start condition */\
	I2C1ERR = 0x07; /* interrupt on all errors */\
	PIE7bits.I2C1IE = 1; \
	PIE7bits.I2C1TXIE = 1; \
	PIE7bits.I2C1RXIE = 1; \
	PIE7bits.I2C1EIE = 1; \
	I2C1CON0bits.EN = 1; \
	I2C1STAT1bits.CLRBF = 1; \
    }

#define I2C_LOCK { INTCON0bits.GIEL = 0; } /* disable interrupts */
#define I2C_UNLOCK { INTCON0bits.GIEL = 1; } /* enable interrupts */

#define N_I2CREGS 8
extern uint8_t i2c_values[N_I2CREGS];
extern uint8_t i2c_values_wr[N_I2CREGS];
extern volatile uint8_t i2c_writes;

extern uint8_t i2c_pir;
extern uint8_t i2c_rxi;
extern uint8_t i2c_txi;
extern uint8_t i2c_err;

extern volatile union i2csoftintrs {
        struct i2csoftintrs_bits {
		char i2c_i : 1;   
		char i2c_irx : 1;
		char i2c_itx : 1;
		char i2c_ie : 1;
	} bits;
	char byte;
} i2csoftintrs;

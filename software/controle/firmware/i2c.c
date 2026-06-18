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

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>

#define DEBUG 1

#ifdef DEBUG
#define DPRINTF(x) printf x
#else
#define DPRINTF(x) /**/
#endif

uint8_t i2c_values[N_I2CREGS];
uint8_t i2c_values_wr[N_I2CREGS];
volatile uint8_t i2c_writes;
uint8_t i2c_reg;
uint8_t i2c_wrreg;

uint8_t i2c_pir;
uint8_t i2c_rxi;
uint8_t i2c_txi;
uint8_t i2c_err;
volatile union i2csoftintrs i2csoftintrs;

static void i2c_status(void);

static void
i2c_status(void)
{
	printf("CON 0x%x 0x%x 0x%x", I2C1CON0, I2C1CON1, I2C1CON2);
	printf(" STAT 0x%x 0x%x PIR 0x%x", I2C1STAT0, I2C1STAT1, I2C1PIR);
	printf(" ERR 0x%x CNT 0x%x 0x%x\n", I2C1ERR, I2C1CNTH, I2C1CNTL);
}

void __interrupt(__irq(IRQ_I2C1), __low_priority, base(IVECT_BASE))
irqh_i2c(void)
{
	if (I2C1PIRbits.SCIF) {
		i2c_wrreg = 1;
		I2C1CNTL = 0xff;
	} else {
		i2c_pir = I2C1PIR;
		i2csoftintrs.bits.i2c_i = 1;
	}
	I2C1PIR = 0;
}

void __interrupt(__irq(IRQ_I2C1RX), __low_priority, base(IVECT_BASE))
irqh_i2crx(void)
{
#if 0
	i2c_cnt = I2C1CNTL;
	i2csoftintrs.bits.i2c_irx = 1;
#endif
	if (i2c_wrreg) {
		i2c_reg = I2C1RXB;
		i2c_reg = i2c_reg % N_I2CREGS;
		i2c_wrreg = 0;
	} else {
		i2c_values_wr[i2c_reg] = I2C1RXB;
		i2c_writes |= (0x01 << i2c_reg);
		i2c_reg = (i2c_reg + 1) % N_I2CREGS;
	}
}

void __interrupt(__irq(IRQ_I2C1TX), __low_priority, base(IVECT_BASE))
irqh_i2ctx(void)
{
#if 0
	i2c_cnt = I2C1CNTL;
	i2csoftintrs.bits.i2c_itx = 1;
#endif
	I2C1TXB = i2c_values[i2c_reg];
	i2c_reg = (i2c_reg + 1) % N_I2CREGS;
}

void __interrupt(__irq(IRQ_I2C1E), __low_priority, base(IVECT_BASE))
irqh_i2ce(void)
{
	if (I2C1ERRbits.NACKIF) {
		I2C1STAT1bits.CLRBF = 1;
	} else {
		i2c_err = I2C1ERR;
		i2csoftintrs.bits.i2c_ie = 1;
	}
	I2C1ERR = 0x07;
}

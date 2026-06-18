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

/* API between host and controle board's I2C manager */

#define CI2C_MAJOR 1
#define CI2C_MINOR 0

#define I2C_ADDR 0x22

/* registers definition */
#define CI2C_R_VERSION 	0
#define MAJOR_MASK 0xf0
#define MAJOR_SHIFT 4
#define MINOR_MASK 0x0f
#define MINOR_SHIFT 0


#define CI2C_R_LIGHTL 1
#define CI2C_R_LIGHTH 2

#define CI2C_R_PWROFF0	4
#define CI2C_R_PWROFF0_MAGIC	0xde
#define CI2C_R_PWROFF1	5
#define CI2C_R_PWROFF1_MAGIC	0xad
#define CI2C_R_PWROFF2	6
#define CI2C_R_PWROFF2_MAGIC	0xbe
#define CI2C_R_PWROFF3	7
#define CI2C_R_PWROFF3_MAGIC	0xef

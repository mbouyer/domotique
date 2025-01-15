ROM_BASE=300
IVECT_BASE=0x308

.SUFFIXES: .p1 .hex

.PATH: ${.CURDIR}/../../../pic18_n2k

CC= xc8-cc -mcpu=18f25q43 -mno-config -mkeep-startup -O2
CC+= -mcodeoffset=${ROM_BASE} -mreserve=rom@0x80000:0xfffff -mreserve=ram@0x0d00:0x0dff
CFLAGS= -DIVECT_BASE=${IVECT_BASE} -I${.CURDIR} -I${.CURDIR}/../../../pic18_n2k
CFLAGS+= -DN2K_PRINTF
OBJECTS= main.p1 serial.p1
HEADERS= serial.h vers.h

all: energie.bin

obj:
	mkdir obj

energie.bin: energie.hex
	hex2bin 0x300 ${.ALLSRC} ${.TARGET}
energie.hex: ${OBJECTS} vers.p1
	${CC} -Wa,-a -Wl,-Map=energie.map  -o ${.TARGET} ${.ALLSRC} 

${OBJECTS}: ${HEADERS} Makefile

vers.c: ${OBJECTS} ${.CURDIR}/mkvers.sh
	${.CURDIR}/mkvers.sh ${.TARGET}
	
.c.p1:
	${CC} ${CFLAGS} -c ${.IMPSRC} -o ${.TARGET}

clean:
	rm -f *.p1 *.hex

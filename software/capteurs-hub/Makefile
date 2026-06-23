NOMAN=

PROG=capteurs-hub capteurs-client

SRCS.capteurs-hub= bme280.c capteurs-hub.c

CFLAGS+= -I${.CURDIR}/..

DPADD.capteurs-hub +=${LIBUTIL}
LDADD.capteurs-hub +=-lutil

.include <bsd.prog.mk>

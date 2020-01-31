# $FreeBSD$

.PATH: ${.CURDIR}/../../contrib/lib9p
CFLAGS+=-I${.CURDIR}/../../contrib/lib9p
CFLAGS+=-DWITH_CASPER
DEBUG_FLAGS=	-DL9P_DEBUG=L9P_DEBUG -DACE_DEBUG -g -O0

LIB=		9p
PACKAGE=	lib${LIB}
SHLIB_MAJOR=	1
SRCS=		connection.c \
		genacl.c \
		hashtable.c \
		log.c \
		pack.c \
		request.c \
		rfuncs.c \
		threadpool.c \
		utils.c \
		backend/fs.c \
		transport/socket.c

INCSDIR=	${INCLUDEDIR}/lib9p
INCS=		fid.h lib9p.h backend/fs.h

LIBADD=		sbuf

.include <bsd.lib.mk>

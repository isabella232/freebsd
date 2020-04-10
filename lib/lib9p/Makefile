# $FreeBSD$
# Note: to turn on debug, use -DL9P_DEBUG=L9P_DEBUG,
# and set env variable LIB9P_LOGGING to stderr or to
# the (preferably full path name of) the debug log file.

CFLAGS+=	-DWITH_CASPER -I${.CURDIR}
DEBUG_FLAGS=	-DL9P_DEBUG=L9P_DEBUG -DACE_DEBUG -g -O0

LIB=		9p
PACKAGE=	lib${LIB}
SHLIB_MAJOR=	1
SHLIBDIR?=	/lib
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

cscope: .PHONY
	cd ${.CURDIR}; cscope -buq $$(find . -name '*.[ch]' -print)

.include <bsd.lib.mk>

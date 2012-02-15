noinst_LTLIBRARIES = libutils.la
libutils_la_SOURCES = eps.cc

utilsincludedir = $(includedir)/numerics/utils

INCLUDES = $(FMATVEC_CFLAGS)
LDADD = $(FMATVEC_LIBS)

libutils_la_CPPFLAGS = -I.. -DTIXML_USE_STL $(FMATVEC_CFLAGS)
libutils_la_LIBADD = $(FMATVEC_LIBS)

utilsinclude_HEADERS = eps.h

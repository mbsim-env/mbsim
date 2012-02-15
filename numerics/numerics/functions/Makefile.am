noinst_LTLIBRARIES = libfunctions.la
libfunctions_la_SOURCES = function.cc

functionsincludedir = $(includedir)/numerics/functions

INCLUDES = $(FMATVEC_CFLAGS)
LDADD = $(FMATVEC_LIBS)

libfunctions_la_CPPFLAGS = -I.. -DTIXML_USE_STL $(FMATVEC_CFLAGS)
libfunctions_la_LIBADD = $(FMATVEC_LIBS)

functionsinclude_HEADERS = function.h

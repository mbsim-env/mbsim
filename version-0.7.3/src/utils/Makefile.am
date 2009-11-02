noinst_LTLIBRARIES = libutils.la
libutils_la_SOURCES = eps.cc\
		      nonlinear_algebra.cc\
		      nonsmooth_algebra.cc\
		      function.cc\
		      ppolynom.cc\
		      contact_utils.cc\
                      stopwatch.cc\
		      rotarymatrices.cc

libutils_la_CPPFLAGS = -I..
utilsincludedir = $(includedir)/mbsim

INCLUDES = $(DEPS_CFLAGS)
LDADD = $(DEPS_LIBS)

utilsinclude_HEADERS = eps.h\
		       nonlinear_algebra.h\
		       nonsmooth_algebra.h\
		       function.h\
		       ppolynom.h\
	               contact_utils.h\
                       stopwatch.h\
		       rotarymatrices.h

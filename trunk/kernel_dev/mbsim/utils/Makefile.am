noinst_LTLIBRARIES = libutils.la
libutils_la_SOURCES = eps.cc\
                      nonlinear_algebra.cc\
                      nonsmooth_algebra.cc\
                      ppolynom.cc\
                      contact_utils.cc\
                      rotarymatrices.cc\
                      utils.cc

utilsincludedir = $(includedir)/mbsim/utils

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

utilsinclude_HEADERS = eps.h\
                       nonlinear_algebra.h\
                       nonsmooth_algebra.h\
                       function.h\
                       ppolynom.h\
                       contact_utils.h\
                       rotarymatrices.h\
                       utils.h


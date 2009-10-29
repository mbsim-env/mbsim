noinst_LTLIBRARIES = libutils.la
libutils_la_SOURCES = eps.cc\
                      nonlinear_algebra.cc\
                      nonsmooth_algebra.cc\
                      ppolynom.cc\
                      contact_utils.cc\
                      rotarymatrices.cc\
                      utils.cc\
                      function.cc\
                      function_library.cc\
                      ansatz_functions.cc

utilsincludedir = $(includedir)/mbsim/utils

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

utilsinclude_HEADERS = eps.h\
                       nonlinear_algebra.h\
                       nonsmooth_algebra.h\
                       function.h\
                       function_library.h\
                       ppolynom.h\
                       contact_utils.h\
                       rotarymatrices.h\
                       utils.h\
                       ansatz_functions.h\
											 contour_functions.h





bin_SCRIPTS = mbsiminit.sh mbsiminit.csh mbsiminit.ksh mbsiminit.bat

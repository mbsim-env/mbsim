noinst_LTLIBRARIES = libutils.la
libutils_la_SOURCES = colors.cc\
                      eps.cc\
                      nonlinear_algebra.cc\
                      nonsmooth_algebra.cc\
                      ppolynom.cc\
                      contact_utils.cc\
                      rotarymatrices.cc\
                      utils.cc\
                      stopwatch.cc\
                      ansatz_functions.cc\
                      openmbv_utils.cc

utilsincludedir = $(includedir)/mbsim/utils

libutils_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)
libutils_la_CPPFLAGS = -I$(top_srcdir) -DTIXML_USE_STL  $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)

utilsinclude_HEADERS = colors.h\
                       eps.h\
                       nonlinear_algebra.h\
                       nonsmooth_algebra.h\
                       ppolynom.h\
                       contact_utils.h\
                       rotarymatrices.h\
                       utils.h\
                       stopwatch.h\
                       ansatz_functions.h\
                       contour_functions.h\
		       openmbv_utils.h

if COND_CASADI
  MAYBE_CPPCASADITOXML = cppCasADiToXML
endif
noinst_PROGRAMS = $(MAYBE_CPPCASADITOXML)
cppCasADiToXML_SOURCES = cppCasADiToXML.cc
cppCasADiToXML_LDADD = libutils.la
cppCasADiToXML_CPPFLAGS = $(libutils_la_CPPFLAGS)

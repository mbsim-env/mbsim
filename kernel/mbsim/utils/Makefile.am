noinst_LTLIBRARIES = libutils.la
libutils_la_SOURCES = colors.cc\
                      eps.cc\
                      nonlinear_algebra.cc\
                      nonsmooth_algebra.cc\
                      contact_utils.cc\
                      rotarymatrices.cc\
                      utils.cc\
                      xmlutils.cc\
                      stopwatch.cc\
                      ansatz_functions.cc\
                      openmbv_utils.cc

utilsincludedir = $(includedir)/mbsim/utils

libutils_la_LIBADD = $(LDFLAGS_NOUNDEF) $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)
libutils_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)

utilsinclude_HEADERS = colors.h\
                       eps.h\
                       nonlinear_algebra.h\
                       nonsmooth_algebra.h\
                       contact_utils.h\
                       rotarymatrices.h\
                       utils.h\
                       xmlutils.h\
                       plotfeatureenum.h\
                       initconfigenum.h\
                       stopwatch.h\
                       ansatz_functions.h\
		       boost_parameters.h\
		       openmbv_utils.h\
		       index.h

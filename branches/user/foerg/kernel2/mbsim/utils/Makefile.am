noinst_LTLIBRARIES = libutils.la
libutils_la_SOURCES = eps.cc\
                      nonlinear_algebra.cc\
                      nonsmooth_algebra.cc\
		      contact_utils.cc\
                      rotarymatrices.cc\
                      utils.cc\
                      function.cc\
                      function_library.cc

utilsincludedir = $(includedir)/mbsim/utils

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)  $(MBSIMNUMERICS_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)  $(MBSIMNUMERICS_LIBS)
libutils_la_CPPFLAGS = -DTIXML_USE_STL

utilsinclude_HEADERS = eps.h\
                       nonlinear_algebra.h\
                       nonsmooth_algebra.h\
                       function.h\
                       function_library.h\
		       contact_utils.h\
                       rotarymatrices.h\
                       utils.h

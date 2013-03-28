noinst_LTLIBRARIES = libutils.la
libutils_la_SOURCES = colors.cc\
                      eps.cc\
                      nonlinear_algebra.cc\
                      nonsmooth_algebra.cc\
                      ppolynom.cc\
                      contact_utils.cc\
                      rotarymatrices.cc\
                      utils.cc\
                      function.cc\
                      function_library.cc\
                      stopwatch.cc\
                      ansatz_functions.cc

utilsincludedir = $(includedir)/mbsim/utils

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)
libutils_la_CPPFLAGS = -DTIXML_USE_STL

utilsinclude_HEADERS = colors.h\
                       eps.h\
                       nonlinear_algebra.h\
                       nonsmooth_algebra.h\
                       function.h\
                       differentiable_function_library.h\
                       function_library.h\
                       ppolynom.h\
                       contact_utils.h\
                       rotarymatrices.h\
                       utils.h\
                       stopwatch.h\
                       ansatz_functions.h\
                       contour_functions.h

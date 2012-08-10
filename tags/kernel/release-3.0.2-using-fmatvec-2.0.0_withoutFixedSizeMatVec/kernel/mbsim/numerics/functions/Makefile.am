noinst_LTLIBRARIES = libfunctions.la
libfunctions_la_SOURCES = criteria_functions.cc\
                          damping_functions.cc\
                          lcp_reformulation_functions.cc\
                          newton_method_jacobian_functions.cc

functionsincludedir = $(includedir)/mbsim/numerics/functions

INCLUDES = $(FMATVEC_CFLAGS)
LDADD = $(FMATVEC_LIBS)

libfunctions_la_CPPFLAGS = -I.. -DTIXML_USE_STL $(DEPS_CFLAGS)
libfunctions_la_LIBADD = $(DEPS_LIBS)

functionsinclude_HEADERS = criteria_functions.h\
                           damping_functions.h\
                           lcp_reformulation_functions.h\
                           newton_method_jacobian_functions.h

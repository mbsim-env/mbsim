noinst_LTLIBRARIES = libfunctions.la
libfunctions_la_SOURCES = criteria_functions.cc\
                          damping_functions.cc\
                          function.cc\
                          lcp_reformulation_functions.cc\
                          newton_method_jacobian_functions.cc

functionsincludedir = $(includedir)/numerics/functions

INCLUDES = $(FMATVEC_CFLAGS)
LDADD = $(FMATVEC_LIBS)

libfunctions_la_CPPFLAGS = -I.. -DTIXML_USE_STL $(FMATVEC_CFLAGS)
libfunctions_la_LIBADD = $(FMATVEC_LIBS)

functionsinclude_HEADERS = criteria_functions.h\
                           damping_functions.h\
                           function.h\
                           lcp_reformulation_functions.h\
                           newton_method_jacobian_functions.h

noinst_LTLIBRARIES = libfunctions.la
libfunctions_la_SOURCES = criteria_functions.cc\
                          damping_functions.cc\
                          lcp_reformulation_functions.cc\
                          newton_method_jacobian_functions.cc

functionsincludedir = $(includedir)/mbsim/numerics/functions

libfunctions_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(FMATVEC_CFLAGS)
libfunctions_la_LIBADD = $(LDFLAGS_NOUNDEF) $(DEPS_LIBS) $(FMATVEC_LIBS)

functionsinclude_HEADERS = criteria_functions.h\
                           damping_functions.h\
                           lcp_reformulation_functions.h\
                           newton_method_jacobian_functions.h

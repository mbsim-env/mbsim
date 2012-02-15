noinst_LTLIBRARIES = libnonlinear_algebra.la
libnonlinear_algebra_la_SOURCES = lemke_algorithm.cc\
                                  multi_dimensional_fixpoint_solver.cc\
                                  multi_dimensional_newton_method.cc

nonlinear_algebraincludedir = $(includedir)/numerics/nonlinear_algebra

INCLUDES = $(FMATVEC_CFLAGS)
LDADD = $(FMATVEC_LIBS)

libnonlinear_algebra_la_CPPFLAGS = -I.. -DTIXML_USE_STL $(FMATVEC_CFLAGS)
libnonlinear_algebra_la_LIBADD = $(FMATVEC_LIBS)

nonlinear_algebrainclude_HEADERS = lemke_algorithm.h\
                                   multi_dimensional_fixpoint_solver.h\
                                   multi_dimensional_newton_method.h

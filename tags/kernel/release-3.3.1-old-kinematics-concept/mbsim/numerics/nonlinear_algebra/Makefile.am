noinst_LTLIBRARIES = libnonlinear_algebra.la
libnonlinear_algebra_la_SOURCES = multi_dimensional_fixpoint_solver.cc\
                                  multi_dimensional_newton_method.cc

nonlinear_algebraincludedir = $(includedir)/mbsim/numerics/nonlinear_algebra

libnonlinear_algebra_la_CPPFLAGS = -I$(top_srcdir) -DTIXML_USE_STL $(DEPS_CFLAGS)
libnonlinear_algebra_la_LIBADD = $(DEPS_LIBS)

nonlinear_algebrainclude_HEADERS = multi_dimensional_fixpoint_solver.h\
                                   multi_dimensional_newton_method.h

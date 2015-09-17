SUBDIRS = functions nonlinear_algebra linear_complementarity_problem nurbs .

noinst_LTLIBRARIES = libnumerics.la

libnumerics_la_SOURCES = csparse.c\
                         gaussian_quadratur.cc
          
libnumerics_la_LIBADD = functions/libfunctions.la 
libnumerics_la_LIBADD += linear_complementarity_problem/liblinear_complementarity_problem.la 
libnumerics_la_LIBADD += nonlinear_algebra/libnonlinear_algebra.la
libnumerics_la_LIBADD += nurbs/libnurbs.la 
libnumerics_la_LIBADD += -lgfortran $(DEPS_LIBS)

libnumerics_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS)

numericsincludedir = $(includedir)/mbsim/numerics


numericsinclude_HEADERS = csparse.h\
                          gaussian_quadratur.h
                          
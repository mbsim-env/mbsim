noinst_LTLIBRARIES = liblinear_complementarity_problem.la
liblinear_complementarity_problem_la_SOURCES = lemke_algorithm.cc\
                                               linear_complementarity_problem.cc

linear_complementarity_problemincludedir = $(includedir)/numerics/linear_complementarity_problem

INCLUDES = $(FMATVEC_CFLAGS)
LDADD = $(FMATVEC_LIBS)

liblinear_complementarity_problem_la_CPPFLAGS = -I.. -DTIXML_USE_STL $(FMATVEC_CFLAGS)
liblinear_complementarity_problem_la_LIBADD = $(FMATVEC_LIBS)

linear_complementarity_probleminclude_HEADERS = lemke_algorithm.h\
                                                linear_complementarity_problem.h

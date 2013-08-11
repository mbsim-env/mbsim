noinst_LTLIBRARIES = liblinear_complementarity_problem.la
liblinear_complementarity_problem_la_SOURCES = lemke_algorithm.cc\
                                               linear_complementarity_problem.cc

linear_complementarity_problemincludedir = $(includedir)/mbsim/numerics/linear_complementarity_problem

liblinear_complementarity_problem_la_CPPFLAGS = -I$(top_srcdir) -DTIXML_USE_STL $(DEPS_CFLAGS) $(FMATVEC_CFLAGS)
liblinear_complementarity_problem_la_LIBADD = $(DEPS_LIBS) $(FMATVEC_LIBS)

linear_complementarity_probleminclude_HEADERS = lemke_algorithm.h\
                                                linear_complementarity_problem.h

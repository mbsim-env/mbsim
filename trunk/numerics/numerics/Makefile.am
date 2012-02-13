lib_LTLIBRARIES = libnumerics.la
libnumerics_la_LDFLAGS = -version-info @LIBNUMERICS_VERSION_INFO@
libnumerics_la_SOURCES = eps.cc\
                         function.cc\
                         lemke_algorithm.cc\
                         linear_complementarity_problem.cc\
                         multi_dimensional_fixpoint_solver.cc\
                         multi_dimensional_newton_method.cc\
                         nonsmooth_algebra.cc
          
libnumerics_la_LIBADD = -lgfortran

libnumerics_la_CPPFLAGS = -DTIXML_USE_STL          
                    
INCLUDES = $(FMATVEC_CFLAGS)
LDADD = $(FMATVEC_LIBS)

numericsincludedir = $(includedir)/numerics
numericsinclude_HEADERS = eps.h\
                          function.h\
                          lemke_algorithm.h\
                          linear_complementarity_problem.h\
                          multi_dimensional_fixpoint_solver.h\
                          multi_dimensional_newton_method.h\
                          nonsmooth_algebra.h
                        
SUBDIRS = functions utils nonlinear_algebra linear_complementarity_problem .

lib_LTLIBRARIES = libnumerics.la
libnumerics_la_LDFLAGS = -version-info @LIBNUMERICS_VERSION_INFO@
libnumerics_la_SOURCES = nonsmooth_algebra.cc
          
libnumerics_la_LIBADD = functions/libfunctions.la linear_complementarity_problem/liblinear_complementarity_problem.la nonlinear_algebra/libnonlinear_algebra.la utils/libutils.la -lgfortran

libnumerics_la_CPPFLAGS = -DTIXML_USE_STL          
                    
INCLUDES = $(FMATVEC_CFLAGS)
LDADD = $(FMATVEC_LIBS)

numericsincludedir = $(includedir)/numerics
numericsinclude_HEADERS = nonsmooth_algebra.h
                        
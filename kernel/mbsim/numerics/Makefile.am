SUBDIRS = functions nonlinear_algebra linear_complementarity_problem .

noinst_LTLIBRARIES = libnumerics.la

#libnumerics_la_LDFLAGS = -version-info @LIBNUMERICS_VERSION_INFO@
libnumerics_la_SOURCES = 
          
libnumerics_la_LIBADD = functions/libfunctions.la linear_complementarity_problem/liblinear_complementarity_problem.la nonlinear_algebra/libnonlinear_algebra.la -lgfortran

#libnumerics_la_CPPFLAGS = -DTIXML_USE_STL
                    
INCLUDES = $(DEPS_CFLAGS)
LDADD = $(DEPS_LIBS)

numericsincludedir = $(includedir)/mbsim/numerics
numericsinclude_HEADERS =
                        
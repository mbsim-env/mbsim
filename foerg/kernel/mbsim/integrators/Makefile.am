noinst_LTLIBRARIES = libintegrators.la
libintegrators_la_SOURCES = integrator.cc\
			     lsodar_integrator.cc \
			     lsoder_integrator.cc \
			     lsode_integrator.cc \
			     rksuite_integrator.cc \
			     dopri5_integrator.cc \
			     dop853_integrator.cc \
			     odex_integrator.cc \
			     radau5_integrator.cc \
			     time_stepping_integrator.cc \
           time_stepping_ssc_integrator.cc \
           time_stepping_d1minuslinear_integrator.cc \
			     theta_time_stepping_integrator.cc \
			     euler_explicit_integrator.cc \
			     fortran/opkda1.f\
			     fortran/opkda2.f\
			     fortran/opkdmain.f\
			     fortran/dopri5.f\
			     fortran/dop853.f\
			     fortran/odex.f\
			     fortran/radau5.f\
			     fortran/dc_lapack.f\
			     fortran/rksuite.f
			     
#daspk_integrator.cc \
#fortran/ddaspk.f\
#fortran/daux.f

#fortran/mexax.f
#mexax_integrator.cc 

libintegrators_la_CPPFLAGS = -Ifortran -DTIXML_USE_STL
integratorsincludedir = $(includedir)/mbsim/integrators

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

integratorsinclude_HEADERS = integrator.h \
			      integrators.h \
			      time_stepping_integrator.h \
            time_stepping_ssc_integrator.h \
            time_stepping_d1minuslinear_integrator.h \
			      theta_time_stepping_integrator.h \
			      euler_explicit_integrator.h \
			      lsodar_integrator.h \
			      lsoder_integrator.h \
			      lsode_integrator.h \
			      rksuite_integrator.h \
			      dopri5_integrator.h \
			      dop853_integrator.h \
			      odex_integrator.h \
			      radau5_integrator.h \
			      fortran/fortran_wrapper.h
#daspk_integrator.h

# mexax_integrator.h 


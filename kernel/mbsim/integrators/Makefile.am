noinst_LTLIBRARIES = libintegrators.la
libintegrators_la_SOURCES = integrator.cc\
			    lsode_integrator.cc \
			    lsoda_integrator.cc \
			    lsodar_integrator.cc \
			    lsodkr_integrator.cc \
			    lsodi_integrator.cc \
			    rksuite_integrator.cc \
			    boost_odeint_integrator.cc \
			    dopri5_integrator.cc \
			    dop853_integrator.cc \
			    odex_integrator.cc \
			    radau5_integrator.cc \
			    radau_integrator.cc \
			    rodas_integrator.cc \
			    seulex_integrator.cc \
			    daspk_integrator.cc \
			    daskr_integrator.cc \
			    quasi_static_integrator.cc \
			    time_stepping_integrator.cc \
			    theta_time_stepping_integrator.cc \
			    time_stepping_ssc_integrator.cc \
			    hets2_integrator.cc \
			    explicit_euler_integrator.cc \
			    implicit_euler_integrator.cc \
			    fortran/opkda1.f\
			    fortran/opkda2.f\
			    fortran/opkdmain.f\
			    fortran/dopri5.f\
			    fortran/dop853.f\
			    fortran/odex.f\
			    fortran/radau5.f\
			    fortran/radau.f\
			    fortran/rodas.f\
			    fortran/seulex.f\
			    fortran/dc_lapack.f\
			    fortran/rksuite.f\
			    fortran/ddaspk.f\
			    fortran/ddaskr.f
#auto_time_stepping_ssc_integrator.cc
#fortran/mexax.f
#mexax_integrator.cc 

libintegrators_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
libintegrators_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

integratorsincludedir = $(includedir)/mbsim/integrators

integratorsinclude_HEADERS = integrator.h \
			     integrators.h \
			     lsode_integrator.h \
			     lsoda_integrator.h \
			     lsodar_integrator.h \
			     lsodkr_integrator.h \
			     lsodi_integrator.h \
			     rksuite_integrator.h \
			     boost_odeint_integrator.h \
			     boost_odeint_integrator_predef.h \
			     dopri5_integrator.h \
			     dop853_integrator.h \
			     odex_integrator.h \
			     radau5_integrator.h \
			     radau_integrator.h \
			     rodas_integrator.h \
			     seulex_integrator.h \
			     daspk_integrator.h \
			     daskr_integrator.h \
			     quasi_static_integrator.h \
			     time_stepping_integrator.h \
			     theta_time_stepping_integrator.h \
			     time_stepping_ssc_integrator.h \
			     hets2_integrator.h \
			     explicit_euler_integrator.h \
			     implicit_euler_integrator.h \
			     fortran/fortran_wrapper.h
#auto_time_stepping_ssc_integrator.h
# mexax_integrator.h 


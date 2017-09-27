noinst_LTLIBRARIES = libanalysers.la
libanalysers_la_SOURCES = eigenanalyser.cc\
			  harmonic_response_analyser.cc\
			  fortran/dfftf.f\
			  fortran/dffti.f\
			  fortran/radf2.f\
			  fortran/radf3.f\
			  fortran/radf4.f\
			  fortran/radf5.f\
			  fortran/radfg.f\
			  fortran/rfftf1.f\
			  fortran/rffti1.f
			     
libanalysers_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
libanalysers_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

analysersincludedir = $(includedir)/mbsim/analysers

analysersinclude_HEADERS = eigenanalyser.h\
			   harmonic_response_analyser.h

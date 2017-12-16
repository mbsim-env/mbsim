noinst_LTLIBRARIES = libanalyzers.la
libanalyzers_la_SOURCES = eigenanalyzer.cc\
			  harmonic_response_analyzer.cc
			     
libanalyzers_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
libanalyzers_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

analyzersincludedir = $(includedir)/mbsim/analyzers

analyzersinclude_HEADERS = eigenanalyzer.h\
			   harmonic_response_analyzer.h

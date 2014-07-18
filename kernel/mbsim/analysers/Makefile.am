noinst_LTLIBRARIES = libanalysers.la
libanalysers_la_SOURCES = eigenanalyser.cc
			     
libanalysers_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
libanalysers_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

analysersincludedir = $(includedir)/mbsim/analysers

analysersinclude_HEADERS = eigenanalyser.h


noinst_LTLIBRARIES = libanalysis.la
libanalysis_la_SOURCES = eigenanalysis.cc
			     
libanalysis_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
libanalysis_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

analysisincludedir = $(includedir)/mbsim/analysis

analysisinclude_HEADERS = eigenanalysis.h


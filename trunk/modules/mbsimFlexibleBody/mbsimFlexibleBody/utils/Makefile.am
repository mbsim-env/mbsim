noinst_LTLIBRARIES = libutils.la
libutils_la_SOURCES = contact_utils.cc

utilsincludedir = $(includedir)/mbsimFlexibleBody/utils

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)
libutils_la_CPPFLAGS = -DTIXML_USE_STL

utilsinclude_HEADERS = contact_utils.h

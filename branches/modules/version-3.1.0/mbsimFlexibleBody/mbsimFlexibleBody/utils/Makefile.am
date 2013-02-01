noinst_LTLIBRARIES = libutils.la
libutils_la_SOURCES = contact_utils.cc\
			angles.cc\
			cardan.cc\
			#revcardan.cc

utilsincludedir = $(includedir)/mbsimFlexibleBody/utils

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)
libutils_la_CPPFLAGS = -DTIXML_USE_STL

utilsinclude_HEADERS = contact_utils.h\
			angles.h\
			cardan.h\
			#revcardan.h

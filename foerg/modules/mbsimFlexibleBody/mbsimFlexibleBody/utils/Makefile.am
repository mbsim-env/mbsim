noinst_LTLIBRARIES = libutils.la
libutils_la_SOURCES = contact_utils.cc\
			angles.cc\
			cardan.cc\
			revcardan.cc

utilsincludedir = $(includedir)/mbsimFlexibleBody/utils

libutils_la_CPPFLAGS = -I$(top_srcdir) -DTIXML_USE_STL $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)
libutils_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)

utilsinclude_HEADERS = contact_utils.h\
			angles.h\
			cardan.h\
			revcardan.h

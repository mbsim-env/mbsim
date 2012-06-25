SUBDIRS = .

noinst_LTLIBRARIES = libcontour.la
libcontour_la_SOURCES = circle.cc

libcontour_la_CPPFLAGS = -I.. -DTIXML_USE_STL $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
libcontour_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

contourincludedir = $(includedir)/mbsim/contours
contourinclude_HEADERS = line.h\
			 point.h\
			 circle.h

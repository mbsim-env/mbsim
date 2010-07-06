SUBDIRS = .

noinst_LTLIBRARIES = libcontour.la
libcontour_la_SOURCES = nurbs_disk_2s.cc

libcontour_la_CPPFLAGS = -I.. -DTIXML_USE_STL $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
libcontour_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

contourincludedir = $(includedir)/mbsimFlexibleBody/contours
contourinclude_HEADERS = nurbs_disk_2s.h

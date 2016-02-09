SUBDIRS = neutral_contour .

noinst_LTLIBRARIES = libcontour.la
libcontour_la_SOURCES =

libcontour_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)
libcontour_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)
libcontour_la_LIBADD += neutral_contour/libneutralcontour.la

contourincludedir = $(includedir)/mbsimFlexibleBody/contours
contourinclude_HEADERS =

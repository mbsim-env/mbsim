SUBDIRS = neutral_contour .

noinst_LTLIBRARIES = libcontour.la
libcontour_la_SOURCES = contour1s.cc\
			contour1s_flexible.cc\
			flexible_band.cc\
			contour_1s_neutral_factory.cc\
			contour_2s_neutral_factory.cc\
			nurbs_disk_2s.cc\
			flexible_planar_nurbs_contour.cc\
			flexible_spatial_nurbs_contour.cc

libcontour_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)
libcontour_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)
libcontour_la_LIBADD += neutral_contour/libneutralcontour.la

contourincludedir = $(includedir)/mbsimFlexibleBody/contours
contourinclude_HEADERS = contour_continuum.h\
			 contour1s.h\
			 contour1s_flexible.h\
			 flexible_band.h\
			 contour_1s_neutral_factory.h\
			 contour2s.h\
			 contour_2s_neutral_factory.h\
			 nurbs_disk_2s.h\
			 flexible_planar_nurbs_contour.h\
			 flexible_spatial_nurbs_contour.h

SUBDIRS = neutral_contour .

noinst_LTLIBRARIES = libcontour.la
libcontour_la_SOURCES = flexible_band.cc\
												nurbs_disk_2s.cc\
												nurbs_curve_1s.cc																								
#												\
#												nurbs_curve_1s_same_angular_velocites.cc

libcontour_la_CPPFLAGS = -I$(top_srcdir) -DTIXML_USE_STL $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)
libcontour_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)
libcontour_la_LIBADD += neutral_contour/libneutralcontour.la

contourincludedir = $(includedir)/mbsimFlexibleBody/contours
contourinclude_HEADERS = contour1s_flexible.h\
												 contour2s_flexible.h\
												 cylinder_flexible.h\
												 flexible_band.h\												 
												 nurbs_disk_2s.h\
												 nurbs_curve_1s.h
#												 \
#												 nurbs_curve_1s_same_angular_velocites.h


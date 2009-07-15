SUBDIRS = .
  
noinst_LTLIBRARIES = libcontour.la
libcontour_la_SOURCES = circle.cc\
						contour1s_analytical.cc\
						flexible_band.cc\
						sphere.cc

libcontour_la_CPPFLAGS = -I.. -DTIXML_USE_STL $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
libcontour_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

contourincludedir = $(includedir)/mbsim/contours
contourinclude_HEADERS = circle.h\
						 contour_continuum.h\
						 contour1s.h\
						 contour1s_analytical.h\
						 contour1s_flexible.h\
						 contour2s.h\
						 contour2s_flexible.h\
						 cylinder_flexible.h\
						 flexible_band.h\
						 sphere.h


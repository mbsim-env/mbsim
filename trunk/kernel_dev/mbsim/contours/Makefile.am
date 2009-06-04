SUBDIRS = .
  
noinst_LTLIBRARIES = libcontour.la
libcontour_la_SOURCES = flexible_band.cc\
						contour1s_analytical.cc

libcontour_la_CPPFLAGS = -I.. $(DEPS_CFLAGS)
libcontour_la_LIBADD = $(DEPS_LIBS)

contourincludedir = $(includedir)/mbsim/contours
contourinclude_HEADERS = flexible_band.h\
						 contour_continuum.h\
						 contour1s.h\
						 contour2s.h\
						 contour1s_analytical.h\
						 contour1s_flexible.h\
						 contour2s_flexible.h\
						 cylinder_flexible.h


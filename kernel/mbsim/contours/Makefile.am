SUBDIRS = .

noinst_LTLIBRARIES = libcontour.la
libcontour_la_SOURCES = circle.cc\
			line.cc\
			point.cc\
			compound_contour.cc\
			plane.cc\
			sphere.cc

libcontour_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) 
libcontour_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

contourincludedir = $(includedir)/mbsim/contours
contourinclude_HEADERS = circle.h\
			 line.h\
			 point.h\
			 compound_contour.h\
			 plane.h\
			 sphere.h

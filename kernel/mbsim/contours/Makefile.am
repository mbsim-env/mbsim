SUBDIRS = .

noinst_LTLIBRARIES = libcontour.la
libcontour_la_SOURCES = line.cc\
			point.cc\
			compound_contour.cc

libcontour_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) 
libcontour_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

contourincludedir = $(includedir)/mbsim/contours
contourinclude_HEADERS = line.h\
			 point.h\
			 compound_contour.h

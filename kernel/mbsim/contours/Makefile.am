SUBDIRS = .

noinst_LTLIBRARIES = libcontour.la
libcontour_la_SOURCES = circle.cc\
			rectangle.cc\
			line.cc\
			line_segment.cc\
			point.cc\
			compound_contour.cc\
			cuboid.cc\
			edge.cc\
			plane.cc\
			sphere.cc\
			frustum.cc\
			frustum2d.cc\
			room.cc\
			planewithfrustum.cc\
			polynomial_frustum.cc\
			contour1s.cc\
			contour1s_analytical.cc

libcontour_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) 
libcontour_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

contourincludedir = $(includedir)/mbsim/contours
contourinclude_HEADERS = circle.h\
			 circle_hollow.h\
			 circle_solid.h\
			 rectangle.h\
			 line.h\
			 line_segment.h\
			 point.h\
			 compound_contour.h\
			 cuboid.h\
			 edge.h\
			 plane.h\
			 sphere.h\
			 frustum.h\
			 frustum2d.h\
			 room.h\
			 planewithfrustum.h\
			 polynomial_frustum.h\
			 contour_continuum.h\
			 contour1s.h\
			 contour1s_analytical.h

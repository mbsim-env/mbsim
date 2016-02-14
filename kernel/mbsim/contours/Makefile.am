SUBDIRS = .

noinst_LTLIBRARIES = libcontour.la
libcontour_la_SOURCES = rigid_contour.cc\
			circle.cc\
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
			planar_contour.cc\
			contour_interpolation.cc\
			contour_quad.cc

libcontour_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) 
libcontour_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

contourincludedir = $(includedir)/mbsim/contours
contourinclude_HEADERS = rigid_contour.h\
			 circle.h\
			 hollow_circle.h\
			 solid_circle.h\
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
			 planar_contour.h\
			 contour2s.h\
			 contour_interpolation.h\
			 contour_quad.h

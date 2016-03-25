SUBDIRS = .

noinst_LTLIBRARIES = libcontour.la
libcontour_la_SOURCES = contour.cc\
			rigid_contour.cc\
			circle.cc\
			plate.cc\
			line.cc\
			line_segment.cc\
			point.cc\
			compound_contour.cc\
			cuboid.cc\
			edge.cc\
			plane.cc\
			sphere.cc\
			frustum.cc\
			planar_frustum.cc\
			room.cc\
			planewithfrustum.cc\
			polynomial_frustum.cc\
			planar_contour.cc\
			contour_interpolation.cc\
			contour_quad.cc

libcontour_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) 
libcontour_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

contourincludedir = $(includedir)/mbsim/contours
contourinclude_HEADERS = contour.h\
			 rigid_contour.h\
			 circle.h\
			 plate.h\
			 line.h\
			 line_segment.h\
			 point.h\
			 compound_contour.h\
			 cuboid.h\
			 edge.h\
			 plane.h\
			 sphere.h\
			 frustum.h\
			 planar_frustum.h\
			 room.h\
			 planewithfrustum.h\
			 polynomial_frustum.h\
			 planar_contour.h\
			 contour_interpolation.h\
			 contour_quad.h

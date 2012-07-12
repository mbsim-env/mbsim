SUBDIRS = .

noinst_LTLIBRARIES = libcontour.la
libcontour_la_SOURCES = face.cc\
			circle.cc\
			area.cc\
			compound_contour.cc\
			contour1s_analytical.cc\
			contour_interpolation.cc\
			contour_quad.cc\
			cuboid.cc\
			frustum.cc\
			frustum2d.cc\
			plane.cc\
			planewithfrustum.cc\
			sphere.cc\
			line_segment.cc

libcontour_la_CPPFLAGS = -I.. -DTIXML_USE_STL $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) 
libcontour_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

contourincludedir = $(includedir)/mbsim/contours
contourinclude_HEADERS = face.h\
			 circle.h\
			 area.h\
			 circle_hollow.h\
			 circle_solid.h\
			 compound_contour.h\
			 contour_continuum.h\
			 contour_interpolation.h\
			 contour_quad.h\
			 contour1s.h\
			 contour1s_analytical.h\
			 contour2s.h\
			 cuboid.h\
			 edge.h\
			 frustum.h\
			 frustum2d.h\
			 line.h\
			 plane.h\
			 planewithfrustum.h\
			 point.h\
			 sphere.h\
			 line_segment.h

SUBDIRS = .

noinst_LTLIBRARIES = libcontour.la
libcontour_la_SOURCES = circle.cc\
						circle_solid.cc\
						circle_hollow.cc\
						contour1s_analytical.cc\
						flexible_band.cc\
						sphere.cc\
						area.cc\
						frustum.cc\
						frustum2d.cc\
						compound_contour.cc\
						contour_interpolation.cc\
						contour_quad.cc\
						cuboid.cc
						

  libcontour_la_CPPFLAGS = -I.. -DTIXML_USE_STL $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
libcontour_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

  contourincludedir = $(includedir)/mbsim/contours
  contourinclude_HEADERS = circle.h\
						   circle_hollow.h\
						   circle_solid.h\
						   contour_continuum.h\
						   contour1s.h\
						   contour1s_analytical.h\
						   contour1s_flexible.h\
						   contour2s.h\
						   contour2s_flexible.h\
						   cylinder_flexible.h\
						   flexible_band.h\
						   point.h\
						   line.h\
						   sphere.h\
						   area.h\
						   frustum.h\
						   frustum2d.h\
						   plane.h\
						   edge.h\
						   planewithfrustum.h\
						   compound_contour.h\
						   contour_interpolation.h\
						   contour_quad.h\
						   cuboid.h

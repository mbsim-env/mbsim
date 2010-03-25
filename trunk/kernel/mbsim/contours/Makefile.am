SUBDIRS = .

noinst_LTLIBRARIES = libcontour.la
libcontour_la_SOURCES = area.cc\
						circle.cc\
						compound_contour.cc\
						contour1s_analytical.cc\
						contour_interpolation.cc\
						contour_quad.cc\
						cuboid.cc\
						flexible_band.cc\
						frustum.cc\
						frustum2d.cc\
						nurbs_disk_2s.cc\
						nurbs_disk_2s_mfr_mindlin.cc\
						plane.cc\
						planewithfrustum.cc\
						sphere.cc

  libcontour_la_CPPFLAGS = -I.. -DTIXML_USE_STL $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
libcontour_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

  contourincludedir = $(includedir)/mbsim/contours
  contourinclude_HEADERS = area.h\
						   circle.h\
						   circle_hollow.h\
						   circle_solid.h\
						   compound_contour.h\
						   contour_continuum.h\
						   contour_interpolation.h\
						   contour_quad.h\
						   contour1s.h\
						   contour1s_analytical.h\
						   contour1s_flexible.h\
						   contour2s.h\
						   contour2s_flexible.h\
						   cuboid.h\
						   cylinder_flexible.h\
						   edge.h\
						   flexible_band.h\
						   frustum.h\
						   frustum2d.h\
						   line.h\
						   nurbs_disk_2s.h\
						   nurbs_disk_2s_mfr_mindlin.h\
						   plane.h\
						   planewithfrustum.h\
						   point.h\
						   sphere.h

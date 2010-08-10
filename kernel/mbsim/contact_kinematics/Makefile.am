noinst_LTLIBRARIES = libcontactkinematics.la
libcontactkinematics_la_SOURCES = circle_frustum.cc\
					circlehollow_cylinderflexible.cc \
					circlesolid_circlehollow.cc\
					circlesolid_circlesolid.cc\
					circlesolid_contour1s.cc\
					circlesolid_flexibleband.cc\
					circlesolid_frustum2d.cc\
					circlesolid_line.cc\
					circlesolid_linesegment.cc\
					circlesolid_plane.cc\
					compoundcontour_compoundcontour.cc\
					compoundcontour_contour.cc\
					contact_kinematics.cc\
					edge_edge.cc\
					line_contour1s.cc\
					point_area.cc\
					point_contour1s.cc\
					point_contourinterpolation.cc\
  					point_cylinderflexible.cc \
					point_flexibleband.cc \
					point_frustum.cc\
					point_line.cc\
					point_circlesolid.cc\
					point_plane.cc\
					point_planewithfrustum.cc\
					sphere_frustum.cc\
					sphere_plane.cc\
					sphere_sphere.cc\
					point_line_segment.cc

contactkinematicsincludedir = $(includedir)/mbsim/contact_kinematics

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

contactkinematicsinclude_HEADERS = circle_frustum.h\
  circlehollow_cylinderflexible.h \
  circlesolid_circlehollow.h \
  circlesolid_circlesolid.h \
  circlesolid_contour1s.h \
  circlesolid_flexibleband.h\
  circlesolid_frustum2d.h \
  circlesolid_line.h \
  circlesolid_linesegment.h \
  circlesolid_plane.h \
  compoundcontour_compoundcontour.h \
  compoundcontour_contour.h \
  contact_kinematics.h \
  edge_edge.h \
  line_contour1s.h \
  point_area.h \
  point_contour1s.h \
  point_contourinterpolation.h \
  point_cylinderflexible.h \
  point_flexibleband.h \
  point_frustum.h \
  point_line.h \
  point_circlesolid.h\
  point_plane.h \
  point_planewithfrustum.h \
  sphere_frustum.h \
  sphere_plane.h \
  sphere_sphere.h \
  point_line_segment.cc

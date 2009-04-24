noinst_LTLIBRARIES = libcontactkinematics.la
libcontactkinematics_la_SOURCES = 	contact_kinematics.cc\
					point_line.cc\
					sphere_plane.cc\
					point_plane.cc\
					compoundcontour_contour.cc\
					compoundcontour_compoundcontour.cc\
					circlesolid_line.cc\
					point_contour1s.cc\
					point_contourinterpolation.cc\
					line_contour1s.cc\
					circlesolid_circlehollow.cc\
					circlesolid_circlesolid.cc\
					circlesolid_frustum2d.cc\
					circlesolid_plane.cc\
					circlesolid_contour1s.cc\
					circlehollow_cylinderflexible.cc \
  					point_cylinderflexible.cc \
					point_flexibleband.cc \
					edge_edge.cc\
					point_area.cc\
					point_frustum.cc\
					sphere_frustum.cc\
					sphere_sphere.cc 

contactkinematicsincludedir = $(includedir)/mbsim/contact_kinematics

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

contactkinematicsinclude_HEADERS = contact_kinematics.h \
  circlesolid_circlehollow.h \
  circlesolid_circlesolid.h \
  circlesolid_contour1s.h \
  circlesolid_frustum2d.h \
  circlesolid_line.h \
  circlesolid_plane.h \
  circlehollow_cylinderflexible.h \
  point_cylinderflexible.h \
  compoundcontour_compoundcontour.h \
  compoundcontour_contour.h \
  point_flexibleband.h \
  edge_edge.h \
  line_contour1s.h \
  point_area.h \
  point_contour1s.h \
  point_contourinterpolation.h \
  point_frustum.h \
  point_line.h \
  point_plane.h \
  sphere_frustum.h \
  sphere_plane.h \
  sphere_sphere.h

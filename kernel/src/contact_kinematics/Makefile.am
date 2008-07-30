noinst_LTLIBRARIES = libcontactkinematics.la
libcontactkinematics_la_SOURCES = 	contact_kinematics.cc\
					point_line.cc\
					point_cylinderflexible.cc\
					point_contour1s.cc\
					point_circlesolid.cc\
					point_contourinterpolation.cc\
					line_contour1s.cc\
					circlesolid_circlehollow.cc\
					circlesolid_circlesolid.cc\
					circlesolid_frustum2d.cc\
					circlesolid_line.cc\
					circlesolid_plane.cc\
					circlesolid_contour1s.cc\
					circlehollow_cylinderflexible.cc\
					edge_edge.cc\
					point_area.cc\
					point_frustum.cc\
					point_plane.cc\
					sphere_frustum.cc\
					sphere_plane.cc\
					sphere_sphere.cc 

libcontactkinematics_la_CPPFLAGS = -I.. 
contactkinematicsincludedir = $(includedir)/mbsim

INCLUDES = $(DEPS_CFLAGS) 
LDADD = $(DEPS_LIBS) 

contactkinematicsinclude_HEADERS = 	contact_kinematics.h\
					point_contour1s.h

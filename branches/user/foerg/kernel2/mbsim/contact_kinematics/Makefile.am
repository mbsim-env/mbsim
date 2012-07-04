noinst_LTLIBRARIES = libcontactkinematics.la
libcontactkinematics_la_SOURCES = contact_kinematics.cc\
				  point_line.cc\
				  circlesolid_circlehollow.cc\
				  circlesolid_circlesolid.cc\
				  circlesolid_line.cc\
				  point_plane.cc\
				  compoundcontour_contour.cc\
				  compoundcontour_compoundcontour.cc\
				  sphere_frustum.cc\
				  sphere_plane.cc\
				  sphere_sphere.cc\
				  point_circlesolid.cc\
				  point_planewithfrustum.cc

contactkinematicsincludedir = $(includedir)/mbsim/contact_kinematics

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

contactkinematicsinclude_HEADERS = contact_kinematics.h \
				   point_line.h\
				   circlesolid_circlehollow.h\
				   circlesolid_circlesolid.h\
				   circlesolid_line.h\
				   point_plane.h\
				   compoundcontour_contour.h\
				   compoundcontour_compoundcontour.h\
				   sphere_frustum.h\
				   sphere_plane.h\
				   sphere_sphere.h\
				   point_circlesolid.h\
				   point_planewithfrustum.h

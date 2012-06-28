noinst_LTLIBRARIES = libcontactkinematics.la
libcontactkinematics_la_SOURCES = contact_kinematics.cc\
				  point_line.cc\
				  circlesolid_circlehollow.cc\
				  circlesolid_circlesolid.cc\
				  circlesolid_line.cc\
				  point_plane.cc\
				  compoundcontour_contour.cc

contactkinematicsincludedir = $(includedir)/mbsim/contact_kinematics

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

contactkinematicsinclude_HEADERS = contact_kinematics.h \
				   point_line.h\
				   circlesolid_circlehollow.h\
				   circlesolid_circlesolid.h\
				   circlesolid_line.h\
				   point_plane.h\
				   compoundcontour_contour.h

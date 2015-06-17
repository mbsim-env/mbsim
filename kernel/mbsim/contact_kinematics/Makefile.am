noinst_LTLIBRARIES = libcontactkinematics.la
libcontactkinematics_la_SOURCES = contact_kinematics.cc\
				  point_line.cc\
				  circlesolid_line.cc\
				  point_plane.cc\
				  sphere_plane.cc\
				  sphere_sphere.cc\
				  compoundcontour_contour.cc

contactkinematicsincludedir = $(includedir)/mbsim/contact_kinematics

libcontactkinematics_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
libcontactkinematics_la_LIBADD =  $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

contactkinematicsinclude_HEADERS = contact_kinematics.h\
				   point_line.h\
				   circlesolid_line.h\
				   point_plane.h\
				   sphere_plane.h\
				   sphere_sphere.h\
				   compoundcontour_contour.h

noinst_LTLIBRARIES = libcontactkinematics.la
libcontactkinematics_la_SOURCES = contact_kinematics.cc\
				  point_line.cc\
				  point_line_segment.cc\
				  point_plane.cc\
				  point_rectangle.cc\
				  point_frustum.cc\
				  circlesolid_line.cc\
				  circlesolid_circlehollow.cc\
				  sphere_plane.cc\
				  sphere_sphere.cc\
				  edge_edge.cc\
				  compoundcontour_contour.cc\
				  compoundcontour_compoundcontour.cc\
				  circlesolid_contour1s.cc

contactkinematicsincludedir = $(includedir)/mbsim/contact_kinematics

libcontactkinematics_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
libcontactkinematics_la_LIBADD =  $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

contactkinematicsinclude_HEADERS = contact_kinematics.h\
				   point_line.h\
				   point_line_segment.h\
				   point_plane.h\
				   point_rectangle.h\
				   point_frustum.h\
				   circlesolid_line.h\
				   circlesolid_circlehollow.h\
				   sphere_plane.h\
				   sphere_sphere.h\
				   edge_edge.h\
				   compoundcontour_contour.h\
				   compoundcontour_compoundcontour.h\
				   circlesolid_contour1s.h

noinst_LTLIBRARIES = libcontactkinematics.la
libcontactkinematics_la_SOURCES = contact_kinematics.cc\
				  point_line.cc\
				  point_line_segment.cc\
				  point_plane.cc\
				  point_sphere.cc\
				  point_rectangle.cc\
				  point_circlesolid.cc\
				  point_frustum.cc\
				  point_planewithfrustum.cc\
				  point_polynomialfrustum.cc\
				  point_contour1s.cc\
				  point_contourinterpolation.cc\
				  circlesolid_line.cc\
				  circle_circle.cc\
				  circlesolid_plane.cc\
				  circlesolid_linesegment.cc\
				  circlesolid_frustum2d.cc\
				  circle_planarcontour.cc\
				  circle_frustum.cc\
				  line_contour1s.cc\
				  sphere_plane.cc\
				  sphere_sphere.cc\
				  sphere_frustum.cc\
				  sphere_rectangle.cc\
				  sphere_polynomialfrustum.cc\
				  edge_edge.cc\
				  rectangle_polynomialfrustum.cc\
				  compoundcontour_contour.cc\
				  compoundcontour_compoundcontour.cc

contactkinematicsincludedir = $(includedir)/mbsim/contact_kinematics

libcontactkinematics_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
libcontactkinematics_la_LIBADD =  $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

contactkinematicsinclude_HEADERS = contact_kinematics.h\
				   point_line.h\
				   point_line_segment.h\
				   point_plane.h\
				   point_sphere.h\
				   point_rectangle.h\
				   point_circlesolid.h\
				   point_frustum.h\
				   point_planewithfrustum.h\
				   point_polynomialfrustum.h\
				   point_contour1s.h\
				   point_contourinterpolation.h\
				   circlesolid_line.h\
				   circle_circle.h\
				   circlesolid_plane.h\
				   circlesolid_linesegment.h\
				   circlesolid_frustum2d.h\
				   circle_frustum.h\
				   circle_planarcontour.h\
				   line_contour1s.h\
				   sphere_plane.h\
				   sphere_sphere.h\
				   sphere_frustum.h\
				   sphere_rectangle.h\
				   sphere_polynomialfrustum.h\
				   edge_edge.h\
				   rectangle_polynomialfrustum.h\
				   compoundcontour_contour.h\
				   compoundcontour_compoundcontour.h

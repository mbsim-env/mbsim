noinst_LTLIBRARIES = libcontactkinematics.la
libcontactkinematics_la_SOURCES = contact_kinematics.cc\
				  point_line.cc\
				  point_linesegment.cc\
				  point_plane.cc\
				  point_sphere.cc\
				  point_plate.cc\
				  point_circle.cc\
				  point_cylinder.cc\
				  point_frustum.cc\
				  point_planewithfrustum.cc\
				  point_polynomialfrustum.cc\
				  point_planarcontour.cc\
				  point_extrusion.cc\
				  point_contourinterpolation.cc\
				  point_spatialcontour.cc\
				  circle_line.cc\
				  circle_circle.cc\
				  circle_plane.cc\
				  circle_spatialcontour.cc\
				  circle_linesegment.cc\
				  circle_planarfrustum.cc\
				  circle_planarcontour.cc\
				  circle_extrusion.cc\
				  circle_frustum.cc\
				  line_planarcontour.cc\
				  planarcontour_planarcontour.cc\
				  sphere_plane.cc\
				  sphere_sphere.cc\
				  sphere_frustum.cc\
				  sphere_plate.cc\
				  sphere_polynomialfrustum.cc\
				  sphere_spatialcontour.cc\
				  plane_spatialcontour.cc\
				  spatialcontour_spatialcontour.cc\
				  edge_edge.cc\
				  plate_polynomialfrustum.cc\
				  cylindricalgear_cylindricalgear.cc\
				  cylindricalgear_rack.cc\
                                  bevelgear_planargear.cc\
                                  bevelgear_bevelgear.cc\
				  compoundcontour_contour.cc\
				  compoundcontour_compoundcontour.cc

contactkinematicsincludedir = $(includedir)/mbsim/contact_kinematics

libcontactkinematics_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
libcontactkinematics_la_LIBADD =  $(LDFLAGS_NOUNDEF) $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

contactkinematicsinclude_HEADERS = contact_kinematics.h\
				   point_line.h\
				   point_linesegment.h\
				   point_plane.h\
				   point_sphere.h\
				   point_plate.h\
				   point_circle.h\
				   point_cylinder.h\
				   point_frustum.h\
				   point_planewithfrustum.h\
				   point_polynomialfrustum.h\
				   point_planarcontour.h\
				   point_extrusion.h\
				   point_contourinterpolation.h\
				   point_spatialcontour.h\
				   circle_line.h\
				   circle_circle.h\
				   circle_plane.h\
				   circle_spatialcontour.h\
				   circle_linesegment.h\
				   circle_planarfrustum.h\
				   circle_frustum.h\
				   circle_planarcontour.h\
				   circle_extrusion.h\
				   line_planarcontour.h\
				   planarcontour_planarcontour.h\
				   sphere_plane.h\
				   sphere_sphere.h\
				   sphere_frustum.h\
				   sphere_plate.h\
				   sphere_polynomialfrustum.h\
				   sphere_spatialcontour.h\
				   plane_spatialcontour.h\
				   spatialcontour_spatialcontour.h\
				   edge_edge.h\
				   plate_polynomialfrustum.h\
                                   cylindricalgear_cylindricalgear.h\
                                   cylindricalgear_rack.h\
                                   bevelgear_planargear.h\
                                   bevelgear_bevelgear.h\
				   compoundcontour_contour.h\
				   compoundcontour_compoundcontour.h

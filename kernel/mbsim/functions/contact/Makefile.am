noinst_LTLIBRARIES = libcontact.la
libcontact_la_SOURCES = funcpair_planarcontour_point.cc\
			funcpair_planarcontour_circle.cc\
			funcpair_planarcontour_line.cc\
			funcpair_planarcontour_planarcontour.cc\
			funcpair_spatialcontour_point.cc\
			funcpair_spatialcontour_plane.cc\
			funcpair_spatialcontour_circle.cc\
			funcpair_spatialcontour_sphere.cc\
			funcpair_spatialcontour_spatialcontour.cc\
			funcpair_point_contourinterpolation.cc\
			funcpair_conesection_circle.cc\
			funcpair_ellipse_circle.cc\
			funcpair_hyperbola_circle.cc\
			jacpair_conesection_circle.cc\
			jacpair_ellipse_circle.cc\
			jacpair_hyperbola_circle.cc\
			polar_contour_function.cc

contactincludedir = $(includedir)/mbsim/functions/contact

libcontact_la_LIBADD = $(LDFLAGS_NOUNDEF) $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)
libcontact_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)

contactinclude_HEADERS = funcpair_planarcontour_point.h\
			 funcpair_planarcontour_circle.h\
			 funcpair_planarcontour_line.h\
			 funcpair_planarcontour_planarcontour.h\
			 funcpair_spatialcontour_point.h\
			 funcpair_spatialcontour_plane.h\
			 funcpair_spatialcontour_circle.h\
			 funcpair_spatialcontour_sphere.h\
			 funcpair_spatialcontour_spatialcontour.h\
			 funcpair_point_contourinterpolation.h\
			 funcpair_conesection_circle.h\
			 funcpair_ellipse_circle.h\
			 funcpair_hyperbola_circle.h\
			 jacpair_conesection_circle.h\
			 jacpair_ellipse_circle.h\
			 jacpair_hyperbola_circle.h\
			 polar_contour_function.h

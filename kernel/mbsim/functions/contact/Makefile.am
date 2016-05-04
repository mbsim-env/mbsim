noinst_LTLIBRARIES = libcontact.la
libcontact_la_SOURCES = funcpair_planarcontour_point.cc\
			funcpair_planarcontour_circle.cc\
			funcpair_planarcontour_line.cc\
			funcpair_spatialcontour_point.cc\
			funcpair_point_contourinterpolation.cc\
			funcpair_conesection_circle.cc\
			funcpair_ellipse_circle.cc\
			funcpair_hyperbola_circle.cc\
			jacpair_conesection_circle.cc\
			jacpair_ellipse_circle.cc\
			jacpair_hyperbola_circle.cc\
			polar_contour_function.cc

contactincludedir = $(includedir)/mbsim/functions/contact

libcontact_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(CASADI_LIBS)
libcontact_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(CASADI_CFLAGS)

contactinclude_HEADERS = distance_function.h\
			 funcpair_planarcontour_point.h\
			 funcpair_planarcontour_circle.h\
			 funcpair_planarcontour_line.h\
			 funcpair_spatialcontour_point.h\
			 funcpair_point_contourinterpolation.h\
			 funcpair_conesection_circle.h\
			 funcpair_ellipse_circle.h\
			 funcpair_hyperbola_circle.h\
			 jacpair_conesection_circle.h\
			 jacpair_ellipse_circle.h\
			 jacpair_hyperbola_circle.h\
			 polar_contour_function.h

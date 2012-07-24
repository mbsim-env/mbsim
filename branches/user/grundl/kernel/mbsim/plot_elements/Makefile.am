noinst_LTLIBRARIES = libplotelements.la
libplotelements_la_SOURCES = plot_rigid_body_group.cc\
			     plot_frame.cc\
			     plot_cartesian_coordinates.cc\
			     plot_natural_coordinates.cc\
			     plot_cylinder_coordinates.cc

plotelementsincludedir = $(includedir)/mbsim/plot_elements

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)
libplotelements_la_CPPFLAGS = -DTIXML_USE_STL

plotelementsinclude_HEADERS = plot_rigid_body_group.h\
			      plot_frame.h\
			      plot_cartesian_coordinates.h\
			      plot_natural_coordinates.h\
			      plot_cylinder_coordinates.h

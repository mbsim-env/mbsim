noinst_LTLIBRARIES = libobserver.la
libobserver_la_SOURCES = kinematics_observer.cc \
			 cartesian_coordinates_observer.cc \
			 cylinder_coordinates_observer.cc \
			 natural_coordinates_observer.cc \
			 frame_observer.cc \
			 rigid_body_observer.cc \
			 rigid_body_group_observer.cc

observerincludedir = $(includedir)/mbsim/observers

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)
libobserver_la_CPPFLAGS = -DTIXML_USE_STL

observerinclude_HEADERS = kinematics_observer.h \
			  cartesian_coordinates_observer.h \
			  cylinder_coordinates_observer.h \
			  natural_coordinates_observer.h \
			  frame_observer.h \
			  rigid_body_observer.h \
			  rigid_body_group_observer.h

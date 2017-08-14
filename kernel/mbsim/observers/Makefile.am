noinst_LTLIBRARIES = libobserver.la
libobserver_la_SOURCES = observer.cc\
			 relative_kinematics_observer.cc \
			 kinematic_coordinates_observer.cc \
			 rigid_body_observer.cc \
			 rigid_body_system_observer.cc \
			 mechanical_link_observer.cc \
			 mechanical_constraint_observer.cc \
			 single_contact_observer.cc \
			 contact_observer.cc \
			 frame_observer.cc

observerincludedir = $(includedir)/mbsim/observers

libobserver_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)
libobserver_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)

observerinclude_HEADERS = observer.h\
			  relative_kinematics_observer.h \
			  kinematic_coordinates_observer.h \
			  rigid_body_observer.h \
			  rigid_body_system_observer.h \
			  mechanical_link_observer.h \
			  mechanical_constraint_observer.h \
			  single_contact_observer.h \
			  contact_observer.h \
			  frame_observer.h

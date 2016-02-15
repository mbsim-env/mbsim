SUBDIRS = .

noinst_LTLIBRARIES = liblink.la
liblink_la_SOURCES = contact.cc\
		     maxwell_contact.cc\
		     single_contact.cc\
		     joint.cc\
		     gear.cc\
		     spring_damper.cc\
		     directional_spring_damper.cc\
		     generalized_spring_damper.cc\
		     friction.cc\
		     extern_generalized_io.cc\
		     frame_link.cc\
		     floating_frame_link.cc\
		     contour_link.cc\
		     rigid_body_link.cc\
		     kinetic_excitation.cc\
		     kinematic_excitation.cc

liblink_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) 
liblink_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

linkincludedir = $(includedir)/mbsim/links
linkinclude_HEADERS = contact.h\
		      maxwell_contact.h\
		      single_contact.h\
		      joint.h\
		      gear.h\
		      spring_damper.h\
		      directional_spring_damper.h\
		      generalized_spring_damper.h\
		      friction.h\
		      extern_generalized_io.h\
		      frame_link.h\
		      floating_frame_link.h\
		      contour_link.h\
		      rigid_body_link.h\
		      kinetic_excitation.h\
		      kinematic_excitation.h

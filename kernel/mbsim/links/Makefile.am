SUBDIRS = .

noinst_LTLIBRARIES = liblink.la
liblink_la_SOURCES = link.cc\
		     test_element.cc\
		     mechanical_link.cc\
		     frame_link.cc\
		     fixed_frame_link.cc\
		     floating_frame_link.cc\
		     contour_link.cc\
		     rigid_body_link.cc\
		     dual_rigid_body_link.cc\
		     kinetic_excitation.cc\
		     contact.cc\
		     maxwell_contact.cc\
		     single_contact.cc\
		     joint.cc\
		     elastic_joint.cc\
		     spring_damper.cc\
		     directional_spring_damper.cc\
		     isotropic_rotational_spring_damper.cc\
		     generalized_spring_damper.cc\
		     generalized_friction.cc\
		     generalized_clutch.cc\
		     generalized_kinematic_excitation.cc\
		     generalized_position_excitation.cc\
		     generalized_velocity_excitation.cc\
		     generalized_acceleration_excitation.cc\
		     generalized_connection.cc\
		     generalized_elastic_connection.cc\
		     generalized_gear.cc \
		     generalized_elastic_structure.cc \
                     disk_contact.cc \
                     tyre_contact.cc \
		     initial_condition.cc \
		     generalized_initial_position.cc \
		     generalized_initial_velocity.cc

liblink_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) 
liblink_la_LIBADD = $(LDFLAGS_NOUNDEF) $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

linkincludedir = $(includedir)/mbsim/links
linkinclude_HEADERS = link.h\
		      test_element.h\
		      mechanical_link.h\
		      generalized_friction.h\
		      generalized_clutch.h\
		      frame_link.h\
		      fixed_frame_link.h\
		      floating_frame_link.h\
		      contour_link.h\
		      rigid_body_link.h\
		      dual_rigid_body_link.h\
		      kinetic_excitation.h\
		      contact.h\
		      maxwell_contact.h\
		      single_contact.h\
		      joint.h\
		      elastic_joint.h\
		      spring_damper.h\
		      directional_spring_damper.h\
		      isotropic_rotational_spring_damper.h\
		      generalized_spring_damper.h\
		      generalized_kinematic_excitation.h\
		      generalized_position_excitation.h\
		      generalized_velocity_excitation.h\
		      generalized_acceleration_excitation.h\
		      generalized_connection.h\
		      generalized_elastic_connection.h\
		      generalized_gear.h \
		      generalized_elastic_structure.h \
                      disk_contact.h \
		      tyre_contact.h \
		      initial_condition.h \
		      generalized_initial_position.h \
		      generalized_initial_velocity.h

SUBDIRS = .

noinst_LTLIBRARIES = libconstraint.la
libconstraint_la_SOURCES = constraint.cc\
			   generalized_constraint.cc\
			   generalized_dual_constraint.cc\
			   generalized_position_constraint.cc\
			   generalized_velocitiy_constraint.cc\
			   generalized_acceleration_constraint.cc\
			   joint_constraint.cc\
			   generalized_connection_constraint.cc\
			   generalized_gear_constraint.cc\
			   inverse_kinematics_constraint.cc

libconstraint_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) 
libconstraint_la_LIBADD = $(LDFLAGS_NOUNDEF) $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

constraintincludedir = $(includedir)/mbsim/constraints
constraintinclude_HEADERS = constraint.h\
			    mechanical_constraint.h\
			    generalized_constraint.h\
			    generalized_dual_constraint.h\
			    generalized_position_constraint.h\
			    generalized_velocitiy_constraint.h\
			    generalized_acceleration_constraint.h\
			    joint_constraint.h\
			    generalized_connection_constraint.h\
			    generalized_gear_constraint.h\
                            inverse_kinematics_constraint.h

SUBDIRS = .

noinst_LTLIBRARIES = libconstraint.la
libconstraint_la_SOURCES = constraint.cc\
			   gear_constraint.cc\
			   kinematic_constraint.cc\
			   generalized_position_constraint.cc\
			   generalized_velocitiy_constraint.cc\
			   generalized_acceleration_constraint.cc\
			   joint_constraint.cc

libconstraint_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) 
libconstraint_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

constraintincludedir = $(includedir)/mbsim/constraints
constraintinclude_HEADERS = constraint.h\
			    gear_constraint.h\
			    kinematic_constraint.h\
			    generalized_position_constraint.h\
			    generalized_velocitiy_constraint.h\
			    generalized_acceleration_constraint.h\
			    joint_constraint.h

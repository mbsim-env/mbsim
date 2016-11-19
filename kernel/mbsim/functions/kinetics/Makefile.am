noinst_LTLIBRARIES = libkinetics.la
libkinetics_la_SOURCES = linear_spring_damper_force.cc\
			 nonlinear_spring_damper_force.cc\
			 linear_regularized_unilateral_constraint.cc\
			 linear_regularized_bilateral_constraint.cc\
			 linear_regularized_coulomb_friction.cc\
			 linear_regularized_stribeck_friction.cc\
			 influence_function.cc\
			 flexibility_influence_function.cc\
			 constant_influence_function.cc\
			 linear_elastic_function.cc

kineticsincludedir = $(includedir)/mbsim/functions/kinetics

libkinetics_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(CASADI_LIBS)
libkinetics_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(CASADI_CFLAGS)

kineticsinclude_HEADERS = kinetics.h\
			  linear_spring_damper_force.h\
			  nonlinear_spring_damper_force.h\
			  linear_regularized_unilateral_constraint.h\
			  linear_regularized_bilateral_constraint.h\
			  linear_regularized_coulomb_friction.h\
			  linear_regularized_stribeck_friction.h\
			  influence_function.h\
			  flexibility_influence_function.h\
			  constant_influence_function.h\
			  linear_elastic_function.h

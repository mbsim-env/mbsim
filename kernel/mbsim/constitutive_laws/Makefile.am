SUBDIRS = .

noinst_LTLIBRARIES = libconstitutivelaws.la
libconstitutivelaws_la_SOURCES = bilateral_constraint.cc\
				 bilateral_impact.cc\
				 maxwell_unilateral_constraint.cc\
				 planar_coulomb_friction.cc\
				 planar_coulomb_impact.cc\
				 planar_stribeck_friction.cc\
				 planar_stribeck_impact.cc\
				 regularized_constraint.cc\
				 regularized_bilateral_constraint.cc\
				 regularized_friction.cc\
				 regularized_planar_friction.cc\
				 regularized_spatial_friction.cc\
				 regularized_unilateral_constraint.cc\
				 spatial_coulomb_friction.cc\
				 spatial_coulomb_impact.cc\
				 spatial_stribeck_friction.cc\
				 spatial_stribeck_impact.cc\
				 unilateral_constraint.cc\
				 unilateral_newton_impact.cc\
				 tyre_model.cc\
				 linear_tyre_model.cc\
				 magic_formula_sharp.cc\
				 magic_formula_62.cc

libconstitutivelaws_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) 
libconstitutivelaws_la_LIBADD = $(LDFLAGS_NOUNDEF) $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

constitutivelawsincludedir = $(includedir)/mbsim/constitutive_laws
constitutivelawsinclude_HEADERS = bilateral_constraint.h\
				  bilateral_impact.h\
				  friction_force_law.h\
				  friction_impact_law.h\
				  generalized_force_law.h\
				  generalized_impact_law.h\
				  maxwell_unilateral_constraint.h\
				  planar_coulomb_friction.h\
				  planar_coulomb_impact.h\
				  planar_stribeck_friction.h\
				  planar_stribeck_impact.h\
				  regularized_constraint.h\
				  regularized_bilateral_constraint.h\
				  regularized_friction.h\
				  regularized_planar_friction.h\
				  regularized_spatial_friction.h\
				  regularized_unilateral_constraint.h\
				  spatial_coulomb_friction.h\
				  spatial_coulomb_impact.h\
				  spatial_stribeck_friction.h\
				  spatial_stribeck_impact.h\
				  unilateral_constraint.h\
				  unilateral_newton_impact.h\
				  constitutive_laws.h\
				  tyre_model.h\
				  linear_tyre_model.h\
				  magic_formula_sharp.h\
				  magic_formula_62.h

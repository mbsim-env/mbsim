SUBDIRS = kinematics kinetics contact .

noinst_LTLIBRARIES = libfunction.la
libfunction_la_SOURCES = identity_function.cc\
			 constant_function.cc\
			 linear_function.cc\
			 multivariate_linear_function.cc\
			 multivariate_quadratic_function.cc\
			 quadratic_function.cc\
			 sinusoidal_function.cc\
			 polynom_function.cc\
			 absolute_value_function.cc\
			 modulo_function.cc\
			 signum_function.cc\
			 bounded_function.cc\
			 composite_function.cc\
			 vector_valued_function.cc\
			 piecewise_defined_function.cc\
			 tabular_function.cc\
			 two_dimensional_tabular_function.cc\
			 function.cc\
			 symbolic_function.cc\
			 piecewise_polynom_function.cc\
			 two_dimensional_piecewise_polynom_function.cc\
			 fourier_function.cc\
			 bidirectional_function.cc\
			 continued_function.cc

functionincludedir = $(includedir)/mbsim/functions

libfunction_la_LIBADD = $(LDFLAGS_NOUNDEF) kinematics/libkinematics.la
libfunction_la_LIBADD += kinetics/libkinetics.la
libfunction_la_LIBADD += contact/libcontact.la
libfunction_la_LIBADD += $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)
libfunction_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)

functioninclude_HEADERS = function.h\
			  identity_function.h\
			  constant_function.h\
			  linear_function.h\
			  multivariate_linear_function.h\
			  multivariate_quadratic_function.h\
			  quadratic_function.h\
			  sinusoidal_function.h\
			  polynom_function.h\
			  absolute_value_function.h\
			  modulo_function.h\
			  signum_function.h\
			  bounded_function.h\
			  composite_function.h\
			  vector_valued_function.h\
			  piecewise_defined_function.h\
			  tabular_function.h\
			  two_dimensional_tabular_function.h\
			  symbolic_function.h\
			  piecewise_polynom_function.h\
			  piecewise_polynom_function_impl.h\
			  two_dimensional_piecewise_polynom_function.h\
			  fourier_function.h\
			  bidirectional_function.h\
			  continued_function.h\
			  time_dependent_function.h\
			  state_dependent_function.h

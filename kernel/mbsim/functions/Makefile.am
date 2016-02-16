noinst_LTLIBRARIES = libfunction.la
libfunction_la_SOURCES = constant_function.cc\
			 linear_function.cc\
			 quadratic_function.cc\
			 sinusoidal_function.cc\
			 polynom_function.cc\
			 step_function.cc\
			 absolute_value_function.cc\
			 modulo_function.cc\
			 signum_function.cc\
			 bounded_function.cc\
			 nested_function.cc\
			 binary_nested_function.cc\
			 vector_valued_function.cc\
			 piecewise_defined_function.cc\
			 tabular_function.cc\
			 two_dimensional_tabular_function.cc\
			 symbolic_function.cc\
			 kinematic_functions.cc\
			 kinetic_functions.cc\
			 piecewise_polynom_function.cc\
			 fourier_function.cc\
			 contact_functions.cc

functionincludedir = $(includedir)/mbsim/functions

libfunction_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(CASADI_LIBS)
libfunction_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(CASADI_CFLAGS)

functioninclude_HEADERS = function.h\
			  constant_function.h\
			  linear_function.h\
			  quadratic_function.h\
			  sinusoidal_function.h\
			  polynom_function.h\
			  step_function.h\
			  absolute_value_function.h\
			  modulo_function.h\
			  signum_function.h\
			  bounded_function.h\
			  nested_function.h\
			  binary_nested_function.h\
			  vector_valued_function.h\
			  piecewise_defined_function.h\
			  tabular_function.h\
			  two_dimensional_tabular_function.h\
			  symbolic_function.h\
			  kinematic_functions.h\
			  kinetic_functions.h\
			  piecewise_polynom_function.h\
			  fourier_function.h\
			  time_dependent_function.h\
			  state_dependent_function.h\
			  contact_functions.h

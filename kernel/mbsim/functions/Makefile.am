noinst_LTLIBRARIES = libfunction.la
libfunction_la_SOURCES = basic_functions.cc \
			 nested_functions.cc \
			 vector_valued_functions.cc \
			 piecewise_defined_function.cc \
			 tabular_functions.cc \
			 symbolic_functions.cc \
			 kinematic_functions.cc \
			 kinetic_functions.cc \
			 piecewise_polynom_function.cc \
			 fourier_function.cc\
			 contact_functions.cc

functionincludedir = $(includedir)/mbsim/functions

libfunction_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(CASADI_LIBS)
libfunction_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(CASADI_CFLAGS)

functioninclude_HEADERS = function.h \
			  basic_functions.h \
			  nested_functions.h \
			  vector_valued_functions.h \
			  piecewise_defined_function.h \
			  tabular_functions.h \
			  symbolic_functions.h \
			  kinematic_functions.h \
			  kinetic_functions.h \
			  piecewise_polynom_function.h \
			  fourier_function.h\
			  time_dependent_function.h\
			  state_dependent_function.h\
			  contact_functions.h

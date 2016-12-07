dummydir = $(abs_builddir)
dummy_LTLIBRARIES = _mymod.la

CLEANFILES=mymod_swig_python.cc mymod.py mymod_swig_python.d
# create a empty dummy mymod_swig_python.cc file if MBSIM_SWIG is not set (to avoid a long build time)
mymod_swig_python.cc: $(srcdir)/mymod.i
	rm -f mymod_swig_python.cc
	test -z "$(MBSIM_SWIG)" && echo "" > $@ || $(swig) -python -MP -MD -Werror -c++ -o mymod_swig_python.cc $(DEPS_CFLAGS) -I$(srcdir)/.. $(srcdir)/mymod.i
-include mymod_swig_python.d
mymod.py: mymod_swig_python.cc
	test ! -f $@ && (rm -f mymod_swig_python.cc && $(MAKE) $(AM_MAKEFLAGS) mymod_swig_python.cc) || :

_mymod_la_SOURCES = mymod_swig_python.cc
_mymod_la_LDFLAGS = -module -shrext $(PYTHON_SHEXT) -avoid-version
_mymod_la_CPPFLAGS = $(PYTHON_CFLAGS) -I$(PYTHON_LIBDIR)/python$(PYTHON_VERSION)/site-packages/numpy/core/include $(DEPS_CFLAGS)
_mymod_la_LIBADD = $(PYTHON_LIBS) $(DEPS_LIBS)



TESTS = fmatvec_main.sh

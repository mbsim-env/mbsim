dummydir = $(abs_builddir)
dummy_LTLIBRARIES = _mymod.la

CLEANFILES=mymod_swig_python.cc mymod.py mymod_swig_python.d
mymod_swig_python.cc: $(srcdir)/mymod.i
	$(swig) -python -MP -MD -Werror -c++ -o mymod_swig_python.cc $(DEPS_CFLAGS) -I$(srcdir)/.. $(srcdir)/mymod.i || (rm -f mymod_swig_python.cc; exit 1)
-include mymod_swig_python.d
mymod.py: mymod_swig_python.cc
	test ! -f $@ && (rm -f mymod_swig_python.cc && $(MAKE) $(AM_MAKEFLAGS) mymod_swig_python.cc) || :

_mymod_la_SOURCES = mymod_swig_python.cc
_mymod_la_LDFLAGS = -module -shrext $(PYTHON_SHEXT) -avoid-version
_mymod_la_CPPFLAGS = $(PYTHON_CFLAGS) -I$(NUMPY_INCDIR) $(DEPS_CFLAGS)
_mymod_la_LIBADD = $(PYTHON_LIBS) $(DEPS_LIBS)



TESTS = fmatvec_main.sh

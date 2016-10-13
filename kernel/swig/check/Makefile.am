dummydir = $(abs_builddir)
dummy_LTLIBRARIES = _mymod.la

CLEANFILES=mymod_swig_pyhton.cc mymod.py mymod_swig_pyhton.d
mymod_swig_pyhton.cc: $(srcdir)/mymod.i
	$(swig) -python -MD -Werror -c++ -o mymod_swig_pyhton.cc $(DEPS_CFLAGS) -I$(srcdir)/.. $(srcdir)/mymod.i || (rm -f mymod_swig_pyhton.cc; exit 1)
-include mymod_swig_pyhton.d
mymod.py: mymod_swig_pyhton.cc
	test ! -f $@ && (rm -f mymod_swig_pyhton.cc && $(MAKE) $(AM_MAKEFLAGS) mymod_swig_pyhton.cc) || :

_mymod_la_SOURCES = mymod_swig_pyhton.cc
_mymod_la_LDFLAGS = -module -shrext $(PYTHON_SHEXT) -avoid-version
_mymod_la_CPPFLAGS = $(PYTHON_CFLAGS) -I$(NUMPY_INCDIR) $(DEPS_CFLAGS)
_mymod_la_LIBADD = $(PYTHON_LIBS) $(DEPS_LIBS)



TESTS = fmatvec_main.sh

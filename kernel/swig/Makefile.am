SUBDIRS = . check

# Note: wee install the resulting python file in $prefix/bin since this is the only place where Windows searches dependent libraries
swigdir = $(bindir)
swig_LTLIBRARIES = _mbsim.la

CLEANFILES=mbsim_swig_pyhton.cc mbsim.py mbsim_swig_pyhton.d
mbsim_swig_pyhton.cc: $(srcdir)/mbsim.i $(top_srcdir)/mbsim/objects/*.h
	$(swig) -python -MD -Werror -c++ -w473 -o mbsim_swig_pyhton.cc -I$(top_srcdir) $(DEPS_CFLAGS) $(srcdir)/mbsim.i || (rm -f mbsim_swig_pyhton.cc; exit 1) # mfmf remove -w473 if possible
-include mbsim_swig_pyhton.d
mbsim.py: mbsim_swig_pyhton.cc
	test ! -f $@ && (rm -f mbsim_swig_pyhton.cc && $(MAKE) $(AM_MAKEFLAGS) mbsim_swig_pyhton.cc) || :

_mbsim_la_SOURCES = mbsim_swig_pyhton.cc
_mbsim_la_LDFLAGS = -module -shrext $(PYTHON_SHEXT) -avoid-version
_mbsim_la_CPPFLAGS = $(PYTHON_CFLAGS) -I$(NUMPY_INCDIR) $(DEPS_CFLAGS) -I$(top_srcdir)
_mbsim_la_LIBADD = $(PYTHON_LIBS) ../mbsim/libmbsim.la

mbsimdir = $(datadir)/mbsim
mbsim_DATA = mbsim_modules.i

install-exec-hook: mbsim.py
	$(MKDIR_P) $(swigdir)
	$(INSTALL_DATA) mbsim.py $(swigdir)/mbsim.py
uninstall-local:
	rm -f $(swigdir)/mbsim.py

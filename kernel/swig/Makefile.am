SUBDIRS = . check

# Note: wee install the resulting python file in $prefix/bin since this is the only place where Windows searches dependent libraries
swigdir = $(bindir)
swig_LTLIBRARIES = _mbsim.la

# MISSING: swig has some bugs regarding directors and protected members,
# see https://github.com/swig/swig/issues/828
# As a workaround we define all protected members as public on swig processing and compilation.
# (-Dprotected=public)

CLEANFILES=mbsim_swig_pyhton.cc mbsim.py mbsim_swig_pyhton.d
mbsim_swig_pyhton.cc: $(srcdir)/mbsim.i $(top_srcdir)/mbsim/objects/*.h
	$(swig) -Dprotected=public -debug-classes -python -MD -Werror -builtin -c++ -o mbsim_swig_pyhton.cc -I$(top_srcdir) $(DEPS_CFLAGS) $(srcdir)/mbsim.i > swig.stdout || (rm -f mbsim_swig_pyhton.cc; exit 1)
	@echo BEGIN: THE FOLLOWING CLASSES ARE NOT WRAPPED BY SWIG
	@sed -nre "3,\$$s/(.*)/WRAPPED_CLASS(\1)/p" swig.stdout | sed -re "s/<\(/</g;s/\)>/>/g;s/<p\./</g" | grep -v ".*(.*(" > swig.wrapped_classes || (echo ERROR; exit 0)
	@$(swig) -Dprotected=public -DSHOW_UNWRAPPED_CLASSES=swig.wrapped_classes -python -MD -Werror -builtin -c++ -o dummy.cpp -I$(top_srcdir) $(DEPS_CFLAGS) $(srcdir)/mbsim.i 2>&1 | sed -nre "s/.*: Warning 901: //p" | sort | uniq || (echo ERROR; exit 0)
	@echo END: THE FOLLOWING CLASSES ARE NOT WRAPPED BY SWIG
-include mbsim_swig_pyhton.d
mbsim.py: mbsim_swig_pyhton.cc
	test ! -f $@ && (rm -f mbsim_swig_pyhton.cc && $(MAKE) $(AM_MAKEFLAGS) mbsim_swig_pyhton.cc) || :

_mbsim_la_SOURCES = mbsim_swig_pyhton.cc
_mbsim_la_LDFLAGS = -module -shrext $(PYTHON_SHEXT) -avoid-version
_mbsim_la_CPPFLAGS = -Dprotected=public $(PYTHON_CFLAGS) -I$(NUMPY_INCDIR) $(DEPS_CFLAGS) -I$(top_srcdir) -Wno-unused-label
_mbsim_la_LIBADD = $(PYTHON_LIBS) ../mbsim/libmbsim.la

mbsimdir = $(datadir)/mbsim
mbsim_DATA = fmatvec.i mbsim_modules.i showUnwrappedClasses.i

install-exec-hook: mbsim.py
	$(MKDIR_P) $(swigdir)
	$(INSTALL_DATA) mbsim.py $(swigdir)/mbsim.py
uninstall-local:
	rm -f $(swigdir)/mbsim.py

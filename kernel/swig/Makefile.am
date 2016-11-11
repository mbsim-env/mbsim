SUBDIRS = . check

# Note: wee install the resulting python file in $prefix/bin since this is the only place where Windows searches dependent libraries
swigdir = $(bindir)
swig_LTLIBRARIES = _fmatvec.la _mbsim.la

# MISSING: swig has some bugs regarding directors and protected members,
# see https://github.com/swig/swig/issues/828
# As a workaround we define all protected members as public on swig processing and compilation.
# (-Dprotected=public)

CLEANFILES=fmatvec_swig_python.cc fmatvec.py fmatvec_swig_python.d mbsim_swig_python.cc mbsim.py mbsim_swig_python.d

fmatvec_swig_python.cc: $(srcdir)/fmatvec.i
	$(swig) -python -MD -Werror -builtin -c++ -o fmatvec_swig_python.cc $(FMATVEC_CFLAGS) $(srcdir)/fmatvec.i || (rm -f fmatvec_swig_python.cc; exit 1)

mbsim_swig_python.cc: $(srcdir)/mbsim.i $(top_srcdir)/mbsim/objects/*.h
	$(swig) -Dprotected=public $(DEFINES) $(openmbv_compilerflag) "-DBOOST_PARAMETER_MEMBER_FUNCTION(ret,a,b,c)=void swigignore()" -debug-classes -python -MD -Werror -builtin -c++ -o mbsim_swig_python.cc -I$(top_srcdir) -I$(prefix)/share $(DEPS_CFLAGS) $(srcdir)/mbsim.i > swig.stdout || (rm -f mbsim_swig_python.cc; exit 1)
	@echo BEGIN: THE FOLLOWING CLASSES ARE NOT WRAPPED BY SWIG
	@sed -nre "3,\$$s/(.*)/WRAPPED_CLASS(\1)/p" swig.stdout | sed -re "s/<\(/</g;s/\)>/>/g;s/<p\./</g" | grep -v ".*(.*(" > swig.wrapped_classes || (echo ERROR; exit 0)
	@$(swig) -Dprotected=public $(DEFINES) $(openmbv_compilerflag) "-DBOOST_PARAMETER_MEMBER_FUNCTION(ret,a,b,c)=void swigignore()" -DSHOW_UNWRAPPED_CLASSES=swig.wrapped_classes -python -MD -Werror -builtin -c++ -o dummy.cpp -I$(top_srcdir) -I$(prefix)/share $(DEPS_CFLAGS) $(srcdir)/mbsim.i 2>&1 | sed -nre "s/.*: Warning 901: //p" | sort | uniq || (echo ERROR; exit 0)
	@echo END: THE FOLLOWING CLASSES ARE NOT WRAPPED BY SWIG

-include fmatvec_swig_python.d
-include mbsim_swig_python.d

fmatvec.py: fmatvec_swig_python.cc
	test ! -f $@ && (rm -f fmatvec_swig_python.cc && $(MAKE) $(AM_MAKEFLAGS) fmatvec_swig_python.cc) || :

mbsim.py: mbsim_swig_python.cc
	test ! -f $@ && (rm -f mbsim_swig_python.cc && $(MAKE) $(AM_MAKEFLAGS) mbsim_swig_python.cc) || :

_fmatvec_la_SOURCES = fmatvec_swig_python.cc
_fmatvec_la_LDFLAGS = -module -shrext $(PYTHON_SHEXT) -avoid-version
_fmatvec_la_CPPFLAGS = $(PYTHON_CFLAGS) $(FMATVEC_CFLAGS) -I$(NUMPY_INCDIR)
_fmatvec_la_LIBADD = $(PYTHON_LIBS) $(FMATVEC_LIBS)

_mbsim_la_SOURCES = mbsim_swig_python.cc
_mbsim_la_LDFLAGS = -module -shrext $(PYTHON_SHEXT) -avoid-version
_mbsim_la_CPPFLAGS = -Dprotected=public -DMBSIM_COMPILE_SWIG $(PYTHON_CFLAGS) -I$(NUMPY_INCDIR) $(DEPS_CFLAGS) -I$(top_srcdir) -Wno-unused-label
_mbsim_la_LIBADD = $(PYTHON_LIBS) ../mbsim/libmbsim.la

swigdir = $(datadir)/mbsim
swig_DATA = fmatvec.i fmatvec_include.i mbsim.i mbsim_include.i showUnwrappedClasses.i

install-exec-hook: fmatvec.py mbsim.py
	$(MKDIR_P) $(swigdir)
	$(INSTALL_DATA) fmatvec.py $(swigdir)/fmatvec.py mbsim.py $(swigdir)/mbsim.py
uninstall-local:
	rm -f $(swigdir)/fmatvec.py $(swigdir)/mbsim.py

SUBDIRS = .swig_prepare . check

if COND_LOWRAM
.NOTPARALLEL:
endif

# Note: wee install the resulting python file in $prefix/bin since this is the only place where Windows searches dependent libraries
swigbindir = $(bindir)
swigbin_LTLIBRARIES = _fmatvec.la _mbsim.la __mbsim_part100.la __mbsim_part200.la __mbsim_part250.la __mbsim_part300.la

# MISSING: swig has some bugs regarding directors and protected members,
# see https://github.com/swig/swig/issues/828
# As a workaround we define all protected members as public on swig processing and compilation.
# (-Dprotected=public)

BUILT_SOURCES = fmatvec_swig_python.cc mbsim_swig_python.cc _mbsim_part100_swig_python.cc _mbsim_part200_swig_python.cc _mbsim_part250_swig_python.cc _mbsim_part300_swig_python.cc
AM_MAKEFLAGS=SWIGFILES="$(addprefix ../,$(BUILT_SOURCES))" # pass the SWIGFILES variable to .swig_prepare/Makefile

CLEANFILES = fmatvec_swig_python.cc fmatvec.py fmatvec_swig_python.d mbsim_swig_python.cc mbsim.py mbsim_swig_python.d _mbsim_part100_swig_python.cc _mbsim_part100.py _mbsim_part100_swig_python.d _mbsim_part200_swig_python.cc _mbsim_part200.py _mbsim_part200_swig_python.d _mbsim_part250_swig_python.cc _mbsim_part250.py _mbsim_part250_swig_python.d _mbsim_part300_swig_python.cc _mbsim_part300.py _mbsim_part300_swig_python.d

# create a empty dummy fmatvec_swig_python.cc file if MBSIM_SWIG is not set (to avoid a long build time)
fmatvec_swig_python.cc: $(srcdir)/fmatvec.i
	rm -f fmatvec_swig_python.cc
	test -z "$(MBSIM_SWIG)" && (echo "" > $@; echo "" > fmatvec.py) || $(swig) -python -MP -MD -Werror -builtin -c++ -o fmatvec_swig_python.cc $(FMATVEC_CFLAGS) $(srcdir)/fmatvec.i

# create a empty dummy mbsim_swig_python.cc file if MBSIM_SWIG is not set (to avoid a long build time)
%_swig_python.cc: %.i
	rm -f $@
	test -z "$(MBSIM_SWIG)" && (echo "" > $@; echo "" > $*.py) || $(swig) -Dprotected=public $(DEFINES) "-DBOOST_PARAMETER_MEMBER_FUNCTION(ret,a,b,c)=void swigignore()" -debug-classes -python -MP -MD -Werror -builtin -c++ -o $@ -I$(top_srcdir) -I$(prefix)/share $(DEPS_CFLAGS) $< > swig-$*.stdout

# show unwrapped swig classes
swig-unwrapped-local: swig-unwrapped-start mbsim_unwrapped _mbsim_part100_unwrapped _mbsim_part200_unwrapped _mbsim_part250_unwrapped _mbsim_part300_unwrapped
	@echo END: THE FOLLOWING CLASSES ARE NOT WRAPPED BY SWIG
swig-unwrapped-start:
	@echo BEGIN: THE FOLLOWING CLASSES ARE NOT WRAPPED BY SWIG
%_unwrapped: %_swig_python.cc
	@sed -nre "3,\$$s/(.*)/WRAPPED_CLASS(\1)/p" swig-$*.stdout | sed -re "s/<\(/</g;s/\)>/>/g;s/<p\./</g;s/,p\./,/g" | grep -v ".*(.*(" > swig.wrapped_classes || (echo ERROR; exit 0)
	@$(swig) -Dprotected=public $(DEFINES) "-DBOOST_PARAMETER_MEMBER_FUNCTION(ret,a,b,c)=void swigignore()" -DSHOW_UNWRAPPED_CLASSES=swig.wrapped_classes -python -MD -Werror -builtin -c++ -o dummy.cpp -I$(top_srcdir) -I$(prefix)/share $(DEPS_CFLAGS) $(srcdir)/$*.i 2>&1 | sed -nre "s/.*: Warning 901: //p" | sort | uniq || (echo ERROR; exit 0)

-include fmatvec_swig_python.d
-include mbsim_swig_python.d
-include _mbsim_part100_swig_python.d
-include _mbsim_part200_swig_python.d
-include _mbsim_part250_swig_python.d
-include _mbsim_part300_swig_python.d

%.py: %_swig_python.cc
	test ! -f $@ && (rm -f $< && $(MAKE) $(AM_MAKEFLAGS) $<) || :

_fmatvec_la_SOURCES = fmatvec_swig_python.cc
_fmatvec_la_LDFLAGS = -module -shrext $(PYTHON_SHEXT) -avoid-version
_fmatvec_la_CPPFLAGS = $(PYTHON_CFLAGS) $(FMATVEC_CFLAGS) -I$(PYTHON_LIBDIR)/site-packages/numpy/core/include
_fmatvec_la_LIBADD = $(PYTHON_LIBS) $(FMATVEC_LIBS)

_mbsim_la_SOURCES = mbsim_swig_python.cc
_mbsim_la_LDFLAGS = -module -shrext $(PYTHON_SHEXT) -avoid-version
_mbsim_la_CPPFLAGS = -Dprotected=public -DMBSIM_COMPILE_SWIG $(PYTHON_CFLAGS) -I$(PYTHON_LIBDIR)/site-packages/numpy/core/include $(DEPS_CFLAGS) -I$(top_srcdir) -Wno-unused-label -D_hypot=hypot # -D_hypot is a MSVS/python bug workaround
_mbsim_la_LIBADD = $(PYTHON_LIBS) ../mbsim/libmbsim.la

__mbsim_part100_la_SOURCES = _mbsim_part100_swig_python.cc
__mbsim_part100_la_LDFLAGS = -module -shrext $(PYTHON_SHEXT) -avoid-version
__mbsim_part100_la_CPPFLAGS = -Dprotected=public -DMBSIM_COMPILE_SWIG $(PYTHON_CFLAGS) -I$(PYTHON_LIBDIR)/site-packages/numpy/core/include $(DEPS_CFLAGS) -I$(top_srcdir) -Wno-unused-label -D_hypot=hypot # -D_hypot is a MSVS/python bug workaround
__mbsim_part100_la_LIBADD = $(PYTHON_LIBS) ../mbsim/libmbsim.la

__mbsim_part200_la_SOURCES = _mbsim_part200_swig_python.cc
__mbsim_part200_la_LDFLAGS = -module -shrext $(PYTHON_SHEXT) -avoid-version
__mbsim_part200_la_CPPFLAGS = -Dprotected=public -DMBSIM_COMPILE_SWIG $(PYTHON_CFLAGS) -I$(PYTHON_LIBDIR)/site-packages/numpy/core/include $(DEPS_CFLAGS) -I$(top_srcdir) -Wno-unused-label -D_hypot=hypot # -D_hypot is a MSVS/python bug workaround
__mbsim_part200_la_LIBADD = $(PYTHON_LIBS) ../mbsim/libmbsim.la

__mbsim_part250_la_SOURCES = _mbsim_part250_swig_python.cc
__mbsim_part250_la_LDFLAGS = -module -shrext $(PYTHON_SHEXT) -avoid-version
__mbsim_part250_la_CPPFLAGS = -Dprotected=public -DMBSIM_COMPILE_SWIG $(PYTHON_CFLAGS) -I$(PYTHON_LIBDIR)/site-packages/numpy/core/include $(DEPS_CFLAGS) -I$(top_srcdir) -Wno-unused-label -D_hypot=hypot # -D_hypot is a MSVS/python bug workaround
__mbsim_part250_la_LIBADD = $(PYTHON_LIBS) ../mbsim/libmbsim.la

__mbsim_part300_la_SOURCES = _mbsim_part300_swig_python.cc
__mbsim_part300_la_LDFLAGS = -module -shrext $(PYTHON_SHEXT) -avoid-version
__mbsim_part300_la_CPPFLAGS = -Dprotected=public -DMBSIM_COMPILE_SWIG $(PYTHON_CFLAGS) -I$(PYTHON_LIBDIR)/site-packages/numpy/core/include $(DEPS_CFLAGS) -I$(top_srcdir) -Wno-unused-label -D_hypot=hypot # -D_hypot is a MSVS/python bug workaround
__mbsim_part300_la_LIBADD = $(PYTHON_LIBS) ../mbsim/libmbsim.la

swigdatadir = $(datadir)/mbsim
swigdata_DATA = fmatvec.i fmatvec_include.i mbsim.i _mbsim_part100.i _mbsim_part200.i _mbsim_part250.i _mbsim_part300.i mbsim_include.i showUnwrappedClasses.i

install-exec-hook: fmatvec.py mbsim.py _mbsim_part100.py _mbsim_part200.py _mbsim_part250.py _mbsim_part300.py
	$(MKDIR_P) $(swigbindir)
	$(INSTALL_DATA) fmatvec.py $(swigbindir)/fmatvec.py
	$(INSTALL_DATA) mbsim.py $(swigbindir)/mbsim.py
	$(INSTALL_DATA) _mbsim_part100.py $(swigbindir)/_mbsim_part100.py
	$(INSTALL_DATA) _mbsim_part200.py $(swigbindir)/_mbsim_part200.py
	$(INSTALL_DATA) _mbsim_part250.py $(swigbindir)/_mbsim_part250.py
	$(INSTALL_DATA) _mbsim_part300.py $(swigbindir)/_mbsim_part300.py
uninstall-local:
	rm -f $(swigbindir)/fmatvec.py $(swigbindir)/mbsim.py $(swigbindir)/_mbsim_part100.py $(swigbindir)/_mbsim_part200.py $(swigbindir)/_mbsim_part250.py $(swigbindir)/_mbsim_part300.py

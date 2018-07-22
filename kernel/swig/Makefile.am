AUTOMAKE_OPTIONS = -Wno-portability

SUBDIRS = .swig_prepare . check

if COND_LOWRAM
.NOTPARALLEL:
endif

MBSIM_PARTS_LA = \
  __mbsim_part050.la \
  __mbsim_part100.la \
  __mbsim_part125.la \
  __mbsim_part150.la \
  __mbsim_part200.la \
  __mbsim_part250.la \
  __mbsim_part300.la \
  __mbsim_part350.la

MBSIM_PARTS = $(patsubst _%.la,%,$(MBSIM_PARTS_LA))

# Note: wee install the resulting python file in $prefix/bin since this is the only place where Windows searches dependent libraries
swigbindir = $(bindir)
swigbin_LTLIBRARIES = _fmatvec.la _mbsim.la $(MBSIM_PARTS_LA)

# MISSING: swig has some bugs regarding directors and protected members,
# see https://github.com/swig/swig/issues/828
# As a workaround we define all protected members as public on swig processing and compilation.
# (-Dprotected=public)

BUILT_SOURCES = _fmatvec_swig_python.cc _mbsim_swig_python.cc $(addprefix _,$(addsuffix _swig_python.cc,$(MBSIM_PARTS)))
AM_MAKEFLAGS=SWIGFILES="$(addprefix ../,$(BUILT_SOURCES))" # pass the SWIGFILES variable to .swig_prepare/Makefile

CLEANFILES = _fmatvec_swig_python.cc fmatvec.py fmatvec_swig_python.d _mbsim_swig_python.cc mbsim.py mbsim_swig_python.d \
  $(addprefix _,$(addsuffix _swig_python.cc,$(MBSIM_PARTS))) \
  $(addsuffix .py,$(MBSIM_PARTS)) \
  $(addsuffix _swig_python.d,$(MBSIM_PARTS))

# create a empty dummy _fmatvec_swig_python.cc file if MBSIM_SWIG is not set (to avoid a long build time)
_fmatvec_swig_python.cc: $(srcdir)/fmatvec.i
	rm -f _fmatvec_swig_python.cc
	test -z "$(MBSIM_SWIG)" && (echo "" > $@; echo "" > fmatvec.py) || $(swig) -python -MP -MD -Werror -builtin -c++ -o _fmatvec_swig_python.cc $(FMATVEC_CFLAGS) $(srcdir)/fmatvec.i

# create a empty dummy _mbsim_swig_python.cc file if MBSIM_SWIG is not set (to avoid a long build time)
_%_swig_python.cc: %.i
	rm -f $@
	test -z "$(MBSIM_SWIG)" && (echo "" > $@; echo "" > $*.py) || $(swig) -Dprotected=public $(DEFINES) "-DBOOST_PARAMETER_MEMBER_FUNCTION(ret,a,b,c)=void swigignore()" -debug-classes -python -MP -MD -Werror -builtin -c++ -o $@ -I$(top_srcdir) -I$(prefix)/share $(DEPS_CFLAGS) $< > swig-$*.stdout

# show unwrapped swig classes
swig-unwrapped-local: swig-unwrapped-start mbsim_unwrapped $(addsuffix _unwrapped,$(MBSIM_PARTS))
	@echo END: THE FOLLOWING CLASSES ARE NOT WRAPPED BY SWIG
swig-unwrapped-start:
	@echo BEGIN: THE FOLLOWING CLASSES ARE NOT WRAPPED BY SWIG
%_unwrapped: _%_swig_python.cc
	@sed -nre "3,\$$s/(.*)/WRAPPED_CLASS(\1)/p" swig-$*.stdout | sed -re "s/<\(/</g;s/\)>/>/g;s/<p\./</g;s/,p\./,/g" | grep -v ".*(.*(" > swig.wrapped_classes || (echo ERROR; exit 0)
	@$(swig) -Dprotected=public $(DEFINES) "-DBOOST_PARAMETER_MEMBER_FUNCTION(ret,a,b,c)=void swigignore()" -DSHOW_UNWRAPPED_CLASSES=swig.wrapped_classes -python -MD -Werror -builtin -c++ -o dummy.cpp -I$(top_srcdir) -I$(prefix)/share $(DEPS_CFLAGS) $(srcdir)/$*.i 2>&1 | sed -nre "s/.*: Warning 901: //p" | sort | uniq || (echo ERROR; exit 0)

-include fmatvec_swig_python.d
-include mbsim_swig_python.d
$(foreach PART,$(MBSIM_PARTS),$(eval -include $(PART)_swig_python.d))

%.py: _%_swig_python.cc
	test ! -f $@ && (rm -f $< && $(MAKE) $(AM_MAKEFLAGS) $<) || :

# source files for libtool libraries
_fmatvec_la_SOURCES = _fmatvec_swig_python.cc
_mbsim_la_SOURCES = _mbsim_swig_python.cc
# set default source for all __mbsim_partXXX.la targets to _mbsim_partXXX_swig_python.cc
AM_DEFAULT_SOURCE_EXT = _swig_python.cc

# compile flags for libtool libraries
_fmatvec_la_CPPFLAGS = $(PYTHON_CFLAGS) $(FMATVEC_CFLAGS) -I$(PYTHON_LIBDIR)/site-packages/numpy/core/include
_mbsim_la_CPPFLAGS = $(AM_CPPFLAGS)
# set default compile flags for all __mbsim_partXXX.la targets
AM_CPPFLAGS = -Dprotected=public -DMBSIM_COMPILE_SWIG $(PYTHON_CFLAGS) -I$(PYTHON_LIBDIR)/site-packages/numpy/core/include \
  $(DEPS_CFLAGS) -I$(top_srcdir) -Wno-unused-label -D_hypot=hypot # -D_hypot is a MSVS/python bug workaround

# link flags for libtool libraries
_fmatvec_la_LDFLAGS = -module -shrext $(PYTHON_SHEXT) -avoid-version $(PYTHON_LIBS) $(FMATVEC_LIBS)
_mbsim_la_LDFLAGS = $(AM_LDFLAGS)
# set default link flags for all __mbsim_partXXX.la targets
AM_LDFLAGS = -module -shrext $(PYTHON_SHEXT) -avoid-version $(PYTHON_LIBS) $(DEPS_LIBS) ../mbsim/libmbsim.la

swigdatadir = $(datadir)/mbsim
swigdata_DATA = fmatvec.i fmatvec_include.i mbsim.i $(addsuffix .i,$(MBSIM_PARTS)) mbsim_include.i showUnwrappedClasses.i

install-exec-hook: fmatvec.py mbsim.py $(addsuffix .py,$(MBSIM_PARTS))
	$(MKDIR_P) $(swigbindir)
	$(INSTALL_DATA) fmatvec.py $(swigbindir)/fmatvec.py
	$(INSTALL_DATA) mbsim.py $(swigbindir)/mbsim.py
	for F in $(MBSIM_PARTS); do \
	  $(INSTALL_DATA) $$F.py $(swigbindir)/$$F.py; \
	done
uninstall-local:
	rm -f $(swigbindir)/fmatvec.py $(swigbindir)/mbsim.py
	for F in $(MBSIM_PARTS); do \
	  rm -f $(swigbindir)/$$F.py; \
	done

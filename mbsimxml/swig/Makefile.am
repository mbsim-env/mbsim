AUTOMAKE_OPTIONS = -Wno-portability

SUBDIRS = .swig_prepare .

if COND_LOWRAM
.NOTPARALLEL:
endif

# Note: wee install the resulting python file in $prefix/bin since this is the only place where Windows searches dependent libraries
swigbindir = $(bindir)
swigbin_LTLIBRARIES = _mbsimxml.la

# MISSING: swig has some bugs regarding directors and protected members,
# see https://github.com/swig/swig/issues/828
# As a workaround we define all protected members as public on swig processing and compilation.
# (-Dprotected=public)

BUILT_SOURCES = _mbsimxml_swig_python.cc
AM_MAKEFLAGS=SWIGFILES="$(addprefix ../,$(BUILT_SOURCES))" # pass the SWIGFILES variable to .swig_prepare/Makefile

CLEANFILES = _mbsimxml_swig_python.cc mbsimxml.py mbsimxml_swig_python.d

# create a empty dummy _fmatvec_swig_python.cc file if MBSIM_SWIG is not set (to avoid a long build time)
.DELETE_ON_ERROR: # swig -Werror requires this

# create a empty dummy _mbsimxml_swig_python.cc file if MBSIM_SWIG is not set (to avoid a long build time)
_%_swig_python.cc: %.i
	rm -f $@
	test -z "$(MBSIM_SWIG)" && (echo "" > $@; echo "" > $*.py) || $(swig) -Dprotected=public $(DEFINES) "-DBOOST_PARAMETER_MEMBER_FUNCTION(ret,a,b,c)=void swigignore()" -debug-classes -python -MP -MD -Werror -builtin -c++ -o $@ -I$(top_srcdir) -I$(prefix)/share $(DEPS_CFLAGS) $< > swig-$*.stdout

-include $(builddir)/_mbsimxml_swig_python.d
#
%.py: _%_swig_python.cc
	test ! -f $@ && (rm -f $< && $(MAKE) $(AM_MAKEFLAGS) $<) || :

# source files for libtool libraries
_mbsimxml_la_SOURCES = _mbsimxml_swig_python.cc
# set default source for all __mbsimxml_partXXX.la targets to _mbsimxml_partXXX_swig_python.cc
AM_DEFAULT_SOURCE_EXT = _swig_python.cc

# compile flags for libtool libraries
_mbsimxml_la_CPPFLAGS = $(AM_CPPFLAGS)
# set default compile flags for all __mbsim_partXXX.la targets
AM_CPPFLAGS = -I$(top_srcdir) -Dprotected=public -DMBSIM_COMPILE_SWIG $(PYTHON_CFLAGS) \
  -I$(PYTHON_LIBDIR)/site-packages/numpy/core/include \
  -Wno-error=unused-variable -Wno-error=write-strings -Wno-error=virtual-move-assign $(CXXFLAGSNOWERRORSTRINGOP) \
  $(DEPS_CFLAGS) -Wno-unused-label -Wno-array-bounds -D_hypot=hypot # -D_hypot is a MSVS/python bug workaround

# link flags for libtool libraries
_mbsimxml_la_LDFLAGS = $(AM_LDFLAGS) ../mbsimxml/libmbsimflatxml.la ../mbsimxml/libmbsimxml.la $(PYCPPWRAPPER_LIBS)
# set default link flags for all __mbsimxml_partXXX.la targets
AM_LDFLAGS = -module -shrext $(PYTHON_SHEXT) -avoid-version ../mbsimxml/libmbsimflatxml.la  $(PYCPPWRAPPER_LIBS) $(DEPS_LIBS)

swigdatadir = $(datadir)/mbsimxml
swigdata_DATA = mbsimxml.i

install-exec-hook: mbsimxml.py
	$(MKDIR_P) $(swigbindir)
	$(INSTALL_DATA) mbsimxml.py $(swigbindir)/mbsimxml.py
uninstall-local:
	rm -f $(swigbindir)/mbsimxml.py

if COND_MINGW
  staticLink = -static
  shext = .dll
else
  shext = .so
  staticLink = -static-libgcc -static-libstdc++
endif

noinst_LTLIBRARIES = libmbsim_fmi.la libmbsimwrapper.la
lib_LTLIBRARIES = libmbsimxml_fmi.la libmbsimppxml_fmi.la libmbsimsrc_fmi.la

# build FMU library; just a wrapper which loads at runtime libmbsimXXX_fmi.la); this library
# should NOT depend on any library except glibc ones!
libmbsimwrapper_la_SOURCES = fmiModelFunctions.cc
libmbsimwrapper_la_CPPFLAGS = $(MBXMLUTILSHELPERDEPS_CFLAGS) -fvisibility=hidden
# the mbsim$(shext) file needs very special linking
.libs/mbsim$(shext): libmbsimwrapper.la
	$(CXX) $(AM_CXXFLAGS) $(CXXFLAGS) $(LDFLAGS_NOLIBTOOL) -shared $(staticLink) \
	-Wl,-u,mbsim_fmiGetVersion,--version-script=$(srcdir)/mbsim.map -o .libs/mbsim$(shext) .libs/libmbsimwrapper.a $(LIBDL)
mbsimdir = $(libdir)
mbsim_DATA = .libs/mbsim$(shext)
CLEANFILES = .libs/mbsim$(shext)



# build main FMU library (common part; static archive)
libmbsim_fmi_la_SOURCES = fmiinstance.cc utils.cc
libmbsim_fmi_la_CPPFLAGS = $(FMUDEPS_CFLAGS)

# build main FMU library for XML models
libmbsimxml_fmi_la_SOURCES = fmiinstance_xml.cc
libmbsimxml_fmi_la_CPPFLAGS = $(FMUXMLDEPS_CFLAGS)
libmbsimxml_fmi_la_LIBADD = libmbsim_fmi.la $(FMUXMLDEPS_LIBS) ../general/libfmivariables.la $(FMUDEPS_LIBS)
libmbsimxml_fmi_la_LDFLAGS = -avoid-version

# build main FMU library for XML models with parameters (preprocessing is included in the FMU)
libmbsimppxml_fmi_la_SOURCES = fmiinstance_ppxml.cc
libmbsimppxml_fmi_la_CPPFLAGS = $(FMUPPXMLDEPS_CFLAGS)
libmbsimppxml_fmi_la_LIBADD = libmbsim_fmi.la ../general/libxmlpputils.la $(FMUPPXMLDEPS_LIBS) ../general/libfmivariables.la $(FMUDEPS_LIBS)
libmbsimppxml_fmi_la_LDFLAGS = -avoid-version

# build main FMU library for source models
libmbsimsrc_fmi_la_SOURCES = fmiinstance_src.cc
libmbsimsrc_fmi_la_CPPFLAGS = $(FMUDEPS_CFLAGS) $(MBXMLUTILSHELPERDEPS_CFLAGS)
libmbsimsrc_fmi_la_LIBADD = libmbsim_fmi.la ../general/libfmivariables.la $(FMUDEPS_LIBS)
libmbsimsrc_fmi_la_LDFLAGS = -avoid-version


include $(prefix)/share/mbxmlutils/python/deplibs.mk
install-exec-hook: deplibs.target

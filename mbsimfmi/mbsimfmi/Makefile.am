noinst_LTLIBRARIES = libmbsim_fmi.la
lib_LTLIBRARIES = mbsim.la libmbsimxml_fmi.la libmbsimppxml_fmi.la libmbsimsrc_fmi.la

# build FMU library; just a wrapper which loads at runtime libmbsimXXX_fmi.la); this library
# should NOT depend on any library except system libraries!
mbsim_la_SOURCES = fmiModelFunctions.cc
mbsim_la_CPPFLAGS = $(MBXMLUTILSBASEDEPS_CFLAGS)
mbsim_la_LDFLAGS = -module -avoid-version -Wl,--disable-new-dtags,-rpath,\$$ORIGIN
mbsim_la_LIBADD = $(MBXMLUTILSBASEDEPS_LIBS)



# build main FMU library (common part; static archive)
libmbsim_fmi_la_SOURCES = fmiinstance.cc utils.cc
libmbsim_fmi_la_CPPFLAGS = $(FMUDEPS_CFLAGS)

# build main FMU library for XML models
libmbsimxml_fmi_la_SOURCES = fmiinstance_xml.cc
libmbsimxml_fmi_la_LDFLAGS = -Wl,--disable-new-dtags,-rpath,\$$ORIGIN/../lib
libmbsimxml_fmi_la_CPPFLAGS = $(FMUXMLDEPS_CFLAGS)
libmbsimxml_fmi_la_LIBADD = libmbsim_fmi.la $(FMUXMLDEPS_LIBS) ../general/libfmivariables.la $(FMUDEPS_LIBS)

# build main FMU library for XML models with parameters (preprocessing is included in the FMU)
libmbsimppxml_fmi_la_SOURCES = fmiinstance_ppxml.cc
libmbsimppxml_fmi_la_LDFLAGS = -Wl,--disable-new-dtags,-rpath,\$$ORIGIN/../lib
libmbsimppxml_fmi_la_CPPFLAGS = $(FMUPPXMLDEPS_CFLAGS)
libmbsimppxml_fmi_la_LIBADD = libmbsim_fmi.la ../general/libxmlpputils.la $(FMUPPXMLDEPS_LIBS) ../general/libfmivariables.la $(FMUDEPS_LIBS)

# build main FMU library for source models
libmbsimsrc_fmi_la_SOURCES = fmiinstance_src.cc
libmbsimsrc_fmi_la_LDFLAGS = -Wl,--disable-new-dtags,-rpath,\$$ORIGIN/../lib
libmbsimsrc_fmi_la_CPPFLAGS = $(FMUDEPS_CFLAGS) $(MBXMLUTILSBASEDEPS_CFLAGS)
libmbsimsrc_fmi_la_LIBADD = libmbsim_fmi.la ../general/libfmivariables.la $(FMUDEPS_LIBS) $(MBXMLUTILSBASEDEPS_LIBS)


include $(prefix)/share/mbxmlutils/python/deplibs.mk
install-exec-hook: deplibs.target

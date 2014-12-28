# the main binary library to be packed into the FMU (named mbsim.so or mbsim.dll)
noinst_LTLIBRARIES = libmbsim_fmi.la
lib_LTLIBRARIES = mbsimxml_fmi.la mbsimppxml_fmi.la mbsimsrc_fmi.la

# build main FMU library (common part)
libmbsim_fmi_la_SOURCES = fmiModelFunctions.cc fmiinstance.cc utils.cc
libmbsim_fmi_la_CPPFLAGS = $(FMUDEPS_CFLAGS)
libmbsim_fmi_la_LIBADD = ../general/libfmivariables.la $(FMUDEPS_LIBS)

# build main FMU library for XML models
mbsimxml_fmi_la_SOURCES = fmiinstance_xml.cc
mbsimxml_fmi_la_LDFLAGS = -module -Wl,-rpath,\$$ORIGIN -avoid-version
mbsimxml_fmi_la_CPPFLAGS = $(FMUXMLDEPS_CFLAGS)
mbsimxml_fmi_la_LIBADD = libmbsim_fmi.la $(FMUXMLDEPS_LIBS)

# build main FMU library for XML models with parameters (preprocessing is included in the FMU)
mbsimppxml_fmi_la_SOURCES = fmiinstance_ppxml.cc
mbsimppxml_fmi_la_LDFLAGS = -module -Wl,-rpath,\$$ORIGIN -avoid-version
mbsimppxml_fmi_la_CPPFLAGS = $(FMUPPXMLDEPS_CFLAGS)
mbsimppxml_fmi_la_LIBADD = libmbsim_fmi.la ../general/libxmlpputils.la $(FMUPPXMLDEPS_LIBS)

# build main FMU library for source models
mbsimsrc_fmi_la_SOURCES = fmiinstance_src.cc
mbsimsrc_fmi_la_LDFLAGS = -module -Wl,-rpath,\$$ORIGIN -avoid-version
mbsimsrc_fmi_la_CPPFLAGS = $(FMUDEPS_CFLAGS)
mbsimsrc_fmi_la_LIBADD = libmbsim_fmi.la

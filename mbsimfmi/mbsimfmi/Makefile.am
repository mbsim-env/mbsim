# the main binary library to be packed into the FMU (named mbsim.so or mbsim.dll)
lib_LTLIBRARIES = mbsimxml_fmi.la mbsimppxml_fmi.la mbsimsrc_fmi.la

# build main FMU library for XML models (use -DMBSIMXMLFMI)
mbsimxml_fmi_la_SOURCES = fmiModelFunctions.cc fmiinstance.cc utils.cc
mbsimxml_fmi_la_LDFLAGS = -module -Wl,-rpath,\$$ORIGIN -avoid-version
mbsimxml_fmi_la_CPPFLAGS = -DMBSIMXMLFMI $(FMUXMLDEPS_CFLAGS)
mbsimxml_fmi_la_LIBADD = ../general/libfmivariables.la $(FMUXMLDEPS_LIBS)

# build main FMU library for XML models with parameters (preprocessing is included in the FMU) (use -DMBSIMPPXMLFMI)
mbsimppxml_fmi_la_SOURCES = fmiModelFunctions.cc fmiinstance.cc utils.cc
mbsimppxml_fmi_la_LDFLAGS = -module -Wl,-rpath,\$$ORIGIN -avoid-version
mbsimppxml_fmi_la_CPPFLAGS = -DMBSIMPPXMLFMI $(FMUPPXMLDEPS_CFLAGS)
mbsimppxml_fmi_la_LIBADD = ../general/libfmivariables.la ../general/libxmlpputils.la $(FMUPPXMLDEPS_LIBS)

# build main FMU library for source models (use -DMBSIMSRCFMI)
mbsimsrc_fmi_la_SOURCES = fmiModelFunctions.cc fmiinstance.cc utils.cc
mbsimsrc_fmi_la_LDFLAGS = -module -Wl,-rpath,\$$ORIGIN -avoid-version
mbsimsrc_fmi_la_CPPFLAGS = -DMBSIMSRCFMI $(FMUSRCDEPS_CFLAGS)
mbsimsrc_fmi_la_LIBADD = ../general/libfmivariables.la $(FMUSRCDEPS_LIBS)

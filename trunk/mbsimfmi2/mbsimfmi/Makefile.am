# the main binary library to be packed into the FMU (named mbsim.so or mbsim.dll)
lib_LTLIBRARIES = mbsimxml_fmi.la

mbsimxml_fmi_la_SOURCES = fmiModelFunctions.cc fmiinstance.cc utils.cc
mbsimxml_fmi_la_LDFLAGS = -module -Wl,-rpath,\$$ORIGIN -avoid-version
mbsimxml_fmi_la_CPPFLAGS = $(FMUDEPS_CFLAGS)
mbsimxml_fmi_la_LIBADD = $(FMUDEPS_LIBS)

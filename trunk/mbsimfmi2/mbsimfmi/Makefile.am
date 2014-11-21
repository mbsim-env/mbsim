# the main binary library to be packed into the FMU (named mbsim.so or mbsim.dll)
lib_LTLIBRARIES = mbsimfmi.la

mbsimfmi_la_SOURCES = fmiModelFunctions.cc fmiinstance.cc utils.cc
mbsimfmi_la_LDFLAGS = -module -Wl,-rpath,\$$ORIGIN -avoid-version
mbsimfmi_la_CPPFLAGS = $(FMUDEPS_CFLAGS)
mbsimfmi_la_LIBADD = $(FMUDEPS_LIBS)

SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimElectronics.la
libmbsimElectronics_la_LDFLAGS = -version-info @LIBMBSIMELECTRONICS_VERSION_INFO@
libmbsimElectronics_la_SOURCES = simulation_classes.cc\
			    modelling_classes.cc

libmbsimElectronics_la_LIBADD = $(DEPS_LIBS)
INCLUDES = $(DEPS_CFLAGS)

mbsimElectronicsincludedir = $(includedir)/mbsimElectronics
mbsimElectronicsinclude_HEADERS = simulation_classes.h\
			     modelling_classes.h




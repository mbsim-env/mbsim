SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimElectronics.la
libmbsimElectronics_la_LDFLAGS = -version-info @LIBMBSIMELECTRONICS_VERSION_INFO@
libmbsimElectronics_la_SOURCES = simulation_classes.cc\
			    modeling_classes.cc

libmbsimElectronics_la_LIBADD = 
INCLUDES = $(DEPS_CFLAGS)
LDADD = $(DEPS_LIBS)

mbsimElectronicsincludedir = $(includedir)/mbsimElectronics
mbsimElectronicsinclude_HEADERS = simulation_classes.h\
			     modeling_classes.h




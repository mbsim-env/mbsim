SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimelectronics.la
libmbsimelectronics_la_LDFLAGS = -version-info @LIBMBSIMELECTRONICS_VERSION_INFO@
libmbsimelectronics_la_SOURCES = simulation_classes.cc\
			    modeling_classes.cc

libmbsimelectronics_la_LIBADD = 
INCLUDES = $(DEPS_CFLAGS)
LDADD = $(DEPS_LIBS)

mbsimelectronicsincludedir = $(includedir)/mbsimelectronics
mbsimelectronicsinclude_HEADERS = simulation_classes.h\
			     modeling_classes.h




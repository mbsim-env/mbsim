SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimHydraulics.la
libmbsimHydraulics_la_LDFLAGS = -version-info @LIBMBSIMHYDRAULICS_VERSION_INFO@
libmbsimHydraulics_la_SOURCES = \
  hydline.cc \
  hydline_galerkin.cc \
  hydnode.cc \
  hydnode_mec.cc \
  objectfactory.cc \
  environment.cc \
  pressure_loss.cc \
	hydline_pressureloss.cc \
	controlvalve43.cc \
	checkvalve.cc

libmbsimHydraulics_la_LIBADD = 
INCLUDES = $(DEPS_CFLAGS)
LDADD = $(DEPS_LIBS)

mbsimHydraulicsincludedir = $(includedir)/mbsimHydraulics
mbsimHydraulicsinclude_HEADERS = \
  hydline.h \
  hydline_galerkin.h \
  hydnode.h \
  hydnode_mec.h \
  object_hydraulics.h \
  objectfactory.h \
  environment.h \
  pressure_loss.h \
	hydline_pressureloss.h \
	controlvalve43.h \
	checkvalve.h

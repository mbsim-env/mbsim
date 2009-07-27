SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimHydraulics.la
libmbsimHydraulics_la_LDFLAGS = -version-info @LIBMBSIMHYDRAULICS_VERSION_INFO@
libmbsimHydraulics_la_SOURCES = \
  hydleakage.cc \
  hydline.cc \
  hydline_closed.cc \
  hydline_galerkin.cc \
  hydnode.cc \
  hydnode_mec.cc \
  object_hydraulics.cc \
  oil.cc \
  valve43.cc \
  objectfactory.cc \
  environment.cc

libmbsimHydraulics_la_LIBADD = 
INCLUDES = $(DEPS_CFLAGS)
LDADD = $(DEPS_LIBS)

mbsimHydraulicsincludedir = $(includedir)/mbsimHydraulics
mbsimHydraulicsinclude_HEADERS = \
  hydleakage.h \
  hydline.h \
  hydline_closed.h \
  hydline_galerkin.h \
  hydnode.h \
  hydnode_mec.h \
  object_hydraulics.h \
  oil.h \
  valve43.h \
  objectfactory.h \
  environment.h

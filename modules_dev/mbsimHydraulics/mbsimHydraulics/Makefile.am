SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimHydraulics.la
libmbsimHydraulics_la_LDFLAGS = -version-info @LIBMBSIMHYDRAULICS_VERSION_INFO@
libmbsimHydraulics_la_SOURCES = \
  hydfluid.cc \
  hydleakage.cc \
  hydline.cc \
  hydline_closed.cc \
  hydline_galerkin.cc \
  hydnode.cc \
  hydnode_mec.cc \
  object_hydraulics.cc \
  oil.cc \
  valve43.cc

libmbsimHydraulics_la_LIBADD = 
INCLUDES = $(DEPS_CFLAGS)
LDADD = $(DEPS_LIBS)

mbsimHydraulicsincludedir = $(includedir)/mbsimHydraulics
mbsimHydraulicsinclude_HEADERS = \
  hydfluid.h \
  hydleakage.h \
  hydline.h \
  hydline_closed.h \
  hydline_galerkin.h \
  hydnode.h \
  hydnode_mec.h \
  object_hydraulics.h \
  oil.h \
  valve43.h

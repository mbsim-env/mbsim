SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimHydraulics.la
libmbsimHydraulics_la_LDFLAGS = -version-info @LIBMBSIMHYDRAULICS_VERSION_INFO@
libmbsimHydraulics_la_SOURCES = \
  environment.cc \
  hline.cc \
  rigid_line.cc \
	rigid_line_pressureloss.cc \
  hnode.cc \
  hnode_mec.cc \
  pressure_loss.cc \
	controlvalve43.cc \
	checkvalve.cc \
	leakage_line.cc \
  elastic_line_galerkin.cc \
  elastic_line_variational.cc \
  objectfactory.cc \
	dimensionless_line.cc \
	hydraulic_sensor.cc \
	obsolet_hint.cc

libmbsimHydraulics_la_CPPFLAGS = -DTIXML_USE_STL
libmbsimHydraulics_la_LIBADD = $(DEPS_LIBS)
INCLUDES = $(DEPS_CFLAGS)

mbsimHydraulicsincludedir = $(includedir)/mbsimHydraulics
mbsimHydraulicsinclude_HEADERS = \
  environment.h \
  hline.h \
  rigid_line.h \
	rigid_line_pressureloss.h \
  hnode.h \
  hnode_mec.h \
  pressure_loss.h \
	controlvalve43.h \
	checkvalve.h \
	leakage_line.h \
  elastic_line_galerkin.h \
  elastic_line_variational.h \
  objectfactory.h \
	dimensionless_line.h \
	hydraulic_sensor.h


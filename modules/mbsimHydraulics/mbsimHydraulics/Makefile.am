SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimHydraulics.la
libmbsimHydraulics_la_LDFLAGS = -avoid-version
libmbsimHydraulics_la_SOURCES = \
  environment.cc \
  hline.cc \
  rigid_line.cc \
  hnode.cc \
  hnode_mec.cc \
  pressure_loss.cc \
  leakage_line.cc \
  dimensionless_line.cc \
  rigid_line_pressureloss.cc \
  controlvalve43.cc \
  checkvalve.cc \
  hydraulic_sensor.cc \
  elastic_line_galerkin.cc \
  elastic_line_variational.cc

libmbsimHydraulics_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS)
libmbsimHydraulics_la_LIBADD = $(DEPS_LIBS)

mbsimHydraulicsincludedir = $(includedir)/mbsimHydraulics
mbsimHydraulicsinclude_HEADERS = \
  environment.h \
  hline.h \
  rigid_line.h \
  hnode.h \
  hnode_mec.h \
  pressure_loss.h \
  leakage_line.h \
  dimensionless_line.h \
  rigid_line_pressureloss.h \
  controlvalve43.h \
  checkvalve.h \
  hydraulic_sensor.h \
  elastic_line_galerkin.h \
  elastic_line_variational.h

include $(prefix)/share/mbxmlutils/python/deplibs.mk
install-exec-hook: deplibs.target

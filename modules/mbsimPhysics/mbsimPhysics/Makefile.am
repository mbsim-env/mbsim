SUBDIRS = functions .
  
lib_LTLIBRARIES = libmbsimPhysics.la
libmbsimPhysics_la_LDFLAGS = -avoid-version
libmbsimPhysics_la_SOURCES = \
  universal_gravitation.cc \
  weight.cc \
  buoyancy.cc \
  drag.cc \
  aerodynamics.cc

libmbsimPhysics_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS)
libmbsimPhysics_la_LIBADD = functions/libfunction.la
libmbsimPhysics_la_LIBADD += $(DEPS_LIBS)

mbsimPhysicsincludedir = $(includedir)/mbsimPhysics
mbsimPhysicsinclude_HEADERS = \
  namespace.h \
  universal_gravitation.h \
  weight.h \
  buoyancy.h \
  drag.h \
  aerodynamics.h

include $(prefix)/share/mbxmlutils/python/deplibs.mk
install-exec-hook: deplibs.target

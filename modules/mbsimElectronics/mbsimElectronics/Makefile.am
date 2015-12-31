SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimElectronics.la
libmbsimElectronics_la_LDFLAGS = -avoid-version
libmbsimElectronics_la_SOURCES = simulation_classes.cc\
			    modelling_classes.cc

libmbsimElectronics_la_LIBADD = $(DEPS_LIBS)
libmbsimElectronics_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS)

mbsimElectronicsincludedir = $(includedir)/mbsimElectronics
mbsimElectronicsinclude_HEADERS = simulation_classes.h\
			     modelling_classes.h


include $(prefix)/share/mbxmlutils/python/deplibs.mk
install-exec-hook: deplibs.target

SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimPowertrain.la
libmbsimPowertrain_la_LDFLAGS = -version-info @LIBMBSIMELECTRONICS_VERSION_INFO@
libmbsimPowertrain_la_SOURCES = planetary_gear.cc\
			    differential_gear.cc\
			    cardan_shaft.cc

libmbsimPowertrain_la_LIBADD = 
INCLUDES = $(DEPS_CFLAGS)
LDADD = $(DEPS_LIBS)

mbsimPowertrainincludedir = $(includedir)/mbsimPowertrain
mbsimPowertraininclude_HEADERS = planetary_gear.h\
			     differential_gear.h\
			     cardan_shaft.h




SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimPowertrain.la
libmbsimPowertrain_la_LDFLAGS = -version-info @LIBMBSIMELECTRONICS_VERSION_INFO@
libmbsimPowertrain_la_SOURCES = planetary_gear.cc\
			    differential_gear.cc\
			    cardan_shaft.cc\
			    torsional_stiffness.cc

libmbsimPowertrain_la_LIBADD = $(DEPS_LIBS)
libmbsimPowertrain_la_CPPFLAGS = $(DEPS_CFLAGS)

mbsimPowertrainincludedir = $(includedir)/mbsimPowertrain
mbsimPowertraininclude_HEADERS = planetary_gear.h\
			     differential_gear.h\
			     cardan_shaft.h\
			     torsional_stiffness.h
